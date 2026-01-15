#!/usr/bin/env python3
import threading
import time
from pathlib import Path
from typing import Optional, Union

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool


def _default_video_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'videos'
    return Path.home() / 'videos'


class UsbCamStreamNode(Node):
    def __init__(self) -> None:
        super().__init__('usb_cam_stream')
        self.declare_parameter('device_index', 0)
        self.declare_parameter('device_path', '')
        self.declare_parameter('image_topic', 'nikon/image_raw')
        self.declare_parameter('frame_id', 'nikon_camera')
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('fps', 0.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('use_v4l2', True)
        self.declare_parameter('reconnect_delay_sec', 1.0)
        self.declare_parameter('encoding', 'bgr8')
        self.declare_parameter('video_output_dir', str(_default_video_output_dir()))
        self.declare_parameter('record_prefix', 'dual_cam')
        self.declare_parameter('record_fps', 0.0)
        self.declare_parameter('ffmpeg_path', 'ffmpeg')
        self.declare_parameter('log_ffmpeg_stderr', False)
        self.declare_parameter('record_service', '/usb_cam_stream/record')

        self.device_index = int(self.get_parameter('device_index').value)
        self.device_path = str(self.get_parameter('device_path').value).strip()
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value).strip()
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.use_v4l2 = bool(self.get_parameter('use_v4l2').value)
        self.reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )
        self.encoding = str(self.get_parameter('encoding').value)
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value)
        self.record_prefix = str(self.get_parameter('record_prefix').value).strip()
        self.record_fps = float(self.get_parameter('record_fps').value)
        self.ffmpeg_path = str(self.get_parameter('ffmpeg_path').value).strip() or 'ffmpeg'
        self.log_ffmpeg_stderr = bool(
            self.get_parameter('log_ffmpeg_stderr').value
        )
        self.record_service = str(
            self.get_parameter('record_service').value
        ).strip() or '/usb_cam_stream/record'

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()
        self._capture: Optional[cv2.VideoCapture] = None
        self._record_enabled = False
        self._record_session_name: Optional[str] = None
        self._record_output_path: Optional[Path] = None
        self._restart_capture = False
        self.create_service(SetBool, self.record_service, self._on_record_toggle)
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._stop_recording()
        self._release_capture()
        return super().destroy_node()

    def _device_descriptor(self) -> Union[int, str]:
        return self.device_path if self.device_path else self.device_index

    def _gst_device_path(self) -> str:
        if self.device_path:
            return self.device_path
        return f'/dev/video{self.device_index}'

    def _gst_caps(self) -> str:
        caps = []
        if self.width > 0:
            caps.append(f'width={self.width}')
        if self.height > 0:
            caps.append(f'height={self.height}')
        if self.fps > 0.0:
            fps_int = max(1, int(round(self.fps)))
            caps.append(f'framerate={fps_int}/1')
        caps_str = ','.join(caps)
        fmt = self.fourcc.strip().upper()
        if fmt in ('MJPG', 'MJPEG'):
            base = 'image/jpeg'
        else:
            base = 'video/x-raw'
        return f'{base},{caps_str}' if caps_str else base

    def _build_gst_pipeline(self, record_path: Optional[Path]) -> str:
        device = self._gst_device_path()
        caps = self._gst_caps()
        fmt = self.fourcc.strip().upper()
        if fmt in ('MJPG', 'MJPEG'):
            decode = 'jpegdec ! videoconvert'
        else:
            decode = 'videoconvert'

        if record_path:
            record_location = str(record_path)
            return (
                f'v4l2src device={device} ! {caps} ! {decode} ! tee name=t '
                't. ! queue ! videoconvert ! video/x-raw,format=BGR ! '
                'appsink drop=true max-buffers=1 sync=false '
                't. ! queue ! videoconvert ! video/x-raw,format=I420 ! '
                f'avenc_mpeg4 ! mp4mux fragment-duration=1000 streamable=true '
                f'! filesink location="{record_location}"'
            )

        return (
            f'v4l2src device={device} ! {caps} ! {decode} ! '
            'video/x-raw,format=BGR ! appsink drop=true max-buffers=1 sync=false'
        )

    def _open_capture(self) -> Optional[cv2.VideoCapture]:
        pipeline = self._build_gst_pipeline(
            self._record_output_path if self._record_enabled else None
        )
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Unable to open USB camera pipeline: {exc}')
            return None

        if not cap.isOpened():
            self.get_logger().warn('Unable to open USB camera pipeline.')
            return None

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'USB camera opened (GStreamer), {actual_w}x{actual_h} @ {actual_fps:.2f} fps'
        )

        return cap

    def _release_capture(self) -> None:
        if self._capture is None:
            return
        try:
            self._capture.release()
        except Exception:
            pass
        self._capture = None

    def _format_session_name(self) -> str:
        prefix = self.record_prefix or 'recording'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        return f'{prefix}_{timestamp}'

    def _safe_topic(self) -> str:
        return self.image_topic.strip('/').replace('/', '_') or 'image'

    def _on_record_toggle(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            if not self._record_enabled:
                self._start_recording()
            response.success = True
            response.message = 'Recording started.'
        else:
            self._stop_recording()
            response.success = True
            response.message = 'Recording stopped.'
        return response

    def _start_recording(self) -> None:
        self._record_enabled = True
        if self._record_session_name is None:
            self._record_session_name = self._format_session_name()
        self.video_output_dir.mkdir(parents=True, exist_ok=True)
        self._record_output_path = (
            self.video_output_dir / f'{self._record_session_name}_{self._safe_topic()}.mp4'
        )
        self.get_logger().info(f'Recording USB MP4 to {self._record_output_path}')
        self._restart_capture = True

    def _stop_recording(self) -> None:
        self._record_enabled = False
        self._record_session_name = None
        self._record_output_path = None
        self._restart_capture = True

    def _run(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            if self._capture is None or self._restart_capture:
                self._restart_capture = False
                self._release_capture()
                self._capture = self._open_capture()
                if self._capture is None:
                    time.sleep(self.reconnect_delay_sec)
                    continue

            ok, frame = self._capture.read()
            if not ok or frame is None:
                self.get_logger().warn('Failed to read frame; reconnecting...')
                self._release_capture()
                time.sleep(self.reconnect_delay_sec)
                continue

            msg = self.bridge.cv2_to_imgmsg(frame, encoding=self.encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            self.publisher.publish(msg)

        self._release_capture()


def main() -> None:
    rclpy.init()
    node = UsbCamStreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
