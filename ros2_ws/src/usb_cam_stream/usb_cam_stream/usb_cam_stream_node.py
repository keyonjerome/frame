#!/usr/bin/env python3
import threading
import time
from pathlib import Path
from typing import Optional, Union

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .gstreamer_writer import GstreamerWriter


def _default_video_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'videos'
    return Path.home() / 'videos'


class UsbCamStreamNode(Node):
    def __init__(self) -> None:
        super().__init__('usb_cam_stream')
        self.declare_parameter('device_index', 4)
        self.declare_parameter('device_path', '')
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('fps', 0.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('video_output_dir', str(_default_video_output_dir()))
        self.declare_parameter('record_prefix', 'dual_cam')
        self.declare_parameter('record_name', 'nikon_image_raw')
        self.declare_parameter('gstreamer_encoder', '')
        self.declare_parameter('enable_audio', True)
        self.declare_parameter('audio_device', '')
        self.declare_parameter('audio_encoder', '')
        self.declare_parameter('record_command_topic', '/usb_cam_stream/record')
        self.declare_parameter('record_state_topic', '/usb_cam_stream/recording')
        self.declare_parameter('retry_interval_sec', 1.0)

        self.device_index = int(self.get_parameter('device_index').value)
        self.device_path = str(self.get_parameter('device_path').value).strip()
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.fourcc = str(self.get_parameter('fourcc').value).strip()
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value)
        self.record_prefix = str(self.get_parameter('record_prefix').value).strip()
        self.record_name = str(self.get_parameter('record_name').value).strip()
        self.gstreamer_encoder = str(
            self.get_parameter('gstreamer_encoder').value
        ).strip()
        self.enable_audio = bool(self.get_parameter('enable_audio').value)
        self.audio_device = str(self.get_parameter('audio_device').value).strip()
        self.audio_encoder = str(self.get_parameter('audio_encoder').value).strip()
        self.record_command_topic = str(
            self.get_parameter('record_command_topic').value
        ).strip() or '/usb_cam_stream/record'
        self.record_state_topic = str(
            self.get_parameter('record_state_topic').value
        ).strip() or '/usb_cam_stream/recording'
        self.retry_interval_sec = float(
            self.get_parameter('retry_interval_sec').value
        )

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(Bool, self.record_state_topic, state_qos)
        self.create_subscription(Bool, self.record_command_topic, self._on_command, 10)

        self._capture: Optional[cv2.VideoCapture] = None
        self._writer: Optional[GstreamerWriter] = None
        self._desired = False
        self._active = False
        self._session_name: Optional[str] = None
        self._output_path: Optional[Path] = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._set_active(False)

        self.get_logger().info(
            f'USB recorder ready: cmd={self.record_command_topic}, '
            f'state={self.record_state_topic}, output={self.video_output_dir}'
        )

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._stop_recording()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _device_descriptor(self) -> Union[int, str]:
        return self.device_path if self.device_path else self.device_index

    def _open_capture(self) -> Optional[cv2.VideoCapture]:
        device = self._device_descriptor()
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().warn(f'Unable to open USB camera {device}.')
            return None
        if self.fourcc:
            try:
                fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
                cap.set(cv2.CAP_PROP_FOURCC, fourcc)
            except Exception:
                pass
        if self.width > 0:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0.0:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
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

    def _safe_name(self) -> str:
        name = self.record_name.strip().strip('/')
        return name.replace('/', '_') or 'usb_cam'

    def _record_fps(self) -> float:
        if self.fps > 0.0:
            return self.fps
        if self._capture:
            cap_fps = self._capture.get(cv2.CAP_PROP_FPS) or 0.0
            if cap_fps > 0.0:
                return cap_fps
        return 30.0

    def _set_active(self, enabled: bool) -> None:
        if self._active == enabled:
            return
        self._active = enabled
        msg = Bool()
        msg.data = enabled
        self._state_pub.publish(msg)

    def _start_recording(self) -> None:
        if self._session_name is None:
            self._session_name = self._format_session_name()
        self.video_output_dir.mkdir(parents=True, exist_ok=True)
        self._output_path = (
            self.video_output_dir / f'{self._session_name}_{self._safe_name()}.mp4'
        )

    def _stop_recording(self) -> None:
        self._desired = False
        self._session_name = None
        self._output_path = None
        if self._writer:
            try:
                self._writer.close()
            except Exception:
                pass
        self._writer = None
        self._release_capture()
        self._set_active(False)

    def _on_command(self, msg: Bool) -> None:
        self._desired = bool(msg.data)
        if self._desired:
            self._start_recording()
        else:
            self._stop_recording()

    def _run(self) -> None:
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                if not self._desired:
                    if self._writer or self._capture:
                        self._stop_recording()
                    time.sleep(0.1)
                    continue
                if self._capture is None:
                    self._capture = self._open_capture()
                    if self._capture is None:
                        time.sleep(max(0.1, self.retry_interval_sec))
                        continue
                ok, frame = self._capture.read()
                if not ok or frame is None:
                    self._release_capture()
                    time.sleep(max(0.1, self.retry_interval_sec))
                    continue
                if self._writer is None:
                    if self._output_path is None:
                        self._start_recording()
                    if self._output_path is None:
                        time.sleep(max(0.1, self.retry_interval_sec))
                        continue
                    fps = self._record_fps()
                    self._writer = GstreamerWriter(
                        output_path=self._output_path,
                        width=frame.shape[1],
                        height=frame.shape[0],
                        fps=fps,
                        logger=self.get_logger(),
                        encoder=self.gstreamer_encoder,
                        enable_audio=self.enable_audio,
                        audio_device=self.audio_device,
                        audio_encoder=self.audio_encoder,
                    )
                    if not self._writer.is_open():
                        self._writer = None
                        time.sleep(max(0.1, self.retry_interval_sec))
                        continue
                    self.get_logger().info(
                        f'Recording USB MP4 to {self._output_path}'
                    )
                    self._set_active(True)
                if not self._writer.write(frame):
                    self._writer.close()
                    self._writer = None
                    self._set_active(False)
                    time.sleep(max(0.1, self.retry_interval_sec))
        finally:
            self._stop_recording()


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
