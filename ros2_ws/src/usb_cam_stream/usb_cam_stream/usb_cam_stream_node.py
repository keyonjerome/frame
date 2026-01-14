#!/usr/bin/env python3
import threading
import time
from typing import Optional, Union

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


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

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()
        self._capture: Optional[cv2.VideoCapture] = None
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._release_capture()
        return super().destroy_node()

    def _device_descriptor(self) -> Union[int, str]:
        return self.device_path if self.device_path else self.device_index

    def _open_capture(self) -> Optional[cv2.VideoCapture]:
        device = self._device_descriptor()
        backend = cv2.CAP_V4L2 if self.use_v4l2 else cv2.CAP_ANY
        try:
            cap = cv2.VideoCapture(device, backend)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Unable to open USB camera {device}: {exc}')
            return None

        if not cap.isOpened():
            self.get_logger().warn(f'Unable to open USB camera {device}.')
            return None

        if self.buffer_size > 0:
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)
            except Exception:
                pass

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

        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'USB camera opened ({device}), {actual_w}x{actual_h} @ {actual_fps:.2f} fps'
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

    def _run(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            if self._capture is None:
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
