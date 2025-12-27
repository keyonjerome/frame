#!/usr/bin/env python3
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from typing import Optional


class MtplvCapStreamNode(Node):
    def __init__(self) -> None:
        super().__init__('mtplvcap_stream')
        self.declare_parameter('stream_url', 'http://127.0.0.1:5600')
        self.declare_parameter('image_topic', 'nikon/image_raw')
        self.declare_parameter('frame_id', 'nikon_camera')
        self.declare_parameter('reconnect_delay_sec', 1.0)

        self.stream_url = self.get_parameter('stream_url').value
        self.image_topic = self.get_parameter('image_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        return super().destroy_node()

    def _open_capture(self) -> Optional[cv2.VideoCapture]:
        cap = cv2.VideoCapture(self.stream_url)
        if not cap.isOpened():
            return None
        return cap

    def _run(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            cap = self._open_capture()
            if cap is None:
                self.get_logger().warn(
                    f'Unable to open mtplvcap stream at {self.stream_url}. '
                    f'Retrying in {self.reconnect_delay_sec:.1f}s.'
                )
                time.sleep(self.reconnect_delay_sec)
                continue

            self.get_logger().info(f'Connected to mtplvcap stream at {self.stream_url}')
            while rclpy.ok() and not self._stop_event.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    self.get_logger().warn('Stream read failed; reconnecting...')
                    break

                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                self.publisher.publish(msg)

            cap.release()
            time.sleep(self.reconnect_delay_sec)


def main() -> None:
    rclpy.init()
    node = MtplvCapStreamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
