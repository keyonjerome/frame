#!/usr/bin/env python3
import threading
import time
from typing import Optional

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class MtplvCapStreamNode(Node):
    def __init__(self) -> None:
        super().__init__('mtplvcap_stream')
        self.declare_parameter('stream_url', 'http://127.0.0.1:5600/mjpeg')
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

    def _open_stream(self) -> Optional[requests.Response]:
        try:
            resp = requests.get(
                self.stream_url,
                stream=True,
                headers={'User-Agent': 'mtplvcap-stream-node'},
                timeout=5,
            )
            resp.raise_for_status()
            return resp
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f'Unable to open mtplvcap stream at {self.stream_url}: {exc}'
            )
            return None

    def _run(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            resp = self._open_stream()
            if resp is None:
                time.sleep(self.reconnect_delay_sec)
                continue

            buf = bytearray()

            for chunk in resp.iter_content(chunk_size=4096):
                if self._stop_event.is_set():
                    break
                if not chunk:
                    continue
                buf.extend(chunk)

                soi = buf.find(b'\xff\xd8')
                eoi = buf.find(b'\xff\xd9')
                if soi != -1 and eoi != -1 and eoi > soi:
                    jpg = bytes(buf[soi:eoi + 2])
                    buf = buf[eoi + 2:]  # keep remainder

                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_id
                    self.publisher.publish(msg)

            try:
                resp.close()
            except Exception:
                pass
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
