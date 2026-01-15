#!/usr/bin/env python3
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from std_srvs.srv import SetBool

from .recording_utils import TopicRecorder


def _default_video_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'videos'
    return Path.home() / 'videos'


class JoyRecordToggleNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_record_toggle')
        default_output = '/workspaces/isaac-ros-dev/rosbags'
        default_video_output = str(_default_video_output_dir())
        self.declare_parameter('record_button', 3)
        self.declare_parameter(
            'record_topics',
            '/image_rect,/camera_info_rect,/camera/depth/image_rect_raw,/nikon/image_raw',
        )
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('video_output_dir', default_video_output)
        self.declare_parameter('bag_prefix', 'dual_cam')
        self.declare_parameter('storage_id', '')
        self.declare_parameter('convert_to_mp4', True)
        self.declare_parameter('mp4_fps', 30.0)
        self.declare_parameter('topic_fps', '')
        self.declare_parameter('usb_cam_topic', '/nikon/image_raw')
        self.declare_parameter('usb_record_service', '/usb_cam_stream/record')
        self.declare_parameter('ffmpeg_path', 'ffmpeg')
        self.declare_parameter('log_ffmpeg_stderr', False)

        self.record_button = int(self.get_parameter('record_button').value)
        self.record_topics = self._parse_topics(self.get_parameter('record_topics').value)
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value)
        self.bag_prefix = str(self.get_parameter('bag_prefix').value)
        self.storage_id = str(self.get_parameter('storage_id').value).strip()
        self.convert_to_mp4 = bool(self.get_parameter('convert_to_mp4').value)
        self.mp4_fps = float(self.get_parameter('mp4_fps').value)
        self.topic_fps = self._parse_topic_fps(
            self.get_parameter('topic_fps').value
        )
        self.usb_cam_topic = str(self.get_parameter('usb_cam_topic').value).strip()
        self.usb_record_service = str(
            self.get_parameter('usb_record_service').value
        ).strip() or '/usb_cam_stream/record'
        self.ffmpeg_path = str(self.get_parameter('ffmpeg_path').value).strip() or 'ffmpeg'
        self.log_ffmpeg_stderr = bool(
            self.get_parameter('log_ffmpeg_stderr').value
        )

        self._last_button_state = False
        self._recording = False
        self._session_name: Optional[str] = None
        self._recorders: Dict[str, TopicRecorder] = {}
        self._record_subscriptions: Dict[str, object] = {}
        self._bridge = CvBridge()
        self._usb_record_client = self.create_client(SetBool, self.usb_record_service)

        self._topic_warning_emitted = False

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        topics_str = ','.join(self.record_topics)
        self.get_logger().info(
            f'Joy record toggle ready: button {self.record_button}, '
            f'topics={topics_str}, video_output_dir={self.video_output_dir}'
        )
        if self.output_dir != self.video_output_dir:
            self.get_logger().info(
                f'Rosbag output_dir is ignored; recording MP4s to {self.video_output_dir}'
            )

    def destroy_node(self) -> bool:
        self._stop_recording()
        return super().destroy_node()

    def _parse_topics(self, topics_param) -> List[str]:
        if isinstance(topics_param, list):
            return [str(topic) for topic in topics_param if str(topic).strip()]
        if isinstance(topics_param, str):
            return [topic.strip() for topic in topics_param.split(',') if topic.strip()]
        return []

    def _parse_topic_fps(self, fps_param) -> Dict[str, float]:
        mapping: Dict[str, float] = {}
        if isinstance(fps_param, list):
            entries = [str(item).strip() for item in fps_param if str(item).strip()]
        elif isinstance(fps_param, str):
            entries = [item.strip() for item in fps_param.split(',') if item.strip()]
        else:
            return mapping

        for entry in entries:
            if '=' in entry:
                topic, fps_str = entry.split('=', 1)
            elif ':' in entry:
                topic, fps_str = entry.split(':', 1)
            else:
                continue
            topic = self._normalize_topic(topic)
            try:
                fps = float(fps_str.strip())
            except ValueError:
                continue
            if topic and fps > 0.0:
                mapping[topic] = fps
        return mapping

    def _format_session_name(self) -> str:
        prefix = self.bag_prefix or 'recording'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        return f'{prefix}_{timestamp}'

    def _normalize_topic(self, topic: str) -> str:
        topic = str(topic).strip()
        if not topic:
            return ''
        if not topic.startswith('/'):
            return f'/{topic}'
        return topic

    def _resolve_image_topics(self) -> List[str]:
        available = {name: types for name, types in self.get_topic_names_and_types()}
        image_topics: List[str] = []
        usb_topic = self._normalize_topic(self.usb_cam_topic) if self.usb_cam_topic else ''
        for topic in self.record_topics:
            normalized = self._normalize_topic(topic)
            if not normalized:
                continue
            if usb_topic and normalized == usb_topic:
                continue
            types = available.get(normalized)
            if types and 'sensor_msgs/msg/Image' not in types:
                self.get_logger().warn(
                    f'Skipping non-image topic {normalized} (types={",".join(types)})'
                )
                continue
            image_topics.append(normalized)
        return image_topics

    def _on_joy(self, msg: Joy) -> None:
        if self.record_button < 0 or self.record_button >= len(msg.buttons):
            if not self._topic_warning_emitted:
                self.get_logger().error(
                    f'record_button index {self.record_button} out of range for Joy message '
                    f'({len(msg.buttons)} buttons)'
                )
                self._topic_warning_emitted = True
            return

        pressed = msg.buttons[self.record_button] == 1
        if pressed and not self._last_button_state:
            if not self._recording:
                self._start_recording()
            else:
                self._stop_recording()
        self._last_button_state = pressed

    def _start_recording(self) -> None:
        if self._recording:
            return
        if not self.record_topics:
            self.get_logger().error('No record_topics provided; skipping recording.')
            return

        self._recording = True
        self._session_name = self._format_session_name()
        self.video_output_dir.mkdir(parents=True, exist_ok=True)
        image_topics = self._resolve_image_topics()
        if image_topics:
            for topic in image_topics:
                self._record_subscriptions[topic] = self.create_subscription(
                    Image,
                    topic,
                    lambda msg, topic=topic: self._on_image(msg, topic),
                    10,
                )
        else:
            self.get_logger().warn('No D421 image topics to record; USB recording only.')

        self._request_usb_recording(True)
        self.get_logger().info(
            f'Recording started: session={self._session_name}, output_dir={self.video_output_dir}'
        )

    def _stop_recording(self) -> None:
        if not self._recording:
            return
        self.get_logger().info('Stopping recording...')
        self._recording = False
        self._request_usb_recording(False)

        for recorder in self._recorders.values():
            recorder.close()
        self._recorders.clear()

        for sub in self._record_subscriptions.values():
            self.destroy_subscription(sub)
        self._record_subscriptions.clear()
        self._session_name = None
        self.get_logger().info('Recording stopped.')

    def _request_usb_recording(self, enable: bool) -> None:
        if not self._usb_record_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(
                f'USB record service not available at {self.usb_record_service}'
            )
            return
        request = SetBool.Request()
        request.data = enable
        self._usb_record_client.call_async(request)

    def _on_image(self, msg: Image, topic: str) -> None:
        if not self._recording or not self._session_name:
            return
        frame, is_color = self._decode_frame(msg, topic)
        if frame is None:
            return
        if not is_color:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        recorder = self._recorders.get(topic)
        if recorder is None:
            fps = self.topic_fps.get(topic, self.mp4_fps)
            if fps <= 0.0:
                fps = self.mp4_fps
            recorder = TopicRecorder(
                topic=topic,
                output_dir=self.video_output_dir,
                session_name=self._session_name,
                fps=fps,
                ffmpeg_path=self.ffmpeg_path,
                log_ffmpeg_stderr=self.log_ffmpeg_stderr,
                logger=self.get_logger(),
            )
            self._recorders[topic] = recorder
        recorder.write(frame)

    def _decode_frame(
        self, msg: Image, topic: str
    ) -> Tuple[Optional[np.ndarray], bool]:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            return frame, True
        except Exception:
            pass

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return frame, False
        except Exception:
            pass

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Frame decode failed on {topic}: {exc}')
            return None, False

        if frame.dtype != np.uint8:
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
            frame = frame.astype(np.uint8)

        if frame.ndim == 3 and frame.shape[2] == 3:
            return frame, True

        return frame, False


def main() -> None:
    rclpy.init()
    node = JoyRecordToggleNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
