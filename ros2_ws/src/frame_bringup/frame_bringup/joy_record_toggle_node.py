#!/usr/bin/env python3
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Joy


class JoyRecordToggleNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_record_toggle')
        default_output = str(Path.home() / 'rosbags')
        self.declare_parameter('record_button', 3)
        self.declare_parameter(
            'record_topics',
            '/image_rect,/camera_info_rect,/nikon/image_raw',
        )
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('video_output_dir', default_output)
        self.declare_parameter('bag_prefix', 'dual_cam')
        self.declare_parameter('storage_id', '')
        self.declare_parameter('convert_to_mp4', True)
        self.declare_parameter('mp4_fps', 30.0)

        self.record_button = int(self.get_parameter('record_button').value)
        self.record_topics = self._parse_topics(self.get_parameter('record_topics').value)
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value)
        self.bag_prefix = str(self.get_parameter('bag_prefix').value)
        self.storage_id = str(self.get_parameter('storage_id').value).strip()
        self.convert_to_mp4 = bool(self.get_parameter('convert_to_mp4').value)
        self.mp4_fps = float(self.get_parameter('mp4_fps').value)

        self._last_button_state = False
        self._record_process: subprocess.Popen | None = None
        self._current_bag_path: Optional[Path] = None
        self._bridge = CvBridge()

        self._topic_warning_emitted = False

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        topics_str = ','.join(self.record_topics)
        self.get_logger().info(
            f'Joy record toggle ready: button {self.record_button}, '
            f'topics={topics_str}, output_dir={self.output_dir}'
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

    def _on_joy(self, msg: Joy) -> None:
        if self.record_button < 0 or self.record_button >= len(msg.buttons):
            if not self._topic_warning_emitted:
                self.get_logger().error(
                    'record_button index %d out of range for Joy message (%d buttons)',
                    self.record_button,
                    len(msg.buttons),
                )
                self._topic_warning_emitted = True
            return

        pressed = msg.buttons[self.record_button] == 1
        if pressed and not self._last_button_state:
            if self._record_process is None:
                self._start_recording()
            else:
                self._stop_recording()
        self._last_button_state = pressed

    def _start_recording(self) -> None:
        if not self.record_topics:
            self.get_logger().error('No record_topics provided; skipping recording.')
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        bag_name = f'{self.bag_prefix}_{timestamp}'
        bag_path = self.output_dir / bag_name

        cmd = ['ros2', 'bag', 'record', '-o', str(bag_path)] + self.record_topics
        if self.storage_id:
            cmd += ['--storage', self.storage_id]

        try:
            self._record_process = subprocess.Popen(cmd)
        except FileNotFoundError:
            self._record_process = None
            self.get_logger().error(
                'Failed to start ros2 bag; ensure ros2 CLI is available in PATH.'
            )
            return

        self.get_logger().info('Recording started: %s', str(bag_path))
        self._current_bag_path = bag_path

    def _stop_recording(self) -> None:
        if self._record_process is None:
            return

        if self._record_process.poll() is None:
            self.get_logger().info('Stopping recording...')
            self._record_process.send_signal(signal.SIGINT)
            try:
                self._record_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().warn('ros2 bag did not exit, terminating...')
                self._record_process.terminate()
        self._record_process = None
        bag_path = self._current_bag_path
        self._current_bag_path = None
        self.get_logger().info('Recording stopped.')

        if self.convert_to_mp4 and bag_path:
            threading.Thread(
                target=self._convert_bag_to_mp4,
                args=(bag_path,),
                daemon=True,
            ).start()

    def _detect_storage_id(self, bag_path: Path) -> str:
        storage_id = self.storage_id or 'sqlite3'
        metadata_path = bag_path / 'metadata.yaml'
        if metadata_path.exists():
            try:
                for line in metadata_path.read_text().splitlines():
                    if line.strip().startswith('storage_identifier:'):
                        storage_id = line.split(':', 1)[1].strip()
                        break
            except OSError:
                pass
        return storage_id

    def _convert_bag_to_mp4(self, bag_path: Path) -> None:
        """Convert recorded bag image topics to MP4 files for the web UI."""
        storage_id = self._detect_storage_id(bag_path)
        output_dir = self.video_output_dir
        output_dir.mkdir(parents=True, exist_ok=True)

        try:
            reader = rosbag2_py.SequentialReader()
            storage_options = rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id=storage_id
            )
            converter_options = rosbag2_py.ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr',
            )
            reader.open(storage_options, converter_options)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                'Failed to open bag %s for conversion: %s', str(bag_path), exc
            )
            return

        topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
        image_topics = [
            t for t in self.record_topics if topic_types.get(t) == 'sensor_msgs/msg/Image'
        ]
        if not image_topics:
            self.get_logger().info(
                'No image topics to convert for bag %s (topics=%s)',
                str(bag_path),
                ','.join(topic_types.keys()),
            )
            return

        writers: Dict[str, cv2.VideoWriter] = {}
        frame_counts: Dict[str, int] = {}
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')

        self.get_logger().info(
            'Converting bag %s to MP4 (topics=%s)',
            str(bag_path),
            ','.join(image_topics),
        )

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic not in image_topics:
                continue

            try:
                msg: Image = deserialize_message(data, Image)
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn('Frame decode failed on %s: %s', topic, exc)
                continue

            height, width = frame.shape[:2]
            if topic not in writers:
                safe_topic = topic.strip('/').replace('/', '_') or 'image'
                video_path = output_dir / f'{bag_path.name}_{safe_topic}.mp4'
                writer = cv2.VideoWriter(
                    str(video_path),
                    fourcc,
                    self.mp4_fps,
                    (width, height),
                )
                if not writer.isOpened():
                    self.get_logger().error(
                        'Could not open MP4 writer for %s', str(video_path)
                    )
                    continue
                writers[topic] = writer
                frame_counts[topic] = 0
                self.get_logger().info('Writing %s', str(video_path))

            writer = writers.get(topic)
            if writer:
                writer.write(frame)
                frame_counts[topic] = frame_counts.get(topic, 0) + 1

        for writer in writers.values():
            writer.release()

        for topic, count in frame_counts.items():
            self.get_logger().info(
                'Finished %s (%d frames)', topic, count
            )


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
