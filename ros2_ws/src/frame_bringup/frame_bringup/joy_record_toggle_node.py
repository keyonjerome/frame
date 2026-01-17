#!/usr/bin/env python3
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Bool


def _default_video_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'videos'
    return Path.home() / 'videos'


def _default_rosbag_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'rosbags'
    return Path.home() / 'rosbags'


class JoyRecordToggleNode(Node):
    def __init__(self) -> None:
        super().__init__('joy_record_toggle')
        self.declare_parameter('record_button', 3)
        self.declare_parameter(
            'record_topics',
            '/image_rect,/camera_info_rect,/camera/depth/image_rect_raw',
        )
        self.declare_parameter('output_dir', str(_default_rosbag_output_dir()))
        self.declare_parameter('video_output_dir', str(_default_video_output_dir()))
        self.declare_parameter('bag_prefix', 'dual_cam')
        self.declare_parameter('storage_id', '')
        self.declare_parameter('image_topic', '/image_rect')
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('d421_fps', 30.0)
        self.declare_parameter('mp4_fps', 30.0)
        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 6.0)
        self.declare_parameter('usb_cam_topic', '/nikon/image_raw')
        self.declare_parameter('usb_record_topic', '/usb_cam_stream/record')

        self.record_button = int(self.get_parameter('record_button').value)
        self.record_topics = self._parse_topics(self.get_parameter('record_topics').value)
        self.output_dir = Path(self.get_parameter('output_dir').value)
        self.video_output_dir = Path(self.get_parameter('video_output_dir').value)
        self.bag_prefix = str(self.get_parameter('bag_prefix').value)
        self.storage_id = str(self.get_parameter('storage_id').value).strip()
        self.image_topic = self._normalize_topic(
            self.get_parameter('image_topic').value
        )
        self.depth_topic = self._normalize_topic(
            self.get_parameter('depth_topic').value
        )
        self.d421_fps = float(self.get_parameter('d421_fps').value)
        self.mp4_fps = float(self.get_parameter('mp4_fps').value)
        self.depth_min_m = float(self.get_parameter('depth_min_m').value)
        self.depth_max_m = float(self.get_parameter('depth_max_m').value)
        self.usb_cam_topic = self._normalize_topic(
            self.get_parameter('usb_cam_topic').value
        )
        self.usb_record_topic = str(
            self.get_parameter('usb_record_topic').value
        ).strip() or '/usb_cam_stream/record'

        default_rosbag_dir = _default_rosbag_output_dir()
        if self.output_dir != default_rosbag_dir:
            self.get_logger().warn(
                f'Overriding output_dir to {default_rosbag_dir} per requirements.'
            )
            self.output_dir = default_rosbag_dir

        self._record_process: Optional[subprocess.Popen] = None
        self._current_bag_path: Optional[Path] = None
        self._last_button_state = False
        self._topic_warning_emitted = False
        self._bridge = CvBridge()
        self._usb_pub = self.create_publisher(Bool, self.usb_record_topic, 10)

        self.create_subscription(Joy, 'joy', self._on_joy, 10)
        self.get_logger().info(
            f'Joy record toggle ready: button {self.record_button}, '
            f'rosbags={self.output_dir}, videos={self.video_output_dir}'
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

    def _normalize_topic(self, topic: str) -> str:
        topic = str(topic).strip()
        if not topic:
            return ''
        if not topic.startswith('/'):
            return f'/{topic}'
        return topic

    def _format_session_name(self) -> str:
        prefix = self.bag_prefix or 'recording'
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        return f'{prefix}_{timestamp}'

    def _bag_topics(self) -> List[str]:
        topics = []
        for topic in self.record_topics:
            normalized = self._normalize_topic(topic)
            if not normalized or normalized == self.usb_cam_topic:
                continue
            topics.append(normalized)
        return topics

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
            if self._record_process is None:
                self._start_recording()
            else:
                self._stop_recording()
        self._last_button_state = pressed

    def _start_recording(self) -> None:
        topics = self._bag_topics()
        if not topics:
            self.get_logger().error('No D421 topics configured; skipping recording.')
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        bag_name = self._format_session_name()
        bag_path = self.output_dir / bag_name
        cmd = ['ros2', 'bag', 'record', '-o', str(bag_path)] + topics
        if self.storage_id:
            cmd += ['--storage', self.storage_id]

        try:
            self._record_process = subprocess.Popen(cmd)
        except FileNotFoundError:
            self._record_process = None
            self.get_logger().error('Failed to start ros2 bag; check ROS 2 CLI.')
            return

        self._current_bag_path = bag_path
        self._publish_usb(True)
        self.get_logger().info(f'Recording started: {bag_path}')

    def _stop_recording(self) -> None:
        if self._record_process is None:
            return
        self.get_logger().info('Stopping recording...')
        if self._record_process.poll() is None:
            self._record_process.send_signal(signal.SIGINT)
            try:
                self._record_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._record_process.terminate()
        self._record_process = None
        self._publish_usb(False)
        bag_path = self._current_bag_path
        self._current_bag_path = None
        self.get_logger().info('Recording stopped.')

        if bag_path:
            threading.Thread(
                target=self._convert_bag_to_mp4,
                args=(bag_path,),
                daemon=True,
            ).start()

    def _publish_usb(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = enabled
        self._usb_pub.publish(msg)

    def _open_reader(self, bag_path: Path, storage_id: str) -> rosbag2_py.SequentialReader:
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(bag_path), storage_id=storage_id
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        reader.open(storage_options, converter_options)
        return reader

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
        storage_id = self._detect_storage_id(bag_path)
        topics = [t for t in (self.image_topic, self.depth_topic) if t]
        if not topics:
            self.get_logger().warn('No image topics configured for conversion.')
            return

        try:
            reader = self._open_reader(bag_path, storage_id)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Failed to open bag {bag_path}: {exc}')
            return

        self.video_output_dir.mkdir(parents=True, exist_ok=True)
        writers: Dict[str, cv2.VideoWriter] = {}
        frame_counts: Dict[str, int] = {}
        input_counts: Dict[str, int] = {}
        last_stamp_ns: Dict[str, int] = {}
        delta_sum_ns: Dict[str, int] = {}
        delta_count: Dict[str, int] = {}
        next_output_ns: Dict[str, int] = {}
        last_frame: Dict[str, np.ndarray] = {}
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        target_fps = self.mp4_fps if self.mp4_fps > 0.0 else 30.0
        output_interval_ns = max(1, int(1e9 / target_fps))

        self.get_logger().info(
            f'Converting bag {bag_path} to MP4 @ {target_fps:.2f} fps '
            f'(topics={",".join(topics)})'
        )

        while reader.has_next():
            topic, data, timestamp_ns = reader.read_next()
            if topic not in topics:
                continue
            msg: Image = deserialize_message(data, Image)
            if timestamp_ns <= 0 and hasattr(msg, 'header'):
                stamp = msg.header.stamp
                timestamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            if topic == self.depth_topic:
                frame = self._colorize_depth(msg)
            else:
                frame = self._decode_image(msg)
            if frame is None:
                continue
            input_counts[topic] = input_counts.get(topic, 0) + 1

            last_stamp = last_stamp_ns.get(topic)
            if last_stamp is not None:
                delta_ns = timestamp_ns - last_stamp
                if delta_ns > 0:
                    delta_sum_ns[topic] = delta_sum_ns.get(topic, 0) + delta_ns
                    delta_count[topic] = delta_count.get(topic, 0) + 1
            last_stamp_ns[topic] = timestamp_ns

            if topic not in writers:
                safe_topic = topic.strip('/').replace('/', '_') or 'image'
                video_path = self.video_output_dir / f'{bag_path.name}_{safe_topic}.mp4'
                writer = cv2.VideoWriter(
                    str(video_path),
                    fourcc,
                    target_fps,
                    (frame.shape[1], frame.shape[0]),
                    isColor=True,
                )
                if not writer.isOpened():
                    self.get_logger().error(f'Could not open MP4 writer for {video_path}')
                    continue
                writers[topic] = writer
                frame_counts[topic] = 0
                self.get_logger().info(f'Writing {video_path} @ {target_fps:.2f} fps')

            writer = writers.get(topic)
            if not writer:
                continue

            if topic not in next_output_ns:
                next_output_ns[topic] = timestamp_ns
                last_frame[topic] = frame
                writer.write(frame)
                frame_counts[topic] = frame_counts.get(topic, 0) + 1
                next_output_ns[topic] += output_interval_ns
                continue

            previous_frame = last_frame.get(topic)
            while next_output_ns[topic] < timestamp_ns:
                if previous_frame is not None:
                    writer.write(previous_frame)
                    frame_counts[topic] = frame_counts.get(topic, 0) + 1
                next_output_ns[topic] += output_interval_ns

            if next_output_ns[topic] == timestamp_ns:
                writer.write(frame)
                frame_counts[topic] = frame_counts.get(topic, 0) + 1
                next_output_ns[topic] += output_interval_ns
                last_frame[topic] = frame
                continue

            last_frame[topic] = frame

        for writer in writers.values():
            writer.release()

        for topic, count in frame_counts.items():
            avg_fps = None
            if delta_count.get(topic, 0) > 0:
                avg_fps = 1e9 * delta_count[topic] / delta_sum_ns[topic]
            if avg_fps is None:
                self.get_logger().info(f'Finished {topic} ({count} frames)')
            else:
                input_count = input_counts.get(topic, 0)
                self.get_logger().info(
                    f'Finished {topic} (output {count} frames @ {target_fps:.2f} fps, '
                    f'input {input_count} frames ~{avg_fps:.2f} fps)'
                )

    def _decode_image(self, msg: Image) -> Optional[np.ndarray]:
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass
        try:
            mono = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        except Exception:
            pass
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return None
        if frame is None:
            return None
        if frame.ndim == 2:
            return cv2.cvtColor(frame.astype(np.uint8), cv2.COLOR_GRAY2BGR)
        return frame

    def _colorize_depth(self, msg: Image) -> Optional[np.ndarray]:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception:
            return None
        if depth is None:
            return None
        depth = depth.astype(np.float32)
        if msg.encoding == '16UC1':
            depth *= 0.001
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth = np.clip(depth, self.depth_min_m, self.depth_max_m)
        denom = max(1e-6, self.depth_max_m - self.depth_min_m)
        norm = ((depth - self.depth_min_m) / denom * 255.0).astype(np.uint8)
        return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


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
