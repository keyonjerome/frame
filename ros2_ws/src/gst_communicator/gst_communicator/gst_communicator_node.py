#!/usr/bin/env python3
import json
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .daemon_client import (
    GstRecordingDaemonClient,
    busy_reason_for_state,
    command_for_toggle,
)


def _default_output_dir() -> Path:
    for parent in Path(__file__).resolve().parents:
        if parent.name == 'frame':
            return parent / 'videos'
    return Path.home() / 'videos'


def _error_message(reason: str, detail: str, **extra: Any) -> str:
    payload: Dict[str, Any] = {
        'type': 'error',
        'ok': False,
        'reason': reason,
        'detail': detail,
    }
    payload.update(extra)
    return json.dumps(payload, separators=(',', ':'))


class GstCommunicatorNode(Node):
    def __init__(self) -> None:
        super().__init__('gst_communicator')
        self.declare_parameter('record_button', 3)
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('socket_path', '/tmp/filmer_recorder.sock')
        self.declare_parameter('output_dir', str(_default_output_dir()))
        self.declare_parameter('connect_timeout_sec', 1.0)
        self.declare_parameter('command_timeout_sec', 15.0)
        self.declare_parameter('record_state_topic', '/gst_communicator/recording')

        self.record_button = int(self.get_parameter('record_button').value)
        self.joy_topic = str(self.get_parameter('joy_topic').value).strip() or 'joy'
        self.socket_path = str(self.get_parameter('socket_path').value).strip()
        self.output_dir = str(self.get_parameter('output_dir').value).strip()
        self.connect_timeout_sec = float(
            self.get_parameter('connect_timeout_sec').value
        )
        self.command_timeout_sec = float(
            self.get_parameter('command_timeout_sec').value
        )
        self.record_state_topic = str(
            self.get_parameter('record_state_topic').value
        ).strip() or '/gst_communicator/recording'

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._state_pub = self.create_publisher(Bool, self.record_state_topic, state_qos)
        self._last_button_state = False
        self._button_range_warning_emitted = False
        self._last_recording_state: Optional[bool] = None

        self.create_subscription(Joy, self.joy_topic, self._on_joy, 10)
        self.create_service(Trigger, '~/status', self._on_status)
        self.create_service(Trigger, '~/toggle_recording', self._on_toggle_recording)
        self._publish_recording_state(False)

        self.get_logger().info(
            f'gst_communicator ready: button={self.record_button}, '
            f'socket={self.socket_path}, output={self.output_dir}'
        )

    def _new_client(self) -> GstRecordingDaemonClient:
        return GstRecordingDaemonClient(
            socket_path=self.socket_path,
            connect_timeout_sec=self.connect_timeout_sec,
            command_timeout_sec=self.command_timeout_sec,
        )

    def _on_status(self, request: Trigger.Request, response: Trigger.Response):
        del request
        reply = self._new_client().status()
        self._update_state_from_payload(reply.payload)
        response.success = reply.ok
        response.message = reply.message_json()
        return response

    def _on_toggle_recording(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ):
        del request
        success, message = self._toggle_recording()
        response.success = success
        response.message = message
        return response

    def _on_joy(self, msg: Joy) -> None:
        if self.record_button < 0 or self.record_button >= len(msg.buttons):
            if not self._button_range_warning_emitted:
                self.get_logger().error(
                    f'record_button index {self.record_button} out of range for Joy '
                    f'message ({len(msg.buttons)} buttons)'
                )
                self._button_range_warning_emitted = True
            return

        pressed = msg.buttons[self.record_button] == 1
        if pressed and not self._last_button_state:
            success, message = self._toggle_recording()
            if success:
                self.get_logger().info(f'Record toggle accepted: {message}')
            else:
                self.get_logger().error(f'Record toggle failed: {message}')
        self._last_button_state = pressed

    def _toggle_recording(self):
        client = self._new_client()
        status = client.status()
        if not status.ok:
            self._update_state_from_payload(status.payload)
            return False, status.message_json()

        state = str(status.payload.get('state', '')).upper()
        command = command_for_toggle(state, self.output_dir)
        if command is None:
            reason = busy_reason_for_state(state)
            return (
                False,
                _error_message(
                    reason,
                    f'Cannot toggle recording while daemon state is {state or "UNKNOWN"}',
                    state=state,
                ),
            )

        if command == 'STOP':
            reply = client.stop()
        else:
            reply = client.command(command)

        if reply.ok:
            self._update_state_from_payload(reply.payload)
        else:
            self._update_state_from_payload(status.payload)
        return reply.ok, reply.message_json()

    def _update_state_from_payload(self, payload: Dict[str, Any]) -> None:
        state = str(payload.get('state', '')).upper()
        if state in {'STARTING', 'RECORDING'}:
            self._publish_recording_state(True)
        elif state in {'IDLE', 'ERROR'}:
            self._publish_recording_state(False)

    def _publish_recording_state(self, enabled: bool) -> None:
        if self._last_recording_state == enabled:
            return
        self._last_recording_state = enabled
        msg = Bool()
        msg.data = enabled
        self._state_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = GstCommunicatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
