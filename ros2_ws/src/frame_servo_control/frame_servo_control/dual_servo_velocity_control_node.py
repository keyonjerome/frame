#!/usr/bin/env python3
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from frame_servo_interfaces.srv import SetServoAngle
from frame_servo_control.servo_utils import (
    DualServo,
    SERVO_A_CH,
    SERVO_A_PWMCHIP,
    SERVO_B_CH,
    SERVO_B_PWMCHIP,
    SERVO_HZ,
    STARTUP_HOLD_S,
    SysfsPWM,
    clamp,
    describe_pwm,
    servo_pulses_for_angle,
)
from rclpy.node import Node
from sensor_msgs.msg import Joy


DEFAULT_MAX_VELOCITY_DEG_S = 10.0
DEFAULT_CONTROL_HZ = 50.0
DEFAULT_DEADBAND = 0.05


@dataclass(frozen=True)
class ApplyResult:
    requested_angle: float
    applied_angle: float
    clamped: bool


class DualServoVelocityControlNode(Node):
    def __init__(self) -> None:
        super().__init__('dual_servo_velocity_control')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('axis_index', 4)
        self.declare_parameter('invert_axis', False)
        self.declare_parameter('startup_hold_s', STARTUP_HOLD_S)
        self.declare_parameter('reset_button_index', 2)
        self.declare_parameter('reset_angle_deg', 45.0)
        self.declare_parameter('angle_range_deg', 25.0)
        self.declare_parameter('max_velocity_deg_s', DEFAULT_MAX_VELOCITY_DEG_S)
        self.declare_parameter('control_hz', DEFAULT_CONTROL_HZ)
        self.declare_parameter('deadband', DEFAULT_DEADBAND)

        self._axis_index = int(self.get_parameter('axis_index').value)
        self._invert_axis = bool(self.get_parameter('invert_axis').value)
        self._reset_button_index = int(self.get_parameter('reset_button_index').value)
        self._reset_angle_deg = float(self.get_parameter('reset_angle_deg').value)
        self._angle_range_deg = float(self.get_parameter('angle_range_deg').value)
        self._max_velocity_deg_s = float(self.get_parameter('max_velocity_deg_s').value)
        self._control_hz = float(self.get_parameter('control_hz').value)
        self._deadband = float(self.get_parameter('deadband').value)

        self._min_angle_deg = self._reset_angle_deg
        self._max_angle_deg = self._reset_angle_deg + self._angle_range_deg
        if self._angle_range_deg <= 0.0:
            raise ValueError('angle_range_deg must be greater than zero')
        if self._max_angle_deg <= self._min_angle_deg:
            raise ValueError('computed max angle must be greater than min angle')
        if self._max_velocity_deg_s <= 0.0:
            raise ValueError('max_velocity_deg_s must be greater than zero')
        if self._control_hz <= 0.0:
            raise ValueError('control_hz must be greater than zero')
        if self._deadband < 0.0 or self._deadband >= 1.0:
            raise ValueError('deadband must be in the range [0.0, 1.0)')

        self._ready_time = time.time() + max(
            0.0,
            float(self.get_parameter('startup_hold_s').value),
        )

        initial_pulses = self._servo_pulses_for_angle(self._reset_angle_deg)
        self._pwm_a: Optional[SysfsPWM] = None
        self._pwm_b: Optional[SysfsPWM] = None
        self._servos: Optional[DualServo] = None
        self._pwm_available = False
        self._current_angle_deg = self._reset_angle_deg
        self._axis_command = 0.0
        self._last_update_time = time.monotonic()
        self._last_pulses: Optional[Tuple[int, int]] = initial_pulses
        self._last_button_state = False
        self._axis_warning_emitted = False
        self._button_warning_emitted = False
        self._write_error_emitted = False

        try:
            pwm_a = SysfsPWM(SERVO_A_PWMCHIP, SERVO_A_CH)
            pwm_b = SysfsPWM(SERVO_B_PWMCHIP, SERVO_B_CH)
            pwm_a.setup(SERVO_HZ, initial_pulses[0])
            pwm_b.setup(SERVO_HZ, initial_pulses[1])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                'PWM setup failed; servo commands will be rejected until sysfs '
                f'PWM is writable by this process: {exc}'
            )
        else:
            self._pwm_a = pwm_a
            self._pwm_b = pwm_b
            self._servos = DualServo(pwm_a, pwm_b)
            self._pwm_available = True

            self.get_logger().info('=== PWM DEVICES IN USE ===')
            describe_pwm(pwm_a, 'Servo A (pin 32)')
            describe_pwm(pwm_b, 'Servo B (pin 33)')
            self.get_logger().info('==========================')

        joy_topic = str(self.get_parameter('joy_topic').value).strip() or 'joy'
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self._service = self.create_service(
            SetServoAngle,
            '/dual_servo_control/set_angle',
            self._handle_set_angle,
        )
        self._timer = self.create_timer(1.0 / self._control_hz, self._on_timer)
        self.get_logger().info(
            f'Listening for Joy on {joy_topic}; axis {self._axis_index}'
            f"{' (inverted)' if self._invert_axis else ''}; "
            f'reset button {self._reset_button_index}; '
            f'range [{self._min_angle_deg:.2f}, {self._max_angle_deg:.2f}] deg; '
            f'max velocity {self._max_velocity_deg_s:.2f} deg/s; '
            f'deadband {self._deadband:.2f}.'
        )

    def destroy_node(self) -> bool:
        try:
            if self._pwm_a is not None:
                self._pwm_a.relax()
            if self._pwm_b is not None:
                self._pwm_b.relax()
        except Exception:
            pass
        return super().destroy_node()

    def _servo_pulses_for_angle(self, angle_deg: float) -> Tuple[int, int]:
        return servo_pulses_for_angle(
            angle_deg,
            self._min_angle_deg,
            self._max_angle_deg,
        )

    def _apply_angle(self, requested_angle_deg: float, source: str) -> ApplyResult:
        if self._servos is None or not self._pwm_available:
            raise RuntimeError('PWM is unavailable or not writable')

        applied_angle = clamp(requested_angle_deg, self._min_angle_deg, self._max_angle_deg)
        pulses = self._servo_pulses_for_angle(applied_angle)

        if self._last_pulses != pulses:
            self._servos.set_pulses(*pulses)
            self._last_pulses = pulses

        self._current_angle_deg = applied_angle
        self._write_error_emitted = False
        self.get_logger().debug(
            f'Applied {source} angle request {requested_angle_deg:.2f} -> {applied_angle:.2f}'
        )
        return ApplyResult(
            requested_angle=requested_angle_deg,
            applied_angle=applied_angle,
            clamped=not math.isclose(requested_angle_deg, applied_angle, abs_tol=1e-6),
        )

    def _axis_to_velocity_command(self, axis_value: float) -> float:
        if self._invert_axis:
            axis_value = -axis_value
        axis_value = clamp(axis_value, -1.0, 1.0)
        if abs(axis_value) < self._deadband:
            return 0.0
        return axis_value

    def _handle_set_angle(
        self,
        request: SetServoAngle.Request,
        response: SetServoAngle.Response,
    ) -> SetServoAngle.Response:
        try:
            result = self._apply_angle(float(request.target_angle_deg), 'service')
        except Exception as exc:
            if not self._write_error_emitted:
                self.get_logger().error(f'Failed to update servos from service: {exc}')
                self._write_error_emitted = True
            response.success = False
            response.message = f'failed to apply angle: {exc}'
            return response

        response.success = True
        if result.clamped:
            response.message = (
                f'Applied {result.applied_angle:.2f} deg '
                f'(clamped from {result.requested_angle:.2f} deg).'
            )
        else:
            response.message = f'Applied {result.applied_angle:.2f} deg.'
        return response

    def _reset_to_start_angle(self) -> None:
        try:
            result = self._apply_angle(self._reset_angle_deg, 'reset')
        except Exception as exc:
            if not self._write_error_emitted:
                self.get_logger().error(f'Failed to reset servos: {exc}')
                self._write_error_emitted = True
            return

        self.get_logger().info(f'Reset completed: Applied {result.applied_angle:.2f} deg.')

    def _on_joy(self, msg: Joy) -> None:
        if 0 <= self._reset_button_index < len(msg.buttons):
            pressed = msg.buttons[self._reset_button_index] == 1
        else:
            pressed = False
            if not self._button_warning_emitted:
                self.get_logger().error(
                    f'reset_button_index {self._reset_button_index} out of range for Joy message '
                    f'({len(msg.buttons)} buttons)'
                )
                self._button_warning_emitted = True

        if pressed and not self._last_button_state:
            self._reset_to_start_angle()
        self._last_button_state = pressed

        if time.time() < self._ready_time:
            self._axis_command = 0.0
            return

        if self._axis_index < 0 or self._axis_index >= len(msg.axes):
            self._axis_command = 0.0
            if not self._axis_warning_emitted:
                self.get_logger().error(
                    f'axis_index {self._axis_index} out of range for Joy message '
                    f'({len(msg.axes)} axes)'
                )
                self._axis_warning_emitted = True
            return

        self._axis_command = self._axis_to_velocity_command(float(msg.axes[self._axis_index]))

    def _on_timer(self) -> None:
        now = time.monotonic()
        dt = now - self._last_update_time
        self._last_update_time = now

        if time.time() < self._ready_time:
            return
        if math.isclose(self._axis_command, 0.0, abs_tol=1e-9):
            return

        requested_angle = (
            self._current_angle_deg + self._axis_command * self._max_velocity_deg_s * dt
        )
        if (
            (self._current_angle_deg <= self._min_angle_deg and requested_angle <= self._min_angle_deg)
            or (self._current_angle_deg >= self._max_angle_deg and requested_angle >= self._max_angle_deg)
        ):
            return

        try:
            self._apply_angle(requested_angle, 'velocity')
        except Exception as exc:
            if not self._write_error_emitted:
                self.get_logger().error(f'Failed to update servos from Joy velocity: {exc}')
                self._write_error_emitted = True


def main() -> None:
    rclpy.init()
    node: Optional[DualServoVelocityControlNode] = None
    try:
        node = DualServoVelocityControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
