#!/usr/bin/env python3
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import rclpy
from frame_servo_interfaces.srv import SetServoAngle
from rclpy.node import Node
from sensor_msgs.msg import Joy


def describe_pwm(pwm, label):
    chip = pwm.chip
    ch = pwm.ch

    chip_path = f"/sys/class/pwm/pwmchip{chip}"
    pwm_path = f"{chip_path}/pwm{ch}"
    dev_path = os.path.realpath(f"{chip_path}/device")

    print(f"\n[{label}]")
    print(f"  pwmchip        : pwmchip{chip}")
    print(f"  channel        : pwm{ch}")
    print(f"  pwm sysfs path : {pwm_path}")
    print(f"  device backend : {dev_path}")

    try:
        with open(f"{pwm_path}/period", encoding='utf-8') as handle:
            print(f"  period (ns)    : {handle.read().strip()}")
        with open(f"{pwm_path}/duty_cycle", encoding='utf-8') as handle:
            print(f"  duty (ns)      : {handle.read().strip()}")
        with open(f"{pwm_path}/enable", encoding='utf-8') as handle:
            print(f"  enabled        : {handle.read().strip()}")
    except FileNotFoundError:
        print("  (pwm not yet exported)")


STARTUP_HOLD_S = 3.0
SERVO_HZ = 50.0
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000

SERVO_A_PWMCHIP = 3
SERVO_A_CH = 0

SERVO_B_PWMCHIP = 2
SERVO_B_CH = 0


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def mirror_angle(angle_deg: float, min_angle: float, max_angle: float) -> float:
    angle_deg = clamp(angle_deg, min_angle, max_angle)
    return (min_angle + max_angle) - angle_deg


def angle_to_pulse_us(
    angle_deg: float,
    min_angle: float,
    max_angle: float,
) -> int:
    angle_deg = clamp(angle_deg, min_angle, max_angle)
    span = max_angle - min_angle
    if span <= 0.0:
        raise ValueError("max_angle must be greater than min_angle")
    t = (angle_deg - min_angle) / span
    pulse = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)
    return int(round(pulse))


@dataclass(frozen=True)
class ApplyResult:
    requested_angle: float
    applied_angle: float
    clamped: bool


class SysfsPWM:
    def __init__(self, chip: int, ch: int):
        self.chip = chip
        self.ch = ch
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{chip}")
        self.pwm_path = self.chip_path / f"pwm{ch}"
        if not self.chip_path.exists():
            raise RuntimeError(f"pwmchip{chip} not found")

    def _write(self, path: Path, value: str) -> None:
        path.write_text(str(value), encoding='utf-8')

    def export(self) -> None:
        if not self.pwm_path.exists():
            try:
                self._write(self.chip_path / "export", str(self.ch))
            except Exception:
                pass

            for _ in range(100):
                if self.pwm_path.exists():
                    break
                time.sleep(0.01)

        if not self.pwm_path.exists():
            raise RuntimeError(f"failed to export pwm{self.ch} on pwmchip{self.chip}")

    def enable(self, enabled: bool) -> None:
        try:
            self._write(self.pwm_path / "enable", "1" if enabled else "0")
        except Exception:
            pass

    def set_period_ns(self, period_ns: int) -> None:
        self._write(self.pwm_path / "period", str(period_ns))

    def set_duty_ns(self, duty_ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(duty_ns))

    def setup(self, hz: float, initial_pulse_us: int) -> None:
        period_ns = int(round(1e9 / hz))
        duty_ns = int(initial_pulse_us) * 1000
        duty_ns = max(0, min(duty_ns, period_ns))

        self.export()
        self.enable(False)
        self.set_period_ns(period_ns)
        self.set_duty_ns(duty_ns)
        self.enable(True)

    def set_pulse_us(self, pulse_us: int) -> None:
        self.set_duty_ns(int(pulse_us) * 1000)

    def relax(self) -> None:
        self.enable(False)


class DualServo:
    def __init__(self, pwm_a: SysfsPWM, pwm_b: SysfsPWM):
        self.a = pwm_a
        self.b = pwm_b

    def set_pulses(self, a_pulse_us: int, b_pulse_us: int) -> None:
        self.a.set_pulse_us(a_pulse_us)
        self.b.set_pulse_us(b_pulse_us)


class DualServoControlNode(Node):
    def __init__(self) -> None:
        super().__init__('dual_servo_control')

        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('axis_index', 4)
        self.declare_parameter('invert_axis', False)
        self.declare_parameter('startup_hold_s', STARTUP_HOLD_S)
        self.declare_parameter('reset_button_index', 2)
        self.declare_parameter('reset_angle_deg', 45.0)
        self.declare_parameter('angle_range_deg', 25.0)

        self._axis_index = int(self.get_parameter('axis_index').value)
        self._invert_axis = bool(self.get_parameter('invert_axis').value)
        self._reset_button_index = int(self.get_parameter('reset_button_index').value)
        self._reset_angle_deg = float(self.get_parameter('reset_angle_deg').value)
        self._angle_range_deg = float(self.get_parameter('angle_range_deg').value)
        self._min_angle_deg = self._reset_angle_deg
        self._max_angle_deg = self._reset_angle_deg + self._angle_range_deg
        if self._angle_range_deg <= 0.0:
            raise ValueError('angle_range_deg must be greater than zero')
        if self._max_angle_deg <= self._min_angle_deg:
            raise ValueError('computed max angle must be greater than min angle')

        self._ready_time = time.time() + max(
            0.0,
            float(self.get_parameter('startup_hold_s').value),
        )

        pwm_a = SysfsPWM(SERVO_A_PWMCHIP, SERVO_A_CH)
        pwm_b = SysfsPWM(SERVO_B_PWMCHIP, SERVO_B_CH)

        initial_pulses = self._servo_pulses_for_angle(self._reset_angle_deg)
        pwm_a.setup(SERVO_HZ, initial_pulses[0])
        pwm_b.setup(SERVO_HZ, initial_pulses[1])

        self._pwm_a = pwm_a
        self._pwm_b = pwm_b
        self._servos = DualServo(pwm_a, pwm_b)
        self._last_pulses: Optional[Tuple[int, int]] = initial_pulses
        self._last_button_state = False
        self._axis_warning_emitted = False
        self._button_warning_emitted = False
        self._service_warning_emitted = False
        self._write_error_emitted = False

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
        self._set_angle_client = self.create_client(
            SetServoAngle,
            '/dual_servo_control/set_angle',
        )
        self.get_logger().info(
            f'Listening for Joy on {joy_topic}; axis {self._axis_index}'
            f"{' (inverted)' if self._invert_axis else ''}; "
            f'reset button {self._reset_button_index}; '
            f'range [{self._min_angle_deg:.2f}, {self._max_angle_deg:.2f}] deg.'
        )

    def destroy_node(self) -> bool:
        try:
            self._pwm_a.relax()
            self._pwm_b.relax()
        except Exception:
            pass
        return super().destroy_node()

    def _servo_pulses_for_angle(self, angle_deg: float) -> Tuple[int, int]:
        a_pulse = angle_to_pulse_us(angle_deg, self._min_angle_deg, self._max_angle_deg)
        b_pulse = angle_to_pulse_us(
            mirror_angle(angle_deg, self._min_angle_deg, self._max_angle_deg),
            self._min_angle_deg,
            self._max_angle_deg,
        )
        return a_pulse, b_pulse

    def _apply_angle(self, requested_angle_deg: float, source: str) -> ApplyResult:
        applied_angle = clamp(requested_angle_deg, self._min_angle_deg, self._max_angle_deg)
        pulses = self._servo_pulses_for_angle(applied_angle)

        if self._last_pulses != pulses:
            self._servos.set_pulses(*pulses)
            self._last_pulses = pulses

        self._write_error_emitted = False
        self.get_logger().debug(
            f'Applied {source} angle request {requested_angle_deg:.2f} -> {applied_angle:.2f}'
        )
        return ApplyResult(
            requested_angle=requested_angle_deg,
            applied_angle=applied_angle,
            clamped=not math.isclose(requested_angle_deg, applied_angle, abs_tol=1e-6),
        )

    def _axis_to_angle(self, axis_value: float) -> float:
        if self._invert_axis:
            axis_value = -axis_value
        axis_value = max(-1.0, min(1.0, axis_value))
        t = (axis_value + 1.0) * 0.5
        return self._min_angle_deg + t * (self._max_angle_deg - self._min_angle_deg)

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

    def _send_reset_request(self) -> None:
        if not self._set_angle_client.service_is_ready():
            if not self._service_warning_emitted:
                self.get_logger().warn('Set-angle service is not ready yet; ignoring reset button.')
                self._service_warning_emitted = True
            return

        request = SetServoAngle.Request()
        request.target_angle_deg = float(self._reset_angle_deg)
        future = self._set_angle_client.call_async(request)
        future.add_done_callback(self._on_reset_response)

    def _on_reset_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f'Reset service call failed: {exc}')
            return

        if response.success:
            self.get_logger().info(f'Reset service completed: {response.message}')
        else:
            self.get_logger().error(f'Reset service failed: {response.message}')

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
            self._send_reset_request()
        self._last_button_state = pressed

        if time.time() < self._ready_time:
            return

        if self._axis_index < 0 or self._axis_index >= len(msg.axes):
            if not self._axis_warning_emitted:
                self.get_logger().error(
                    f'axis_index {self._axis_index} out of range for Joy message '
                    f'({len(msg.axes)} axes)'
                )
                self._axis_warning_emitted = True
            return

        try:
            angle_deg = self._axis_to_angle(float(msg.axes[self._axis_index]))
            self._apply_angle(angle_deg, 'joy')
        except Exception as exc:
            if not self._write_error_emitted:
                self.get_logger().error(f'Failed to update servos from Joy: {exc}')
                self._write_error_emitted = True


def main() -> None:
    rclpy.init()
    node: Optional[DualServoControlNode] = None
    try:
        node = DualServoControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
