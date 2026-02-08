#!/usr/bin/env python3
import os
import time
from pathlib import Path
from typing import Optional

import rclpy
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
        with open(f"{pwm_path}/period") as f:
            print(f"  period (ns)    : {f.read().strip()}")
        with open(f"{pwm_path}/duty_cycle") as f:
            print(f"  duty (ns)      : {f.read().strip()}")
        with open(f"{pwm_path}/enable") as f:
            print(f"  enabled        : {f.read().strip()}")
    except FileNotFoundError:
        print("  (pwm not yet exported)")


# ===== Joint limits (geometric) =====
MIN_ANGLE = 5
MAX_ANGLE = 25

# ===== Timing =====
STEP_DELAY_S = 0.015
STARTUP_HOLD_S = 3.0

# ===== Servo PWM tuning =====
SERVO_HZ = 50.0
MIN_PULSE_US = 1000
MAX_PULSE_US = 2000

# ===== PWM mappings (based on your debugfs tests) =====
SERVO_A_PWMCHIP = 3
SERVO_A_CH = 0

SERVO_B_PWMCHIP = 2
SERVO_B_CH = 0


def clamp(x: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, x))


def mirrorAngle(angleDeg: int) -> int:
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    return (MIN_ANGLE + MAX_ANGLE) - angleDeg


def angle_to_pulse_us(angleDeg: int) -> int:
    angleDeg = clamp(angleDeg, MIN_ANGLE, MAX_ANGLE)
    span = MAX_ANGLE - MIN_ANGLE
    if span <= 0:
        raise ValueError("MAX_ANGLE must be > MIN_ANGLE")
    t = (angleDeg - MIN_ANGLE) / span
    pulse = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)
    return int(round(pulse))


class SysfsPWM:
    def __init__(self, chip: int, ch: int):
        self.chip = chip
        self.ch = ch
        self.chip_path = Path(f"/sys/class/pwm/pwmchip{chip}")
        self.pwm_path = self.chip_path / f"pwm{ch}"
        if not self.chip_path.exists():
            raise RuntimeError(f"pwmchip{chip} not found")

    def _write(self, path: Path, value: str) -> None:
        path.write_text(str(value))

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
            raise RuntimeError(f"Failed to export pwm{self.ch} on pwmchip{self.chip}")

    def enable(self, en: bool) -> None:
        try:
            self._write(self.pwm_path / "enable", "1" if en else "0")
        except Exception:
            pass

    def set_period_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "period", str(ns))

    def set_duty_ns(self, ns: int) -> None:
        self._write(self.pwm_path / "duty_cycle", str(ns))

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
    def __init__(self, pwmA: SysfsPWM, pwmB: SysfsPWM):
        self.a = pwmA
        self.b = pwmB

    def set_geometric(self, geomDeg: int) -> None:
        geomDeg = clamp(geomDeg, MIN_ANGLE, MAX_ANGLE)
        a_pulse = angle_to_pulse_us(geomDeg)
        b_pulse = angle_to_pulse_us(mirrorAngle(geomDeg))
        self.a.set_pulse_us(a_pulse)
        self.b.set_pulse_us(b_pulse)


class DualServoJoyNode(Node):
    def __init__(self) -> None:
        super().__init__('dual_servo_joy')
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('axis_index', 4)
        self.declare_parameter('invert_axis', False)
        self.declare_parameter('startup_hold_s', STARTUP_HOLD_S)

        self._axis_index = int(self.get_parameter('axis_index').value)
        self._invert_axis = bool(self.get_parameter('invert_axis').value)
        self._ready_time = time.time() + max(
            0.0, float(self.get_parameter('startup_hold_s').value)
        )

        pwmA = SysfsPWM(SERVO_A_PWMCHIP, SERVO_A_CH)
        pwmB = SysfsPWM(SERVO_B_PWMCHIP, SERVO_B_CH)

        init_geom = MIN_ANGLE
        pwmA.setup(SERVO_HZ, angle_to_pulse_us(init_geom))
        pwmB.setup(SERVO_HZ, angle_to_pulse_us(mirrorAngle(init_geom)))

        self._pwmA = pwmA
        self._pwmB = pwmB
        self._servos = DualServo(pwmA, pwmB)
        self._last_angle: Optional[int] = None
        self._axis_warning_emitted = False
        self._write_error_emitted = False

        self.get_logger().info('=== PWM DEVICES IN USE ===')
        describe_pwm(pwmA, 'Servo A (pin 32)')
        describe_pwm(pwmB, 'Servo B (pin 33)')
        self.get_logger().info('==========================')

        joy_topic = str(self.get_parameter('joy_topic').value).strip() or 'joy'
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)
        self.get_logger().info(
            f'Listening for Joy on {joy_topic}; axis {self._axis_index}'
            f"{' (inverted)' if self._invert_axis else ''}."
        )

    def destroy_node(self) -> bool:
        try:
            self._pwmA.relax()
            self._pwmB.relax()
        except Exception:
            pass
        return super().destroy_node()

    def _axis_to_angle(self, axis_value: float) -> int:
        if self._invert_axis:
            axis_value = -axis_value
        axis_value = max(-1.0, min(1.0, axis_value))
        t = (axis_value + 1.0) * 0.5
        angle = MIN_ANGLE + t * (MAX_ANGLE - MIN_ANGLE)
        return int(round(angle))

    def _on_joy(self, msg: Joy) -> None:
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

        angle = self._axis_to_angle(float(msg.axes[self._axis_index]))
        if self._last_angle == angle:
            return

        try:
            self._servos.set_geometric(angle)
        except Exception as exc:
            if not self._write_error_emitted:
                self.get_logger().error(f'Failed to update servos: {exc}')
                self._write_error_emitted = True
            return

        self._last_angle = angle


def main() -> None:
    rclpy.init()
    node: Optional[DualServoJoyNode] = None
    try:
        node = DualServoJoyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
