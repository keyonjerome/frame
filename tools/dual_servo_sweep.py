#!/usr/bin/env python3
import time
import signal
import sys

import Jetson.GPIO as GPIO

# -------- Pin config (Jetson 40-pin header) --------
# You said you set up pins 32 and 33 for PWM in jetson-io.
# In Jetson.GPIO we’ll use BOARD numbering to match header pin numbers.
SERVO_A_PIN = 32
SERVO_B_PIN = 33

# -------- Motion limits (your sketch’s “geometric” angle range) --------
MIN_ANGLE = 5
MAX_ANGLE = 25

# -------- Servo pulse tuning (IMPORTANT) --------
# Start with conservative standard servo range.
# Period at 50 Hz is 20,000 us.
PWM_HZ = 50
MIN_PULSE_US = 1000   # try 900–1100 if needed
MAX_PULSE_US = 2000   # try 1900–2100 if needed

STEP_DELAY_S = 0.015  # ~15 ms like your Arduino delay(15)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def mirror_angle(geom_deg: int) -> int:
    """
    Mirror around the midpoint of [MIN_ANGLE, MAX_ANGLE] so both servos
    reach the same *geometric* angle even if one is mounted opposite.
    """
    geom_deg = clamp(geom_deg, MIN_ANGLE, MAX_ANGLE)
    return (MIN_ANGLE + MAX_ANGLE) - geom_deg

def angle_to_duty_cycle(angle_deg: int) -> float:
    """
    Map an angle in [MIN_ANGLE, MAX_ANGLE] into pulse width in [MIN_PULSE_US, MAX_PULSE_US],
    then convert to duty cycle percentage for Jetson.GPIO PWM.
    """
    angle_deg = clamp(angle_deg, MIN_ANGLE, MAX_ANGLE)

    # Linear map angle -> pulse_us
    span_deg = (MAX_ANGLE - MIN_ANGLE)
    if span_deg <= 0:
        raise ValueError("MAX_ANGLE must be > MIN_ANGLE")

    t = (angle_deg - MIN_ANGLE) / span_deg  # 0..1
    pulse_us = MIN_PULSE_US + t * (MAX_PULSE_US - MIN_PULSE_US)

    period_us = 1_000_000.0 / PWM_HZ
    duty = (pulse_us / period_us) * 100.0
    return duty

def set_servo(pwm, angle_deg: int):
    duty = angle_to_duty_cycle(angle_deg)
    pwm.ChangeDutyCycle(duty)

def cleanup(pwmA, pwmB):
    # Optionally “relax” servos by setting duty to 0 (stops pulses).
    # Some servos will hold last position without pulses; others may drift.
    pwmA.ChangeDutyCycle(0)
    pwmB.ChangeDutyCycle(0)
    time.sleep(0.1)
    pwmA.stop()
    pwmB.stop()
    GPIO.cleanup()

def main():

    GPIO.setmode(GPIO.BOARD)
    print(GPIO.gpio_function(33))
    print(GPIO.model)
    GPIO.setup(SERVO_A_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(SERVO_B_PIN, GPIO.OUT, initial=GPIO.LOW)


    for p in [32,33]:
        try:
            print(p, GPIO.gpio_function(p))
        except Exception as e:
            print("err", p, e)

    print("JETSON_INFO", getattr(GPIO, "JETSON_INFO", None))
    print("BOARD_INFO", getattr(GPIO, "BOARD_INFO", None))


    pwmA = GPIO.PWM(SERVO_A_PIN, PWM_HZ)
    pwmB = GPIO.PWM(SERVO_B_PIN, PWM_HZ)

    # Start PWM with 0 duty, then set to target to avoid a jump on some setups
    pwmA.start(0)
    pwmB.start(0)
    time.sleep(0.1)

    # 1) Set both to the same geometric min angle position
    geom0 = MIN_ANGLE
    set_servo(pwmA, geom0)
    set_servo(pwmB, mirror_angle(geom0))

    # 2) Wait 3 seconds
    time.sleep(3.0)

    # Handle Ctrl+C nicely
    def handle_sigint(signum, frame):
        cleanup(pwmA, pwmB)
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    while True:
        # 3) Sweep MIN -> MAX
        for geom in range(MIN_ANGLE, MAX_ANGLE + 1):
            set_servo(pwmA, geom)
            set_servo(pwmB, mirror_angle(geom))
            time.sleep(STEP_DELAY_S)

        # Sweep MAX -> MIN
        for geom in range(MAX_ANGLE, MIN_ANGLE - 1, -1):
            set_servo(pwmA, geom)
            set_servo(pwmB, mirror_angle(geom))
            time.sleep(STEP_DELAY_S)

if __name__ == "__main__":
    main()
