#!/usr/bin/env python3
import os
import sys
import time
import traceback

# Jetson.GPIO may not detect the model in some environments; force Orin Nano.
os.environ.setdefault("JETSON_MODEL_NAME", "JETSON_ORIN_NANO")

import Jetson.GPIO as GPIO
import Jetson.GPIO.gpio as _gpio

PIN1 = 32  # GPIO07 on 40-pin header
PIN2 = 33  # GPIO13 on 40-pin header

FREQ = 50  # Hz
MIN_US = 500   # adjust if needed (common ranges: 500-2500 or 1000-2000)
MAX_US = 2500
TEST_ANGLES = (0, 90, 180)
DELAY_SEC = 1.0

def angle_to_duty(angle):
    angle = max(0, min(180, angle))
    pulse_us = MIN_US + (MAX_US - MIN_US) * (angle / 180.0)
    period_us = 1_000_000 / FREQ
    return (pulse_us / period_us) * 100.0

def set_angle(pwm, angle):
    pwm.ChangeDutyCycle(angle_to_duty(angle))

GPIO.setwarnings(False)
GPIO.cleanup()

GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN1, GPIO.OUT)
GPIO.setup(PIN2, GPIO.OUT)

pwm1 = GPIO.PWM(PIN1, FREQ)
pwm2 = GPIO.PWM(PIN2, FREQ)
pwm1.start(0)
pwm2.start(0)

try:
    for angle in TEST_ANGLES:
        set_angle(pwm1, angle)
        set_angle(pwm2, angle)
        time.sleep(DELAY_SEC)
finally:
    first_exc = None
    first_tb = None

    def _try(label, fn):
        nonlocal first_exc, first_tb
        try:
            fn()
        except Exception as exc:
            if first_exc is None:
                first_exc = exc
                first_tb = sys.exc_info()[2]
            sys.stderr.write(f"[cleanup] {label} failed:\\n{traceback.format_exc()}\\n")

    _try("pwm1.stop()", pwm1.stop)
    _try("pwm2.stop()", pwm2.stop)
    _try("GPIO.cleanup()", GPIO.cleanup)

    # Prevent Jetson.GPIO PWM __del__ from running during interpreter shutdown.
    _gpio.PWM.__del__ = lambda self: None

    if first_exc is not None:
        raise first_exc.with_traceback(first_tb)
