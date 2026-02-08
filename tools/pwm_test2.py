#!/usr/bin/env python3
import time
import Jetson.GPIO as GPIO

PIN = 33

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

print("before setup gpio_function:", GPIO.gpio_function(PIN))  # expect 0 (IN)

GPIO.setup(PIN, GPIO.OUT, initial=GPIO.LOW)
print("after setup gpio_function:", GPIO.gpio_function(PIN))   # expect 1 (OUT)

for i in range(20):
    GPIO.output(PIN, GPIO.HIGH)
    time.sleep(0.25)
    GPIO.output(PIN, GPIO.LOW)
    time.sleep(0.25)

GPIO.cleanup()
print("done")
