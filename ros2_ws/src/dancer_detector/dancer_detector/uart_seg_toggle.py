#!/usr/bin/env python3
"""
UART toggle → start/stop dancer detection with single-byte 'M' or 'P'

- Listens on a serial port (default /dev/ttyAMA0 @ 115200).
- On receiving 'M' or 'P', toggles segmentation state:
    inactive -> START service
    active   -> STOP  service
- After toggling, ignores further triggers for debounce_window_s (default 5.0s).
- Extra triggers during the debounce window are simply discarded.
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_srvs.srv import Trigger
import serial


class UartSegToggle(Node):
    def __init__(self):
        super().__init__('uart_seg_toggle')

        # ---------- Parameters ----------
        p = self.declare_parameter
        self.port = str(p('port', '/dev/ttyAMA0').value)       # Pi 4 primary UART
        self.baud = int(p('baud', 115200).value)
        self.timeout_s = float(p('timeout_s', 0.050).value)    # serial read timeout
        self.debounce_window_s = float(p('debounce_window_s', 5.0).value)
        self.initial_active = bool(p('initial_active', False).value)

        self.start_srv_name = str(p('start_service', '/dancer_detector/start').value)
        self.stop_srv_name  = str(p('stop_service',  '/dancer_detector/stop').value)

        # ---------- Service clients ----------
        self.cli_start = self.create_client(Trigger, self.start_srv_name)
        self.cli_stop  = self.create_client(Trigger, self.stop_srv_name)

        # Try to find services briefly (non-fatal if not yet up)
        t0 = self.get_clock().now()
        while (not self.cli_start.wait_for_service(timeout_sec=0.5) or
               not self.cli_stop.wait_for_service(timeout_sec=0.5)):
            if (self.get_clock().now() - t0) > Duration(seconds=8.0):
                self.get_logger().warn('Start/stop services not available yet; will keep trying.')
                break
            self.get_logger().info('Waiting for start/stop services...')

        # ---------- Serial ----------
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout_s)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open serial port {self.port}: {e}')
            raise
        self.get_logger().info(f'Opened {self.port} @ {self.baud} baud')

        # ---------- State ----------
        self.active = self.initial_active
        self._last_toggle_t = 0.0
        self._stop_evt = threading.Event()

        # Background reader
        self._th = threading.Thread(target=self._serial_loop, daemon=True)
        self._th.start()

    # -------- Serial loop --------
    def _serial_loop(self):
        TRIG_BYTES = (b'M', b'P')
        while not self._stop_evt.is_set():
            try:
                b = self.ser.read(1)
                if not b:
                    continue
                if b in TRIG_BYTES:
                    self._maybe_toggle()
                else:
                    # ignore other bytes
                    pass
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(0.2)

    # -------- Toggle logic --------
    def _maybe_toggle(self):
        now = time.monotonic()
        if (now - self._last_toggle_t) < self.debounce_window_s:
            self.get_logger().debug("Trigger received but ignored (debounce window).")
            return  # discard

        self._last_toggle_t = now
        self.active = not self.active

        if self.active:
            self.get_logger().info("Toggled ON → calling start service")
            self._call_trigger(self.cli_start, 'start')
        else:
            self.get_logger().info("Toggled OFF → calling stop service")
            self._call_trigger(self.cli_stop, 'stop')

    def _call_trigger(self, client, label: str):
        if not client.wait_for_service(timeout_sec=0.0):
            self.get_logger().warn(f'{label} service not available yet.')
            return
        future = client.call_async(Trigger.Request())

        def _done_cb(_):
            try:
                resp = future.result()
                if resp.success:
                    self.get_logger().info(f'{label} OK: {resp.message}')
                else:
                    self.get_logger().warn(f'{label} failed: {resp.message}')
            except Exception as e:
                self.get_logger().error(f'{label} call error: {e}')

        future.add_done_callback(_done_cb)

    # -------- Cleanup --------
    def destroy_node(self):
        try:
            self._stop_evt.set()
            if hasattr(self, 'ser') and self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = UartSegToggle()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
