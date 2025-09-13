#!/usr/bin/env python3
# Minimal RealSense demo without color sensor: show depth (and optional IR)

import pyrealsense2 as rs
import numpy as np
import cv2

def main():
    pipeline = rs.pipeline()
    config = rs.config()

    # Enable DEPTH only (works on depth-only modules like D421)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    # Try to also enable infrared (optional; if not present, we'll ignore)
    enable_ir = True
    try:
        # Infrared comes in two sensors on stereo modules: index 1 (left), 2 (right)
        # We’ll enable the first (left) if available.
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
    except Exception:
        enable_ir = False

    # Start streaming
    profile = pipeline.start(config)

    # Get depth scale to map depth units to meters (optional, but handy)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()  # meters per unit

    # Prepare colorizer for nicer visualization of depth
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.color_scheme, 2)  # 0=Jet, 2=Black-to-White, etc.

    cv2.namedWindow("RealSense Depth", cv2.WINDOW_AUTOSIZE)
    if enable_ir:
        cv2.namedWindow("RealSense IR", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Colorize depth for display
            depth_color = np.asanyarray(colorizer.colorize(depth_frame).get_data())

            # Show depth image
            cv2.imshow("RealSense Depth", depth_color)

            # If IR was enabled and available, show it too
            if enable_ir:
                ir_frame = frames.get_infrared_frame(1)  # left IR
                if ir_frame:
                    ir_image = np.asanyarray(ir_frame.get_data())
                    cv2.imshow("RealSense IR", ir_image)

            # Exit on ESC
            if cv2.waitKey(1) == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
