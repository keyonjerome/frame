#!/usr/bin/env python3
# Minimal UVC depth demo via OpenCV (Y16) on camera id=0
# Assumes the device exposes a 16-bit depth stream (millimeters) as UVC.
# Visualizes depth (colorized). No pyrealsense2 required.

import cv2
import numpy as np

CAM_ID = 0
WIDTH, HEIGHT, FPS = 640, 480, 30
# Many depth UVC streams output millimeters as uint16
DEPTH_SCALE_M_PER_UNIT = 0.001  # 1 mm -> 0.001 m
# For visualization clamp range (meters)
VIS_MIN_M, VIS_MAX_M = 0.02, 10.0


def open_uvc_depth(cam_id=CAM_ID, width=WIDTH, height=HEIGHT, fps=FPS):
    cap = cv2.VideoCapture(cam_id)
    # Try to disable color conversion (keep native Y16)
    try:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    except Exception:
        pass
    # Request Y16 fourcc where supported
    try:
        fourcc = cv2.VideoWriter_fourcc('Y', '1', '6', ' ')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    except Exception:
        pass

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open camera id={cam_id}")
    return cap


def colorize_depth(depth_m: np.ndarray, vmin=VIS_MIN_M, vmax=VIS_MAX_M) -> np.ndarray:
    d = depth_m.copy()
    # Clamp for visualization
    d[d < vmin] = vmin
    d[d > vmax] = vmax
    # Normalize to 0..255
    norm = ((d - vmin) / (vmax - vmin) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


def main():
    try:
        cap = open_uvc_depth()
    except Exception as e:
        print(f"Could not start UVC depth camera: {e}")
        return

    cv2.namedWindow("UVC Depth", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[WARN] Failed to read frame")
                continue

            # Expect a single-channel 16-bit frame (uint16). Some backends may deliver 8-bit.
            if frame.dtype != np.uint16:
                # Attempt to keep raw data by avoiding conversions; warn once
                # Convert to 16-bit if possible (fallback will treat as 8-bit depth indices)
                # Here we just upcast to uint16 to keep pipeline consistent
                frame = frame.astype(np.uint16)

            # Convert to meters using scale
            depth_m = frame.astype(np.float32) * DEPTH_SCALE_M_PER_UNIT

            # Colorize for display
            depth_color = colorize_depth(depth_m)

            cv2.imshow("UVC Depth", depth_color)
            if cv2.waitKey(1) == 27:  # ESC
                break
    finally:
        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
