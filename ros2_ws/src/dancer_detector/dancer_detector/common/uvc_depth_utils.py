#!/usr/bin/env python3
"""
UVC depth helpers using OpenCV VideoCapture.
Assumes the device exposes a 16-bit depth stream (uint16) where units are millimeters.
Handles common backend variations (uint16 2D, uint8 2-channel packed, uint16 3D with singleton channel, 8-bit fallback).
"""
from typing import Optional, Tuple
import cv2
import numpy as np

DEFAULT_CAM_ID = 0
DEFAULT_SIZE = (640, 480)
DEFAULT_FPS = 30
DEFAULT_SCALE_M_PER_UNIT = 0.001  # 1 mm -> 0.001 m


def open_uvc_depth(cam_id: int = DEFAULT_CAM_ID, size: Tuple[int, int] = DEFAULT_SIZE, fps: int = DEFAULT_FPS) -> cv2.VideoCapture:
    w, h = size
    cap = cv2.VideoCapture(cam_id)
    # Keep native format (avoid RGB conversion)
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
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open UVC depth camera id={cam_id}")
    return cap


def _to_uint16_depth(frame: np.ndarray) -> Optional[np.ndarray]:
    """Convert various frame layouts to uint16 depth map (2D)."""
    if frame is None:
        return None
    # 2D frames
    if frame.ndim == 2:
        if frame.dtype == np.uint16:
            return frame
        elif frame.dtype == np.uint8:
            return (frame.astype(np.uint16) << 8)
        else:
            return frame.astype(np.uint16)
    # 3D frames
    if frame.ndim == 3:
        h, w, c = frame.shape
        if frame.dtype == np.uint16:
            if c == 1:
                return frame[..., 0]
            # Unexpected, but pick first channel
            return frame[..., 0]
        if frame.dtype == np.uint8:
            if c == 2:
                # Little-endian combine: low byte + (high byte << 8)
                return frame[..., 0].astype(np.uint16) | (frame[..., 1].astype(np.uint16) << 8)
            if c == 1:
                return (frame[..., 0].astype(np.uint16) << 8)
            if c == 3:
                # Fallback: grayscale (not true depth)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                return (gray.astype(np.uint16) << 8)
        # Other dtypes: fallback to first channel
        return frame[..., 0].astype(np.uint16)
    # Unsupported layout
    return None


def read_depth_meters(cap: cv2.VideoCapture, scale_m_per_unit: float = DEFAULT_SCALE_M_PER_UNIT) -> Optional[np.ndarray]:
    ok, frame = cap.read()
    if not ok:
        return None
    depth_u16 = _to_uint16_depth(frame)
    if depth_u16 is None:
        return None
    depth_m = depth_u16.astype(np.float32) * float(scale_m_per_unit)
    return depth_m


def colorize_depth(depth_m: np.ndarray, vmin: float = 0.01, vmax: float = 20.0) -> np.ndarray:
    d = depth_m.copy()
    d[d < vmin] = vmin
    d[d > vmax] = vmax
    # Normalize to 8-bit and apply colormap
    norm = ((d - vmin) / (vmax - vmin) * 255.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)
