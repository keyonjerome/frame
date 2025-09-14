#!/usr/bin/env python3
"""
RealSense depth person-ish mid-range segmentation (simple)
- Streams depth via pyrealsense2
- Masks far-away (and too-near) pixels, keeps middle-range
- Finds the largest/most-central blob and draws a bounding box + centroid
"""

import time
import numpy as np
import cv2
import pyrealsense2 as rs

# ---------------- Tunables ----------------
SIZE = (640, 480)       # (W, H)
FPS = 30

# Keep only "middle" distances (meters)
MID_NEAR_M = 0.8
MID_FAR_M  = 3.0

# Visualization range for colorized depth (meters)
VIZ_MIN_M = 0.2
VIZ_MAX_M = 5.0

# Post-processing
MIN_BLOB_AREA_FRAC = 0.001     # drop tiny blobs (< 0.1% of frame)
KERNEL_SIZE = (5, 5)           # morphology kernel (ellipse)
OPEN_ITERS = 1
CLOSE_ITERS = 1
CENTER_PENALTY = 2.0           # pixels → score penalty weight (bigger = prefer center more)

WINDOW_NAME = "Mid-range person mask (RealSense)"


# ------------- Helpers (standalone) -------------
def colorize_depth(depth_m: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
    """Meters → BGR JET visualization."""
    d = depth_m.copy()
    d[d <= 0] = np.nan  # keep invalid separate
    d = np.clip(d, vmin, vmax)
    # normalize to 0..255 (NaNs become 0 later)
    denom = max(1e-6, (vmin - vmax))  # negative if vmin < vmax
    # handle both orders safely
    if vmax > vmin:
        norm = ( (d - vmin) / (vmax - vmin) * 255.0 )
    else:
        norm = ( (d - vmax) / (vmin - vmax) * 255.0 )
    norm = np.nan_to_num(norm, nan=0.0).astype(np.uint8)
    return cv2.applyColorMap(norm, cv2.COLORMAP_JET)


def mask_midrange(depth_m: np.ndarray, near_m: float, far_m: float) -> np.ndarray:
    """Return uint8 mask (0/255) where depth in [near_m, far_m] and valid."""
    valid = depth_m > 0.0
    mid = (depth_m >= min(near_m, far_m)) & (depth_m <= max(near_m, far_m))
    m = (valid & mid).astype(np.uint8) * 255
    return m


def morph_cleanup(mask: np.ndarray, ksize=(5,5), open_iters=1, close_iters=1) -> np.ndarray:
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ksize)
    out = mask
    if open_iters > 0:
        out = cv2.morphologyEx(out, cv2.MORPH_OPEN, k, iterations=open_iters)
    if close_iters > 0:
        out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, k, iterations=close_iters)
    return out


def remove_small(mask: np.ndarray, min_area: int) -> np.ndarray:
    """Keep only components with area >= min_area."""
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return np.zeros_like(mask)
    out = np.zeros_like(mask)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            out[labels == i] = 255
    return out


def pick_best_component(mask: np.ndarray) -> tuple | None:
    """
    Choose component by area and closeness to image center.
    Score = area - CENTER_PENALTY * distance_to_center_pixels
    Returns (x1, y1, x2, y2, (cx, cy)) or None
    """
    h, w = mask.shape[:2]
    num, labels, stats, cents = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return None

    cx0, cy0 = w * 0.5, h * 0.5
    best = None
    best_score = -1e18

    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area <= 0:
            continue
        cx, cy = cents[i]
        dist = np.hypot(cx - cx0, cy - cy0)
        score = area - CENTER_PENALTY * dist
        if score > best_score:
            best_score = score
            x, y, bw, bh, _ = stats[i]
            best = (int(x), int(y), int(x + bw), int(y + bh), (float(cx), float(cy)))

    return best


# ------------- RealSense setup -------------
def open_rs_depth(width: int, height: int, fps: int):
    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    profile = pipeline.start(cfg)

    # Depth units (meters per Z16 unit), typically 0.001 on D4xx (1 unit = 1 mm)
    try:
        depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
    except Exception as e:
        print(f"[WARN] Could not read depth scale: {e}. Assuming 0.001 m/unit.")
        depth_scale = 0.001
    return pipeline, depth_scale


def get_depth_meters(pipeline: rs.pipeline, depth_scale: float, timeout_ms=200) -> np.ndarray | None:
    try:
        frames = pipeline.wait_for_frames(timeout_ms)
    except Exception:
        return None
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return None
    z16 = np.asanyarray(depth_frame.get_data())  # uint16
    return z16.astype(np.float32) * depth_scale   # meters


# ------------- Main loop -------------
def main():
    try:
        pipeline, depth_scale = open_rs_depth(SIZE[0], SIZE[1], FPS)
    except Exception as e:
        print(f"Failed to open RealSense: {e}")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    target_period = 1.0 / 15.0  # ~15 Hz

    try:
        while True:
            t0 = time.monotonic()

            depth_m = get_depth_meters(pipeline, depth_scale)
            if depth_m is None:
                time.sleep(0.005)
                continue

            h, w = depth_m.shape[:2]

            # 1) Mid-range binary mask
            mask = mask_midrange(depth_m, MID_NEAR_M, MID_FAR_M)

            # 2) Clean up the mask
            min_area = max(1, int(MIN_BLOB_AREA_FRAC * h * w))
            mask = morph_cleanup(mask, KERNEL_SIZE, OPEN_ITERS, CLOSE_ITERS)
            mask = remove_small(mask, min_area)

            # 3) Pick the best blob (area + center bias)
            picked = pick_best_component(mask)

            # 4) Visualize
            vis = colorize_depth(depth_m, VIZ_MIN_M, VIZ_MAX_M)
            overlay = vis.copy()

            # tint mid-range areas (green)
            if np.any(mask):
                green = np.zeros_like(overlay)
                green[:, :] = (0, 255, 0)
                alpha = (mask > 0).astype(np.float32) * 0.35
                overlay = (overlay.astype(np.float32) * (1 - alpha)[..., None] +
                           green.astype(np.float32)   * alpha[..., None]).astype(np.uint8)

            # draw bbox + centroid
            if picked is not None:
                x1, y1, x2, y2, (cx, cy) = picked
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), 3)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cv2.circle(overlay, (int(round(cx)), int(round(cy))), 5, (0, 0, 0), -1)
                cv2.circle(overlay, (int(round(cx)), int(round(cy))), 3, (255, 255, 255), -1)

            # crosshair (visual sanity check for "center bias")
            cv2.drawMarker(overlay, (w // 2, h // 2), (255, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=14, thickness=1)

            cv2.imshow(WINDOW_NAME, overlay)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord('q')):  # ESC or q
                break

            # 5) Pace the loop
            dt = time.monotonic() - t0
            if (sleep := (target_period - dt)) > 0:
                time.sleep(sleep)

    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
