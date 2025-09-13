#!/usr/bin/env python3
# Depth-only RealSense + simple dancer detection (no ML)
# Requirements: pyrealsense2, numpy, opencv-python, scikit-learn

import pyrealsense2 as rs
import numpy as np
import cv2
from sklearn.cluster import DBSCAN

# ---- Tunables (kept minimal) ----
MIN_Z_M, MAX_Z_M = 0.7, 6.0          # plausible person distance (meters)
KERNEL_CLOSE = (7, 7)                # morphology kernel for closing
MIN_BLOB_AREA_FRAC = 0.001           # drop tiny blobs (<0.1% of frame)
DBSCAN_EPS_M = 0.25                  # 3D clustering radius (meters)
DBSCAN_MIN_SAMPLES = 30
SUBSAMPLE_STRIDE = 3                 # subsample points for speed (>=1)

TEXT_COLOR = (0, 255, 0)

def make_mask(depth_m):
    # Valid, within distance range
    m = (depth_m > 0) & (depth_m >= MIN_Z_M) & (depth_m <= MAX_Z_M)
    return (m.astype(np.uint8) * 255)

def morph_close(mask):
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, KERNEL_CLOSE)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

def remove_small(mask, min_area):
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=4)
    keep = np.zeros_like(mask)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            keep[labels == i] = 255
    return keep

def deproject_batch(intr, uv, z):
    # Vectorized pinhole deprojection (meters)
    fx, fy, ppx, ppy = intr.fx, intr.fy, intr.ppx, intr.ppy
    x = (uv[:, 0] - ppx) / fx * z
    y = (uv[:, 1] - ppy) / fy * z
    return np.column_stack((x, y, z))

def human_like_3d(xyz):
    if xyz.size == 0: return False
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    dx, dy, dz = (maxs - mins)  # width, height, depth (meters)
    return (0.2 <= dx <= 2.5) and (0.3 <= abs(dy) <= 2.5) and (0.1 <= dz <= 2.0)

def main():
    pipeline = rs.pipeline()
    config = rs.config()
    # Depth-only stream (works on depth-only modules like D421)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    depth_stream_profile = profile.get_stream(rs.stream.depth)
    intr = depth_stream_profile.as_video_stream_profile().get_intrinsics()

    colorizer = rs.colorizer()  # for visualizing depth
    colorizer.set_option(rs.option.color_scheme, 0)  # 0=Jet

    cv2.namedWindow("Dancer detection (depth)", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue

            # Depth in meters
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * depth_scale
            H, W = depth_m.shape
            min_area = max(1, int(MIN_BLOB_AREA_FRAC * H * W))

            # 1) Depth mask -> 2) close -> 3) remove small
            mask = make_mask(depth_m)
            mask = morph_close(mask)
            mask = remove_small(mask, min_area)

            # Background for display
            depth_vis = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            overlay = depth_vis.copy()

            # Pixels to cluster
            ys, xs = np.where(mask > 0)
            if ys.size >= DBSCAN_MIN_SAMPLES:
                if SUBSAMPLE_STRIDE > 1:
                    xs = xs[::SUBSAMPLE_STRIDE]
                    ys = ys[::SUBSAMPLE_STRIDE]

                uv = np.column_stack((xs.astype(np.float32), ys.astype(np.float32)))
                z = depth_m[ys, xs].astype(np.float32)

                # 3D points in camera frame (meters)
                xyz = deproject_batch(intr, uv, z)

                # 3D Euclidean clustering
                labels = DBSCAN(eps=DBSCAN_EPS_M, min_samples=DBSCAN_MIN_SAMPLES).fit(xyz).labels_

                best_label = None
                best_count = 0
                # Prefer clusters that look human-like, otherwise take largest
                for l in set(labels):
                    if l == -1:  # noise
                        continue
                    idx = np.where(labels == l)[0]
                    pts = xyz[idx]
                    if human_like_3d(pts):
                        if idx.size > best_count:
                            best_count = idx.size
                            best_label = l

                if best_label is None:
                    # fallback: largest cluster
                    for l in set(labels):
                        if l == -1: continue
                        idx = np.where(labels == l)[0]
                        if idx.size > best_count:
                            best_count = idx.size
                            best_label = l

                if best_label is not None:
                    idx = np.where(labels == best_label)[0]
                    sel_x = xs[idx]
                    sel_y = ys[idx]
                    # 2D bbox
                    x1, y1 = int(sel_x.min()), int(sel_y.min())
                    x2, y2 = int(sel_x.max()), int(sel_y.max())
                    # Median depth from original map
                    med_z = float(np.median(depth_m[sel_y, sel_x]))

                    # Draw
                    cv2.rectangle(overlay, (x1, y1), (x2, y2), TEXT_COLOR, 2)
                    cv2.putText(overlay, f"{med_z:.2f} m", (x1, max(0, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, TEXT_COLOR, 2)
                else:
                    cv2.putText(overlay, "No cluster", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(overlay, "No candidates", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imshow("Dancer detection (depth)", overlay)
            if cv2.waitKey(1) == 27:
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
