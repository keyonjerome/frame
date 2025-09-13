#!/usr/bin/env python3
"""
Simple UVC depth segmentation demo (real-time)
- Captures depth from camera id=0 (Y16 UVC)
- Segments the darkest red (farthest in JET colormap) region by thresholding normalized depth
- Finds the best cluster and draws centroid and bounding box on colorized depth
- Minimal dependencies and overhead
"""
import time
from typing import Optional

import numpy as np
import cv2

from common.uvc_depth_utils import open_uvc_depth, read_depth_meters, colorize_depth
from common.segmentation_utils import morph_close, remove_small

# ---- Tunables ----
SIZE = (640, 480)
FPS = 30
DEPTH_MIN_M, DEPTH_MAX_M = 0.01, 20.0   # normalization range for visualization and mask
RED_MIN_NORM = 180                      # 0..255 threshold; higher = only very red
MIN_BLOB_AREA_FRAC = 0.001              # drop tiny blobs
MORPH_KERNEL = (7, 7)
ALPHA_MASK = 0.35                       # overlay transparency

WINDOW_NAME = "Depth Red-Cluster Segmentation"


def main():
    try:
        cap = open_uvc_depth(0, SIZE, FPS)
    except Exception as e:
        print(f"Failed to open UVC depth camera: {e}")
        return

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    period = 1.0 / 15.0  # target ~15 Hz processing

    try:
        while True:
            t0 = time.monotonic()

            depth_m = read_depth_meters(cap)
            if depth_m is None:
                time.sleep(0.005)
                continue

            # Normalize to 2D (H,W)
            if depth_m.ndim == 3 and depth_m.shape[2] == 1:
                depth_m = depth_m[..., 0]
            elif depth_m.ndim != 2:
                depth_m = np.squeeze(depth_m)
                if depth_m.ndim != 2:
                    continue

            H, W = depth_m.shape

            # Compute normalized 0..255 map consistent with colorize_depth
            d = depth_m.copy()
            d[d < DEPTH_MIN_M] = DEPTH_MIN_M
            d[d > DEPTH_MAX_M] = DEPTH_MAX_M
            norm = ((d - DEPTH_MIN_M) / max(1e-6, (DEPTH_MAX_M - DEPTH_MIN_M)) * 255.0).astype(np.uint8)
            norm[depth_m <= 0] = 0

            # Threshold for darkest red regions (highest norm values)
            mask = (norm >= RED_MIN_NORM).astype(np.uint8) * 255
            mask = morph_close(mask, MORPH_KERNEL, iterations=1)

            # Remove small blobs
            min_area = max(1, int(MIN_BLOB_AREA_FRAC * H * W))
            mask = remove_small(mask, min_area)

            # Connected components and select component with highest median norm
            num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
            bbox = None
            centroid = None
            best_score = -1.0
            if num > 1:
                for i in range(1, num):
                    area = stats[i, cv2.CC_STAT_AREA]
                    if area < min_area:
                        continue
                    ys, xs = np.where(labels == i)
                    if xs.size == 0:
                        continue
                    med_norm = float(np.median(norm[ys, xs]))
                    score = med_norm  # prefer redder clusters
                    if score > best_score:
                        best_score = score
                        x, y, w, h, _ = stats[i]
                        bbox = (int(x), int(y), int(x + w), int(y + h))
                        cx, cy = centroids[i]
                        centroid = (float(cx), float(cy))

            # Visualization
            depth_vis = colorize_depth(depth_m, vmin=DEPTH_MIN_M, vmax=DEPTH_MAX_M)
            overlay = depth_vis.copy()

            # Draw filled mask with transparency (for all red regions)
            if np.any(mask):
                colored_mask = np.zeros_like(overlay)
                colored_mask[:, :] = (0, 0, 255)  # red overlay
                alpha = (mask > 0).astype(np.float32) * ALPHA_MASK
                overlay = (overlay.astype(np.float32) * (1 - alpha)[..., None] + colored_mask.astype(np.float32) * alpha[..., None]).astype(np.uint8)

            if bbox is not None:
                x1, y1, x2, y2 = bbox
                # High-contrast bbox and centroid
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), 3)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)
            if centroid is not None:
                cx, cy = centroid
                cv2.circle(overlay, (int(round(cx)), int(round(cy))), 5, (0, 0, 0), -1)
                cv2.circle(overlay, (int(round(cx)), int(round(cy))), 3, (255, 255, 255), -1)

            cv2.imshow(WINDOW_NAME, overlay)
            if cv2.waitKey(1) == 27:
                break

            # Pace loop
            dt = time.monotonic() - t0
            sleep_t = period - dt
            if sleep_t > 0:
                time.sleep(sleep_t)

    finally:
        try:
            cap.release()
        except Exception:
            pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
