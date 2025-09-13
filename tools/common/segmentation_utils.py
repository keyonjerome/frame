#!/usr/bin/env python3
"""
Lightweight depth-based segmentation utilities.
"""
from typing import Optional, Tuple
import numpy as np
import cv2


def make_mask(depth_m: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    valid = (depth_m > 0.0) & (depth_m >= float(z_min)) & (depth_m <= float(z_max))
    return (valid.astype(np.uint8) * 255)


essel = cv2.getStructuringElement


def morph_close(mask: np.ndarray, kernel=(5, 5), iterations: int = 1) -> np.ndarray:
    k = essel(cv2.MORPH_ELLIPSE, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=iterations)


def remove_small(mask: np.ndarray, min_area: int) -> np.ndarray:
    num, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return np.zeros_like(mask)
    keep = np.zeros_like(mask)
    for i in range(1, num):
        if stats[i, cv2.CC_STAT_AREA] >= int(min_area):
            keep[labels == i] = 255
    return keep


def segment_largest_component(
    depth_m: np.ndarray,
    z_min: float,
    z_max: float,
    kernel=(5, 5),
    min_area_frac: float = 0.002,
) -> Tuple[np.ndarray, Optional[Tuple[int, int, int, int]], Optional[Tuple[float, float]]]:
    """Return (mask, bbox(x1,y1,x2,y2) or None, centroid(x,y) or None)."""
    mask = make_mask(depth_m, z_min, z_max)
    mask = morph_close(mask, kernel, iterations=1)

    h, w = mask.shape
    min_area = max(1, int(float(min_area_frac) * h * w))
    num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if num <= 1:
        return mask, None, None

    best_i, best_area = None, 0
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area and area > best_area:
            best_area = area
            best_i = i

    if best_i is None:
        return mask, None, None

    x, y, ww, hh, _ = stats[best_i]
    bbox = (int(x), int(y), int(x + ww), int(y + hh))
    cx, cy = centroids[best_i]
    return mask, bbox, (float(cx), float(cy))


def median_depth_around(depth_m: np.ndarray, cx: float, cy: float, win: int = 15) -> Optional[float]:
    h, w = depth_m.shape
    half = int(win) // 2
    xi = int(round(cx))
    yi = int(round(cy))
    x0 = max(0, xi - half)
    x1 = min(w, xi + half + 1)
    y0 = max(0, yi - half)
    y1 = min(h, yi + half + 1)
    roi = depth_m[y0:y1, x0:x1]
    valid = roi > 0
    if not np.any(valid):
        return None
    return float(np.median(roi[valid]))
