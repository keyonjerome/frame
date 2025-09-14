#!/usr/bin/env python3
"""
ROS 2 Humble node: depth-only segmentation → bbox + center-depth (weighted)
Publishes:
- /dancer_detector/overlay   (sensor_msgs/Image, BGR8)  — visualization
- /person_center_depth       (example_interfaces/Float32MultiArray) — [u_px, z_m]
- /dancer_detector/bbox      (example_interfaces/Int32MultiArray)   — [x1,y1,x2,y2] or empty
Services:
- /dancer_detector/start (std_srvs/Trigger)
- /dancer_detector/stop  (std_srvs/Trigger)
"""
import time
from typing import Optional, Tuple

import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from example_interfaces.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

# Your helpers
from dancer_detector.common.uvc_depth_utils import open_uvc_depth, read_depth_meters, colorize_depth
from dancer_detector.common.segmentation_utils import morph_close, remove_small

from rclpy.parameter import Parameter


class DancerSegNode(Node):
    def __init__(self):
        super().__init__('dancer_seg')

        # -------- Parameters (declare → get) --------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self.declare_parameter('depth_min_m', 0.2)
        self.declare_parameter('depth_max_m', 6.0)
        self.declare_parameter('red_min_norm', 180)          # 0..255
        self.declare_parameter('min_blob_area_frac', 0.003)  # fraction of image
        self.declare_parameter('morph_kernel_w', 7)
        self.declare_parameter('morph_kernel_h', 7)
        self.declare_parameter('alpha_mask', 0.35)
        self.declare_parameter('detection_mode', 'near')     # 'near' or 'far_red'
        self.declare_parameter('blue_max_norm', 30)          # 0..255, lower = only very near

        self.declare_parameter('backend', 'realsense')       # 'realsense' or 'uvc'
        self.declare_parameter('proc_hz', 15.0)

        # NEW: Gaussian sigma for weighted averaging inside bbox (in pixels)
        self.declare_parameter('gauss_sigma_px', 40.0)

        self.camera_id = int(self.get_parameter('camera_id').value)
        W = int(self.get_parameter('width').value)
        H = int(self.get_parameter('height').value)
        self.size = (W, H)
        self.fps = int(self.get_parameter('fps').value)

        self.depth_min = float(self.get_parameter('depth_min_m').value)
        self.depth_max = float(self.get_parameter('depth_max_m').value)
        self.red_min_norm = int(self.get_parameter('red_min_norm').value)
        self.min_blob_area_frac = float(self.get_parameter('min_blob_area_frac').value)
        self.kernel_wh = (
            int(self.get_parameter('morph_kernel_w').value),
            int(self.get_parameter('morph_kernel_h').value),
        )
        self.alpha_mask = float(self.get_parameter('alpha_mask').value)
        self.proc_hz = float(self.get_parameter('proc_hz').value)
        self.detection_mode = str(self.get_parameter('detection_mode').value)
        self.blue_max_norm = int(self.get_parameter('blue_max_norm').value)
        self.gauss_sigma_px = float(self.get_parameter('gauss_sigma_px').value)

        # -------- Publishers / Services --------
        self.pub_overlay = self.create_publisher(Image, '/dancer_detector/overlay', 1)
        self.pub_center_depth = self.create_publisher(Float32MultiArray, '/person_center_depth', 10)
        self.pub_bbox = self.create_publisher(Int32MultiArray, '/dancer_detector/bbox', 10)

        self.srv_start = self.create_service(Trigger, '/dancer_detector/start', self.on_start)
        self.srv_stop  = self.create_service(Trigger, '/dancer_detector/stop', self.on_stop)

        self.bridge = CvBridge()
        self.cap = None
        self.active = False

        # Start inactive (so your UART can start it)
        self.timer = self.create_timer(1.0 / max(1.0, self.proc_hz), self.step)

        self.get_logger().info('dancer_seg node initialized (inactive). Call /dancer_detector/start to begin.')

    # ---------- Services ----------
    def on_start(self, req, resp):
        if self.cap is None:
            try:
                if str(self.get_parameter('backend').value) == 'realsense':
                    from dancer_detector.common.rs_depth_utils import RSDepth
                    self.cap = RSDepth(self.size[0], self.size[1], self.fps)
                    self.read_depth = self.cap.read_depth_meters
                    self.release_cap = self.cap.release
                    self.get_logger().info('Using RealSense backend.')
                else:
                    self.cap = open_uvc_depth(self.camera_id, self.size, self.fps)
                    self.read_depth = lambda: read_depth_meters(self.cap)
                    self.release_cap = lambda: self.cap.release()
                    self.get_logger().info('Using UVC backend.')
            except Exception as e:
                resp.success = False
                resp.message = f'Failed to open depth backend: {e}'
                self.get_logger().error(resp.message)
                return resp
        self.active = True
        resp.success = True
        resp.message = 'started'
        self.get_logger().info('Started segmentation.')
        return resp

    def on_stop(self, req, resp):
        self.active = False
        resp.success = True
        resp.message = 'stopped'
        self.get_logger().info('Stopped segmentation.')
        return resp

    # ---------- Utils ----------
    @staticmethod
    def _gaussian_weights(h: int, w: int, sigma_px: float, center_xy: Tuple[float, float]):
        """Create a Gaussian weight map of shape (h, w) centered at center_xy (in ROI coords)."""
        if sigma_px <= 0.0:
            return np.ones((h, w), dtype=np.float32)
        cy, cx = center_xy  # note: row (y), col (x)
        ys = np.arange(h, dtype=np.float32)[:, None]
        xs = np.arange(w, dtype=np.float32)[None, :]
        dy2 = (ys - cy) ** 2
        dx2 = (xs - cx) ** 2
        r2 = dx2 + dy2
        wmap = np.exp(-0.5 * r2 / (sigma_px ** 2))
        return wmap.astype(np.float32)

    # ---------- Main loop ----------
    def step(self):
        if not self.active or self.cap is None:
            return

        # Refresh tunable params at runtime
        try:
            self.detection_mode = str(self.get_parameter('detection_mode').value)
            self.blue_max_norm = int(self.get_parameter('blue_max_norm').value)
            self.red_min_norm = int(self.get_parameter('red_min_norm').value)
            self.depth_min = float(self.get_parameter('depth_min_m').value)
            self.depth_max = float(self.get_parameter('depth_max_m').value)
            self.gauss_sigma_px = float(self.get_parameter('gauss_sigma_px').value)
        except Exception:
            pass

        depth_m = self.read_depth()
        if depth_m is None:
            return

        # Normalize to 2D (H,W)
        if depth_m.ndim == 3 and depth_m.shape[2] == 1:
            depth_m = depth_m[..., 0]
        elif depth_m.ndim != 2:
            depth_m = np.squeeze(depth_m)
            if depth_m.ndim != 2:
                return

        H, W = depth_m.shape

        # -------- Validity mask (raw) --------
        # Valid if >0, finite
        valid_raw = np.isfinite(depth_m) & (depth_m > 0.0)

        # Copy and clamp for normalization/visuals
        d = depth_m.copy()
        d[~valid_raw] = 0.0  # keep invalids at 0 (will be excluded later)
        d_clamped = d.copy()
        d_clamped[d_clamped < self.depth_min] = self.depth_min
        d_clamped[d_clamped > self.depth_max] = self.depth_max

        # Normalized 0..255 depth for thresholding (consistent with JET color)
        denom = max(1e-6, (self.depth_max - self.depth_min))
        norm = ((d_clamped - self.depth_min) / denom * 255.0).astype(np.uint8)
        norm[~valid_raw] = 0

        # -------- Foreground mask (apply validity!) --------
        if self.detection_mode.lower() == 'near':
            mask = (norm <= self.blue_max_norm)
        else:
            mask = (norm >= self.red_min_norm)

        # Enforce validity and morphology
        mask = (mask & valid_raw).astype(np.uint8) * 255
        mask = morph_close(mask, self.kernel_wh, iterations=1)

        # Remove small blobs
        min_area = max(1, int(self.min_blob_area_frac * H * W))
        mask = remove_small(mask, min_area)

        # Connected components on mask
        num, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        bbox: Optional[Tuple[int, int, int, int]] = None
        centroid: Optional[Tuple[float, float]] = None
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
                # Prefer nearer (lower norm) in 'near' mode; farthest in 'far_red'
                score = (255.0 - med_norm) if (self.detection_mode.lower() == 'near') else med_norm
                if score > best_score:
                    best_score = score
                    x, y, w, h, _ = stats[i]
                    bbox = (int(x), int(y), int(x + w), int(y + h))
                    cx, cy = centroids[i]
                    centroid = (float(cx), float(cy))

        # Visualization base
        depth_vis = colorize_depth(depth_m, vmin=self.depth_min, vmax=self.depth_max)
        overlay = depth_vis.copy()

        # Overlay the (valid & selected) mask for visualization
        if np.any(mask):
            colored_mask = np.zeros_like(overlay)
            colored_mask[:, :] = (0, 0, 255)  # red overlay
            alpha = (mask > 0).astype(np.float32) * self.alpha_mask
            overlay = (overlay.astype(np.float32) * (1 - alpha)[..., None] +
                       colored_mask.astype(np.float32) * alpha[..., None]).astype(np.uint8)

        # Draw bbox/centroid and compute weighted averages inside bbox
        u_px = None
        z_m = None
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), 3)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)

            roi_depth = depth_m[y1:y2, x1:x2]
            roi_valid = valid_raw[y1:y2, x1:x2]

            h_roi, w_roi = roi_depth.shape
            if h_roi > 0 and w_roi > 0:
                # Gaussian weights centered at bbox center (ROI coords)
                cx_roi = 0.5 * (w_roi - 1)
                cy_roi = 0.5 * (h_roi - 1)
                w_gauss = self._gaussian_weights(h_roi, w_roi, self.gauss_sigma_px, (cy_roi, cx_roi))

                # Apply validity as a hard mask
                w_combined = w_gauss * roi_valid.astype(np.float32)

                # Guard against all-invalid ROI
                w_sum = float(w_combined.sum())
                if w_sum > 1e-6:
                    # Weighted center x (in full-image pixels)
                    xs_roi = np.arange(w_roi, dtype=np.float32)[None, :].repeat(h_roi, axis=0)
                    u_px_weighted = (float((w_combined * xs_roi).sum()) / w_sum) + x1

                    # Weighted average depth (meters)
                    z_m_weighted = float((w_combined * roi_depth).sum() / w_sum)

                    # Publish these
                    u_px = u_px_weighted
                    z_m = z_m_weighted
                else:
                    # Fallback: median of valid depths; center = bbox center
                    valid_vals = roi_depth[roi_valid]
                    if valid_vals.size > 0:
                        z_m = float(np.median(valid_vals))
                        u_px = float(0.5 * (x1 + x2))

            # publish bbox either way
            bbox_msg = Int32MultiArray()
            bbox_msg.data = [x1, y1, x2, y2]
            self.pub_bbox.publish(bbox_msg)
        else:
            # publish empty bbox
            self.pub_bbox.publish(Int32MultiArray())

        if centroid is not None:
            cx, cy = centroid
            cv2.circle(overlay, (int(round(cx)), int(round(cy))), 5, (0, 0, 0), -1)
            cv2.circle(overlay, (int(round(cx)), int(round(cy))), 3, (255, 255, 255), -1)

        # Publish overlay image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_overlay.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish overlay: {e}')

        # Publish [u_px, z_m] if available
        if (u_px is not None) and (z_m is not None) and np.isfinite(z_m) and z_m > 0.0:
            out = Float32MultiArray()
            out.data = [float(u_px), float(z_m)]
            self.pub_center_depth.publish(out)

        # pacing handled by ROS timer

    # ---------- Shutdown ----------
    def destroy_node(self):
        try:
            if hasattr(self, 'release_cap') and self.cap is not None:
                self.release_cap()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DancerSegNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
