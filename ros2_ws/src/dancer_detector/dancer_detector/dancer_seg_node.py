#!/usr/bin/env python3
"""
ROS 2 Humble node: depth-only segmentation → bbox + center-depth
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
from rclpy.duration import Duration

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from example_interfaces.msg import Float32MultiArray, Int32MultiArray
from std_srvs.srv import Trigger

# Your helpers (keep your existing modules in the package or repo)
from dancer_detector.common.uvc_depth_utils import open_uvc_depth, read_depth_meters, colorize_depth
from dancer_detector.common.segmentation_utils import morph_close, remove_small

# add at top
from rclpy.parameter import Parameter


class DancerSegNode(Node):
    def __init__(self):
        super().__init__('dancer_seg')

        # -------- Parameters (declare → get) --------
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        self.declare_parameter('depth_min_m', 0.01)
        self.declare_parameter('depth_max_m', 20.0)
        self.declare_parameter('red_min_norm', 180)          # 0..255
        self.declare_parameter('min_blob_area_frac', 0.001)  # fraction of image
        self.declare_parameter('morph_kernel_w', 7)
        self.declare_parameter('morph_kernel_h', 7)
        self.declare_parameter('alpha_mask', 0.35)

        # add param
        self.declare_parameter('backend', 'realsense')   # 'realsense' or 'uvc'
        self.backend = str(self.get_parameter('backend').value)
        
        # publish rate (Hz) for processing loop
        self.declare_parameter('proc_hz', 15.0)

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
                if self.backend == 'realsense':
                    from dancer_detector.common.rs_depth_utils import RSDepth
                    self.cap = RSDepth(self.size[0], self.size[1], self.fps)
                    self.read_depth = self.cap.read_depth_meters
                    self.release_cap = self.cap.release
                else:
                    from dancer_detector.common.uvc_depth_utils import open_uvc_depth, read_depth_meters
                    self.cap = open_uvc_depth(self.camera_id, self.size, self.fps)
                    self.read_depth = lambda: read_depth_meters(self.cap)
                    self.release_cap = lambda: self.cap.release()
            except Exception as e:
                resp.success = False
                resp.message = f'Failed to open depth backend ({self.backend}): {e}'
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

    # ---------- Main loop ----------
    def step(self):
        if not self.active:
            return
        if self.cap is None:
            return

        t0 = time.monotonic()

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

        # 0..255 normalization for thresholding consistent with JET colorize
        d = depth_m.copy()
        d[d < self.depth_min] = self.depth_min
        d[d > self.depth_max] = self.depth_max
        denom = max(1e-6, (self.depth_max - self.depth_min))
        norm = ((d - self.depth_min) / denom * 255.0).astype(np.uint8)
        norm[depth_m <= 0] = 0

        # Threshold for “darkest red” regions (farthest in JET)
        mask = (norm >= self.red_min_norm).astype(np.uint8) * 255
        mask = morph_close(mask, self.kernel_wh, iterations=1)

        # Remove small blobs
        min_area = max(1, int(self.min_blob_area_frac * H * W))
        mask = remove_small(mask, min_area)

        # Connected components — select component with highest median norm
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
                score = med_norm
                if score > best_score:
                    best_score = score
                    x, y, w, h, _ = stats[i]
                    bbox = (int(x), int(y), int(x + w), int(y + h))
                    cx, cy = centroids[i]
                    centroid = (float(cx), float(cy))

        # Visualization
        depth_vis = colorize_depth(depth_m, vmin=self.depth_min, vmax=self.depth_max)
        overlay = depth_vis.copy()

        if np.any(mask):
            colored_mask = np.zeros_like(overlay)
            colored_mask[:, :] = (0, 0, 255)  # red overlay
            alpha = (mask > 0).astype(np.float32) * self.alpha_mask
            overlay = (overlay.astype(np.float32) * (1 - alpha)[..., None] +
                       colored_mask.astype(np.float32) * alpha[..., None]).astype(np.uint8)

        # Draw bbox/centroid and compute median depth inside bbox
        u_px = None
        z_m = None
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), 3)
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 255), 2)

            roi = depth_m[y1:y2, x1:x2]
            valid = roi[(roi > 0.0) & np.isfinite(roi)]
            if valid.size > 0:
                z_m = float(np.median(valid))
            # center x in pixels for bearing; (x1+x2)/2 is fine
            u_px = float(0.5 * (x1 + x2))

            # publish bbox
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
        img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        img_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_overlay.publish(img_msg)

        # Publish [u_px, z_m] if available
        if (u_px is not None) and (z_m is not None) and np.isfinite(z_m) and z_m > 0.0:
            out = Float32MultiArray()
            out.data = [float(u_px), float(z_m)]
            self.pub_center_depth.publish(out)

        # pacing handled by ROS timer period

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
