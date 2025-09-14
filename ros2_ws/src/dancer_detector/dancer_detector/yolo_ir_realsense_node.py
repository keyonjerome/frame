#!/usr/bin/env python3
import os
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

import numpy as np
import pyrealsense2 as rs


class YoloIRRealSenseNode(Node):
    def __init__(self):
        super().__init__('yolo_ir_realsense')

        # --- Parameters (can be overridden via ROS params) ---
        self.declare_parameter('weights_path', os.path.expanduser('~/models/yolov3/yolov3.weights'))
        self.declare_parameter('cfg_path',     os.path.expanduser('~/models/yolov3/yolov3.cfg'))
        self.declare_parameter('names_path',   os.path.expanduser('~/models/yolov3/coco.names'))
        self.declare_parameter('show_window',  True)

        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value
        cfg_path     = self.get_parameter('cfg_path').get_parameter_value().string_value
        names_path   = self.get_parameter('names_path').get_parameter_value().string_value
        self.show_win = self.get_parameter('show_window').get_parameter_value().bool_value

        # --- Load YOLOv3 model (unchanged from your logic) ---
        self.net = cv2.dnn.readNetFromDarknet(cfg_path, weights_path)
        layer_names = self.net.getLayerNames()
        self.output_layers = [layer_names[i - 1] for i in self.net.getUnconnectedOutLayers().flatten()]

        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
        # --- Load COCO class labels (unchanged) ---
        with open(names_path, 'r') as f:
            self.classes = [line.strip() for line in f.readlines()]
        try:
            self.person_class_index = self.classes.index("person")
        except ValueError:
            raise RuntimeError('"person" not found in coco.names')

        # --- Set up RealSense (unchanged except moved into a class) ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        context = rs.context()
        if not context.query_devices():
            raise RuntimeError("No Intel RealSense device connected")

        # Configure streams: infrared + depth (same as your script)
        self.config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)
        self.config.enable_stream(rs.stream.depth,    640, 480, rs.format.z16, 30)

        # Start streaming
        self.pipeline.start(self.config)

        # Publisher for centroid & depth (pixels in x,y; depth meters in z)
        self.pub_centroid = self.create_publisher(PointStamped, 'person_centroid', 10)

        # Main timer loop ~30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.loop_once)

        self.get_logger().info("YOLOv3 + RealSense IR node started. Publishing /person_centroid (PointStamped)")

    def loop_once(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=200)
        except Exception:
            return

        depth_frame = frames.get_depth_frame()
        infrared_frame = frames.get_infrared_frame()
        if not depth_frame or not infrared_frame:
            return

        # Convert images to numpy arrays
        infrared_image = np.asanyarray(infrared_frame.get_data())

        # Convert single-channel to 3-channel for YOLO
        infrared_bgr = cv2.cvtColor(infrared_image, cv2.COLOR_GRAY2BGR)

        # Detecting objects (unchanged core)
        blob = cv2.dnn.blobFromImage(infrared_bgr, 0.00392, (320, 320), (0,0,0), True, crop=False)
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids = []
        confidences = []
        boxes = []
        centers = []

        W, H = 640, 480
        img_cx, img_cy = W // 2, H // 2

        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = int(np.argmax(scores))
                confidence = float(scores[class_id])
                if confidence > 0.5 and class_id == self.person_class_index:
                    center_x = int(detection[0] * W)
                    center_y = int(detection[1] * H)
                    w = int(detection[2] * W)
                    h = int(detection[3] * H)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(confidence)
                    class_ids.append(class_id)
                    centers.append((center_x, center_y))

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        chosen_center = None
        chosen_box = None
        chosen_depth = None

        if len(idxs) > 0:
            # --- Choose the "largest-centermost" person ---
            # Strategy: minimize distance to image center; break ties by larger area.
            best = None
            for i in idxs.flatten():
                x, y, w, h = boxes[i]
                cx, cy = centers[i]
                d_center = (cx - img_cx) ** 2 + (cy - img_cy) ** 2  # no sqrt needed
                area = w * h
                key = (d_center, -area)  # smaller distance is better; larger area is better
                if (best is None) or (key < best[0]):
                    chosen_center = (cx, cy)
                    chosen_box = (x, y, w, h)
                    best = (key, i)
            if chosen_center is not None:
                cx, cy = chosen_center
                chosen_depth = float(depth_frame.get_distance(int(cx), int(cy)))

        # Draw and annotate (same as your display logic)
        if len(idxs) > 0:
            for j, i in enumerate(idxs.flatten()):
                x, y, w, h = boxes[i]
                cx, cy = centers[i]
                label = str(self.classes[class_ids[i]])
                conf = confidences[i]
                color = (0, 255, 0)
                dist_here = float(depth_frame.get_distance(int(cx), int(cy)))

                cv2.rectangle(infrared_image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(
                    infrared_image,
                    f"{label} {conf:.2f} Dist: {dist_here:.2f}m",
                    (x, max(0, y - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

                # Highlight chosen target
                if chosen_box is not None and (x, y, w, h) == chosen_box:
                    cv2.circle(infrared_image, (int(cx), int(cy)), 5, (255, 255, 255), -1)
                    cv2.putText(
                        infrared_image,
                        "TARGET",
                        (x, y + h + 16),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2
                    )

        # Publish centroid + depth for the chosen target
        if chosen_center is not None and chosen_depth is not None and np.isfinite(chosen_depth):
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "realsense_infrared"  # logical frame; adjust if you have TF
            msg.point.x = float(chosen_center[0])      # pixel x
            msg.point.y = float(chosen_center[1])      # pixel y
            msg.point.z = float(chosen_depth)          # depth in meters
            self.pub_centroid.publish(msg)

        if self.show_win:
            cv2.imshow("Infrared Image", infrared_image)
            key = cv2.waitKey(1)
            if key == 27:  # ESC
                rclpy.shutdown()

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloIRRealSenseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
