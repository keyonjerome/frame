#!/usr/bin/env python3
"""
MobileNet-SSD (Caffe) person detection on Intel RealSense D421 IR + Depth.
- Tiny/fast CNN via OpenCV DNN
- Uses IR (grayscale) -> replicate to 3-ch
- Depth aligned to IR; median depth computed per box
- Highlights the closest person

Keys: q / ESC to quit
"""

import time, math
import numpy as np
import cv2
import pyrealsense2 as rs

# ---------- Model files (put these beside the script) ----------
CAFFE_PROTOTXT = "MobileNetSSD_deploy.prototxt.txt"
CAFFE_MODEL    = "MobileNetSSD_deploy.caffemodel"

# VOC labels used by this classic model (person = 15)
VOC_CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat",
    "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
    "dog", "horse", "motorbike", "person", "pottedplant",
    "sheep", "sofa", "train", "tvmonitor"
]
PERSON_ID = 15

# ---------- Inference params ----------
INP_SIZE   = (300, 300)          # MobileNet-SSD default
SCALE      = 0.007843            # 1/127.5
MEAN       = (127.5, 127.5, 127.5)
CONF_TH    = 0.40
NMS_TH     = 0.45                # we'll apply a simple NMS

# ---------- Depth / IR params ----------
W, H, FPS  = 640, 480, 30
DEPTH_MIN, DEPTH_MAX = 0.5, 4.0   # working range for gating
ROI_FRACTION = 0.4               # center box fraction for median depth

WIN = "MobileNet-SSD (IR) + Depth — closest person"

def letterbox_to(img, size):
    """Resize with unchanged aspect ratio and padding."""
    h, w = img.shape[:2]
    tw, th = size
    r = min(tw / w, th / h)
    nw, nh = int(round(w * r)), int(round(h * r))
    left = (tw - nw) // 2
    top  = (th - nh) // 2
    out = np.full((th, tw, 3), 114, dtype=img.dtype)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    out[top:top+nh, left:left+nw] = resized
    return out, r, left, top, nw, nh

def nms(boxes, scores, th=0.45):
    """Very small NMS for [x,y,w,h] boxes in absolute pixels."""
    if len(boxes) == 0:
        return []
    boxes = np.array(boxes, dtype=np.float32)
    scores = np.array(scores, dtype=np.float32)
    x1 = boxes[:,0]; y1 = boxes[:,1]; x2 = x1 + boxes[:,2]; y2 = y1 + boxes[:,3]
    areas = (x2 - x1 + 1) * (y2 - y1 + 1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        w = np.maximum(0.0, xx2 - xx1 + 1)
        h = np.maximum(0.0, yy2 - yy1 + 1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        inds = np.where(iou <= th)[0]
        order = order[inds + 1]
    return keep

def main():
    # Load CNN
    net = cv2.dnn.readNetFromCaffe(CAFFE_PROTOTXT, CAFFE_MODEL)

    # RealSense setup
    ctx = rs.context()
    if not list(ctx.query_devices()):
        raise RuntimeError("No Intel RealSense device connected.")
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.infrared, W, H, rs.format.y8,  FPS)
    cfg.enable_stream(rs.stream.depth,    W, H, rs.format.z16, FPS)
    profile = pipe.start(cfg)

    # Align depth → IR
    dev = profile.get_device()
    depth_sensor = dev.first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()
    align_to_ir = rs.align(rs.stream.infrared)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    t0, n = time.time(), 0

    try:
        while True:
            frames = pipe.wait_for_frames()
            frames = align_to_ir.process(frames)
            d = frames.get_depth_frame()
            irf = frames.get_infrared_frame()
            if not d or not irf:
                continue

            # Get frames
            ir = np.asanyarray(irf.get_data())            # (H,W) uint8
            ir_bgr = cv2.cvtColor(ir, cv2.COLOR_GRAY2BGR) # (H,W,3)
            depth_raw = np.asanyarray(d.get_data())       # uint16
            depth_m = depth_raw.astype(np.float32) * depth_scale

            # Letterbox to model size
            lb, ratio, left, top, nw, nh = letterbox_to(ir_bgr, (INP_SIZE[0], INP_SIZE[1]))

            # DNN
            blob = cv2.dnn.blobFromImage(lb, scalefactor=SCALE, size=INP_SIZE,
                                         mean=MEAN, swapRB=False, crop=False)
            net.setInput(blob)
            detections = net.forward()  # shape: [1,1,N,7] -> [batch, cls, num, (batch_id, class_id, conf, x1,y1,x2,y2)]

            # Collect person boxes
            boxes, scores = [], []
            Hh, Ww = ir.shape[:2]
            for i in range(detections.shape[2]):
                _, class_id, conf, x1, y1, x2, y2 = detections[0, 0, i, :]
                if conf < CONF_TH or int(class_id) != PERSON_ID:
                    continue
                # coords are in [0,1] of the letterboxed input
                # map to letterboxed pixels
                X1 = int(x1 * INP_SIZE[0]); Y1 = int(y1 * INP_SIZE[1])
                X2 = int(x2 * INP_SIZE[0]); Y2 = int(y2 * INP_SIZE[1])
                # remove padding + scale back
                px1 = int((X1 - left) / max(ratio, 1e-9))
                py1 = int((Y1 - top)  / max(ratio, 1e-9))
                px2 = int((X2 - left) / max(ratio, 1e-9))
                py2 = int((Y2 - top)  / max(ratio, 1e-9))
                # clamp
                px1 = max(0, min(px1, Ww-1)); py1 = max(0, min(py1, Hh-1))
                px2 = max(0, min(px2, Ww-1)); py2 = max(0, min(py2, Hh-1))
                w = max(1, px2 - px1); h = max(1, py2 - py1)
                boxes.append([px1, py1, w, h])
                scores.append(float(conf))

            keep = nms(boxes, scores, th=NMS_TH)
            kept = [boxes[i] for i in keep]
            kept_scores = [scores[i] for i in keep]

            # For each person, compute median depth of center ROI
            candidates = []
            for (x, y, w, h), sc in zip(kept, kept_scores):
                # center ROI
                fx = ROI_FRACTION
                rx = int(x + (1 - fx) * 0.5 * w)
                ry = int(y + (1 - fx) * 0.5 * h)
                rw = max(1, int(w * fx))
                rh = max(1, int(h * fx))
                x1 = max(0, rx); y1 = max(0, ry)
                x2 = min(Ww, rx + rw); y2 = min(Hh, ry + rh)

                roi = depth_m[y1:y2, x1:x2]
                vals = roi[(roi > 0) & np.isfinite(roi)]
                if vals.size == 0:
                    continue
                z = float(np.median(vals))
                if z < DEPTH_MIN or z > DEPTH_MAX:
                    continue
                candidates.append((x, y, w, h, z, sc))

            # choose closest
            chosen = min(candidates, key=lambda t: t[4]) if candidates else None

            # Draw
            vis = ir_bgr.copy()
            for (x, y, w, h, z, sc) in candidates:
                col = (0, 200, 255)
                if chosen and (x, y, w, h) == chosen[:4]:
                    col = (0, 255, 0)
                cv2.rectangle(vis, (x, y), (x+w, y+h), col, 2)
                cv2.putText(vis, f"person {sc:.2f} {z:.2f}m", (x, max(0, y-6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

            # HUD + FPS
            n += 1
            if n % 30 == 0:
                fps = n / (time.time() - t0 + 1e-9)
                cv2.setWindowTitle(WIN, f"{WIN}  |  ~{fps:.1f} FPS")
            if chosen:
                x, y, w, h, z, sc = chosen
                cv2.putText(vis, f"CLOSEST: {z:.2f} m", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (50, 255, 50), 2)
            else:
                cv2.putText(vis, "No person in range", (10, 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)

            cv2.imshow(WIN, vis)
            if (cv2.waitKey(1) & 0xFF) in (27, ord('q')):
                break

    finally:
        pipe.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
