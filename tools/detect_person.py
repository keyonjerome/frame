
import pyrealsense2 as rs
import numpy as np
import cv2

# Load YOLOv3 model
net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]

# Load COCO class labels
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Check if a device is connected
context = rs.context()
if not context.query_devices():
    raise RuntimeError("No Intel RealSense device connected")

# Get the first connected device
device = context.query_devices()[0]
sensor = device.query_sensors()[0]

# Configure the streams
config.enable_stream(rs.stream.infrared, 640, 480, rs.format.y8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        infrared_frame = frames.get_infrared_frame()
        if not depth_frame or not infrared_frame:
            continue

        # Convert images to numpy arrays
        infrared_image = np.asanyarray(infrared_frame.get_data())

        # Convert single-channel image to 3-channel image
        infrared_image_3ch = cv2.cvtColor(infrared_image, cv2.COLOR_GRAY2BGR)

        # Detecting objects
        blob = cv2.dnn.blobFromImage(infrared_image_3ch, 0.00392, (416, 416), (0, 0, 0), True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        # Information to show on the screen
        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and class_id == classes.index("person"):
                    # Object detected
                    center_x = int(detection[0] * 640)
                    center_y = int(detection[1] * 480)
                    w = int(detection[2] * 640)
                    h = int(detection[3] * 480)

                    # Rectangle coordinates
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        for i in range(len(boxes)):
            if i in indexes:
                x, y, w, h = boxes[i]
                label = str(classes[class_ids[i]])
                confidence = confidences[i]
                color = (0, 255, 0)
                # Calculate distance
                distance = depth_frame.get_distance(center_x, center_y)

                # Draw rectangle and add text
                cv2.rectangle(infrared_image, (x, y), (x + w, y + h), color, 2)
                cv2.putText(infrared_image, f"{label} {confidence:.2f} Dist: {distance:.2f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                cv2.putText(infrared_image, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Show the image
        cv2.imshow("Infrared Image", infrared_image)
        key = cv2.waitKey(1)
        if key == 27:  # Press 'ESC' to exit
            break
finally:
    # Stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()