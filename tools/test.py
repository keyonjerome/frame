#!/usr/bin/env python3
# test.py - Simple camera recording test

import cv2
import time
import os

# Create videos directory if it doesn't exist
os.makedirs("videos", exist_ok=True)

def test_camera():
    print("Starting 5-second camera test...")
    
    # Open camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: Could not open camera!")
        return
    
    # Set up video writer
    filename = "videos/test_video.mp4"
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Simple codec that works everywhere
    writer = cv2.VideoWriter(filename, fourcc, 20, (640, 480))
    
    # Record for 5 seconds
    start_time = time.time()
    while (time.time() - start_time) < 5:
        ret, frame = cap.read()
        if ret:
            writer.write(frame)
            cv2.imshow("Recording...", frame)
            cv2.waitKey(1)
    
    # Clean up
    writer.release()
    cap.release()
    cv2.destroyAllWindows()
    
    print(f"Video saved to {filename}")

if __name__ == "__main__":
    test_camera()