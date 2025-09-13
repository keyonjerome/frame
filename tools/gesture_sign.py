import cv2
import mediapipe as mp
import serial
import time

UART_PORT = "/dev/ttyUSB0"  # wtf is the port number?
UART_BAUD = 115200

class PeaceSignDetector:
    def __init__(self, min_detection_confidence=0.5):
        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=4,
            min_detection_confidence=min_detection_confidence,
        )
        self.drawing = mp.solutions.drawing_utils

    def is_peace_sign(self, landmarks) -> bool:
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        up = []
        for tip, pip in zip(tips, pips):
            up.append(landmarks[tip].y < landmarks[pip].y)
        return up[0] and up[1] and not up[2] and not up[3]

    def detect(self, frame) -> bool:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        found = False
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.drawing.draw_landmarks(
                    frame, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS
                )
                if self.is_peace_sign(hand_landmarks.landmark):
                    found = True
        return found

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    detector = PeaceSignDetector()
    print("Starting peace sign detection. Press ESC to quit.")

    # Open UART serial port
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"Opened UART on {UART_PORT} at {UART_BAUD} baud.")
    except Exception as e:
        print(f"Could not open UART: {e}")
        ser = None

    detected = False
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera error.")
            break
        found = detector.detect(frame)
        cv2.putText(
            frame,
            "Peace sign: {}".format("YES" if found else "No"),
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 0) if found else (0, 0, 255),
            2,
        )
        cv2.imshow("Gesture Detection", frame)
        if found and not detected:
            print("Peace sign detected!")
            detected = True
            if ser:
                try:
                    ser.write(b"START\n")
                    print("Sent UART trigger: START")
                except Exception as e:
                    print(f"UART send failed: {e}")
        if not found and detected:
            detected = False
        key = cv2.waitKey(1)
        if key == 27:
            break
    cap.release()
    cv2.destroyAllWindows()
    if ser:
        ser.close()

if __name__ == "__main__":
    main()