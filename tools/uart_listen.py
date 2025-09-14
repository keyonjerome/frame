import serial
import time
import os
import cv2
from datetime import datetime
from pathlib import Path
import sys
import threading

# Add the project root to the path so we can import from web
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(repo_root)

# Videos directory (consistent with app.py)
VIDEOS_DIR = os.path.join(repo_root, 'videos')
os.makedirs(VIDEOS_DIR, exist_ok=True)

# UART settings - must match gesture_sign.py
UART_PORT = "/dev/ttyUSB0"  # Update to match your actual port
UART_BAUD = 115200

# Try to import GPIO for Raspberry Pi, fall back to simulation mode if not available
try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)
    LIGHT_PIN = 18  # Choose your GPIO pin
    GPIO.setup(LIGHT_PIN, GPIO.OUT)
    gpio_available = True
    print("GPIO initialized successfully")
except ImportError:
    print("GPIO library not available. Running in simulation mode.")
    gpio_available = False

class VideoRecorder:
    def __init__(self, resolution=(640, 480), fps=20):
        self.recording = False
        self.video_path = None
        self.cap = None
        self.writer = None
        self.resolution = resolution
        self.fps = fps
        self.start_time = None
        self.recording_thread = None
        self.stop_event = threading.Event()
        
    def start(self):
        """Start actual video recording in a separate thread"""
        if self.recording:
            print('Already recording')
            return
            
        # Generate filename with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"video_{timestamp}.mp4"
        self.video_path = os.path.join(VIDEOS_DIR, filename)
        
        # Reset stop event
        self.stop_event.clear()
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self._record_thread)
        self.recording_thread.daemon = True
        self.recording_thread.start()
        
        self.recording = True
        print(f"Started recording to {self.video_path}")
    
    def _record_thread(self):
        """Thread function for continuous recording"""
        try:
            self.cap = cv2.VideoCapture(0)  # Use camera index 0
            if not self.cap.isOpened():
                print("Error: Could not open camera")
                return
                
            # Set resolution
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
            
            # Initialize video writer
            fourcc = cv2.VideoWriter_fourcc(*'avc1')  # H.264 codec
            self.writer = cv2.VideoWriter(
                self.video_path, 
                fourcc, 
                self.fps, 
                self.resolution
            )
            
            if not self.writer.isOpened():
                print(f"Error: Could not create video writer for {self.video_path}")
                self.cap.release()
                return
                
            self.start_time = time.time()
            
            # Recording loop
            while not self.stop_event.is_set():
                ret, frame = self.cap.read()
                if not ret:
                    print("Warning: Failed to capture frame")
                    time.sleep(0.1)
                    continue
                    
                # Add timestamp to frame
                elapsed = time.time() - self.start_time
                cv2.putText(
                    frame,
                    f"{elapsed:.1f}s",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2
                )
                self.writer.write(frame)
                
                # Small sleep to avoid maxing out CPU
                time.sleep(1.0/self.fps)
                
        except Exception as e:
            print(f"Error during recording: {e}")
        finally:
            self._cleanup_resources()
    
    def stop_and_save(self):
        """Stop recording and save the video file"""
        if not self.recording:
            print('No recording in progress')
            return
            
        print(f"Stopping recording and saving to {self.video_path}")
        
        # Signal recording thread to stop
        self.stop_event.set()
        
        # Wait for thread to finish
        if self.recording_thread and self.recording_thread.is_alive():
            self.recording_thread.join(timeout=5.0)
            
        self.recording = False
        
        # Verify the video was created
        if os.path.exists(self.video_path):
            # Get video info for the log
            size_mb = os.path.getsize(self.video_path) / (1024 * 1024)
            print(f"Video saved: {self.video_path}")
            print(f"Size: {size_mb:.1f} MB")
        else:
            print(f"Warning: Video file was not created at {self.video_path}")
            
    def _cleanup_resources(self):
        """Release opencv resources"""
        if self.writer:
            self.writer.release()
            self.writer = None
        if self.cap:
            self.cap.release() 
            self.cap = None

def control_light(state):
    """Turn the light on or off"""
    if gpio_available:
        GPIO.output(LIGHT_PIN, state)
    print(f"Light turned {'ON' if state else 'OFF'}")

def main():
    # Initialize the video recorder
    recorder = VideoRecorder()
    
    # Open the UART port
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        print(f"Opened UART on {UART_PORT} at {UART_BAUD} baud")
    except Exception as e:
        print(f"Error opening UART: {e}")
        print("Running in simulation mode. Press 'p' to simulate peace sign detection.")
        ser = None
        
    # State tracking
    system_active = False
    
    print("Listening for peace sign signals...")
    try:
        while True:
            # Read from UART if available, otherwise simulate
            command = None
            if ser and ser.in_waiting > 0:
                byte_read = ser.read(1)
                if byte_read == b'P':
                    command = 'P'
                    print("Received 'P' from UART")
            elif not ser:
                # Simulation mode - allow keyboard input for testing
                user_input = input("Press 'p' to simulate peace sign, or 'q' to quit: ")
                if user_input.lower() == 'p':
                    command = 'P'
                elif user_input.lower() == 'q':
                    break
                
            # Process command
            if command == 'P':
                system_active = not system_active
                print(f"Peace sign detected. System {'activated' if system_active else 'deactivated'}")
                
                if system_active:
                    # Start recording and turn on light
                    recorder.start()
                    control_light(True)
                    print("Movement started (placeholder for sending to second Pi)")
                else:
                    # Stop recording and turn off light
                    recorder.stop_and_save()
                    control_light(False)
                    print("Movement stopped (placeholder for sending to second Pi)")
            
            # Small sleep to avoid maxing out CPU when no serial port
            if not ser:
                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        # Clean up
        if ser:
            ser.close()
        if gpio_available:
            GPIO.cleanup()
        if recorder.recording:
            recorder.stop_and_save()

if __name__ == "__main__":
    main()
