# Frame - The Breakin' Recording Robot
Hack The North 2025
Keyon Jerome, Inzaghi, Rishabh Sharma, Arsal Khan

**NOTE**: SEE ros2_ws folder for updated launch instructions and system specifics. This project is no longer using Raspberry Pi hardware.

**Objective**: Build a single-target dance recording robot with an iRobot Create 3 Educational Robot base, and an Intel Realsense D421 Depth camera.

## Usage Flow ##
1. Dancer holds up peace-sign gesture to the Raspberry Pi GS Camera. Running program recognizes hand gesture and starts up the dancer-following control loop, and starts recording.
2. Dancer drops their set (dances), and the robot maintains a specified distance during their set, using the depth camera to estimate distance from the target, and correcting itself to always maintain a straight-line view of the target at a specified distance.
3. At the end of their set, the dancer holds up the same peace sign, signifying the end of their set. The robot stops recording and halts the following sequence.
4. The dancer or any user scans the QR code sitting on top of the robot, which auto-joins them to the robot (Raspberry Pi)'s local 5GHz network, which opens a default webpage where they can download recorded videos from the robot.

## Project Material ## 
- 2x Raspberry Pis (1x 3B+, 1x 4B)
- 1x Raspberry Pi Global Shutter Camera w/ Wide Angle Lens
- 1x Create 3 iRobot Educational Robot
- 1x Intel Realsense D421 Depth Camera
- 1-2x LED System Indicators
- 1x K&F 20000lux/1m light

## System Specifics

### Raspberry Pi 3B+:
The Raspberry Pi 3B+ is directly connected to the Create 3 Robot, and is where the robot's main control loop with the depth camera runs. Any other computation is offloaded to the 4B+ for CPU constraint purposes. The two devices communicate directly over UART. The Raspberry Pi 3B+ is running Ubuntu 22.04, with ROS2 Humble, in order to communicate with the Create 3. The 3B+ does not need to send messages to the 4B; only listen to them, in order to start/stop the dancer detection algorithm.
### Raspberry Pi 4B:
The Raspberry Pi 4B concurrently runs two programs:
		- Peace sign gesture detection; when a peace sign is detected, it relays a message over UART to the 3b+ in order to start/stop the dancer-following algorithm.
		- Local-hosting a web page over a 5GHz hotspot that allows for recorded videos to be downloaded. Set up a QR code that allows the network to be auto-joined, and make the default page for the network the video-download page.
		The Raspberry Pi 4B will run QNX.
		
		
## Dancer Detection
Dancer detection has to be done in near real-time in order to feed into our control loop; for that reason, classical computer vision techniques are preferred, in combination with our depth information from the Intel Realsense camera:
1. Run a 1-2cm RANSAC to exclude the "floor" pixels from the detection grame. Mask out the ground plane.
2. Run a depth-based segmentation on the remaining pixels:
  a) clip to plausible person range (e.g: 0.1 - 5m)
  b) Run a morphologically close algoirthm and remove small blobs
  c) Use a connected-components/euclidean clustering in 3D algorithm
  d) Pick the largest cluster with a human-like size/volume; pick the median Z of that cluster as the depth to the target, and find its centroid.
 
 The output from this algorithm needs to be constantly available, published over ROS2 to the remainder of the robot control loop.
 	
## UART Reader
Need to constantly scan UART to check if a peace-sign is being detected; start/stop the robot control loop service. 	
 	
## Robot Control Loop:
1. Subscribe to the message coming out of the dancer-detection algorithm. Use a simply-tuned PID controller for angle and distance error;
- We only care about distance and yaw. 


First Night Tasks:
Keyon: Create3 Robot ROS2 Humble setup. Get it listening to// velocity commands and set up the scaffolding for the other 2-3 nodes.
Rishabh: Peace-sign gesture detection. Use a normal webcam, and assume that your code will be ported to a Raspberry Pi 4B on QNX. The prog gram should be written such that when it sees a peace sign, it starts recording (actually saving the video frames), and stops recording when it sees a new one, or at a time limit of ~45 seconds. 
Arsal/Inz (feel free to decide who takes which):
- Create a website that reads from the local media of the Raspberry Pi 4B and allows a user to scroll through and download their dance video.
- Install QNX on the Raspberry Pi 4B.
- Write a simple Python program that sends a UART message over the wire.
		

		

