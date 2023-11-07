## Autonomous Surface Vehicle in Search and Rescue Process of Marine Casualty using Computer Vision Based Victims Detection
#### Code Implementation used in my research [refer to my IEEE paper](https://ieeexplore.ieee.org/document/10037319)

This is My Final Project (research) for my Bachelor Degree.
All code implemented using C++, except the Computer Vision part is using python.

This Project is consist of some ROS node:
- Control System
  - Mission Planning and Track Target using Computer Vision
  - Obstacle Avoidance using Ultrasonic Sensor
  - Return To Home
- Computer Vision
  - Using YOLOv4 to detect human on the water (Marine Casualty Victims)
  - Model converted to tensorRT so can running on mobile laptop GPU
- Localization
  - Global Localization using GPS
  - IMU, Compass, and Gyro for orientation
  - Combined data implemented to TF
- Ground Control and Monitoring
  - Implemented using QT and ROS RViZ
- UDP Communcation to Microcontroller (STM32F4)
  - Drive Motor
  - Drive LED Indicator
  - Receive any radio communication from RC Controller
  - Emergency Stop
 
