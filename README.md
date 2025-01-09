# Platooning Autonomous Following Robot

![Platooning Autonomous Following Robot](/media/platooning.jpeg)

**Authors:** Vardhan Mistry, Jai Doshi  
**Supervisor:** Prof. Dr. Stefan Elser

---

This repository contains the code and documentation for the **Platooning Autonomous Following Robot** project, developed as part of a scientific project at Ravensburg-Weingarten University. 

## 🚀 Project Overview
The objective of this project is to create a cost-effective, autonomous robot capable of following a lead vehicle in a platoon formation. Using computer vision and ArUco markers, the robot tracks the lead vehicle and maintains a safe distance without human intervention.

## 🌟 Key Features
- **Technologies Used:**
  - Computer Vision (OpenCV) for object detection
  - Raspberry Pi 4 as the control unit
  - ArUco markers for tracking
  - DC motors controlled by GPIO for movement
  - Kalman Filter for noise reduction and movement accuracy

- **Core Objectives:**
  - Develop a low-cost solution without expensive LiDAR sensors.
  - Enable accurate detection and tracking using ArUco markers.
  - Ensure smooth and reliable robot movement in real-time.

## 📂 Repository Structure
```
Platooning_Autonomous_Following_Robot/
├── src/
│   ├── aruco_marker_detection/        # Code for detecting and processing ArUco markers
│   ├── arucoMarker/                   # Code to generate ArUco marker
│   ├── cameraCalibration/             # Scripts for calibrating the camera
│   ├── distance_calculation/          # Code for distance estimation
│   └── main.py                        # Main algorithm that integrates all components
├── docs/                              # Project report and detailed documentation
├── media/                             # Images and videos showcasing the project
│   ├── arucoMarker/                   # Image of ArUco markers used in project
│   ├── arucoMarkerDetection/          # Media of marker detection
│   ├── cameraCalibration/             # Camera calibration process media
│   ├── depthEstimation/               # Visual content related to depth estimation
│   ├── platooning                     # Photo of the two robots in platooning setup
│   └── robot                          # Image of the robot used for experiments
├── tests/                             # Testing scripts for individual components
│   ├── 00_test                        # Tests robot motors
│   ├── 01_camera                      # Tests camera functionality
│   ├── 02_lidar                       # Tests LiDAR sensor (if applicable)
│   ├── 03_drive_circle                # Drives robot in circular movement
│   ├── 04_all                         # Combines and tests all components
│   └── 05_manual                      # Manual control of the robot with keyboard
└── LICENSE                            # License file for project usage
```

## 🛠️ Setup Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/Vardhan1303/Scientific_project.git
   ```
2. Follow the setup guide in the `docs/` folder to configure your Raspberry Pi and install necessary dependencies.

## 📜 License
This project is protected under the **License Agreement**. No part of this project may be reproduced, distributed, or transmitted in any form or by any means without explicit permission from the authors. For permissions, please contact the authors.

## 🤝 Contributors
- **Vardhan Vinodbhai Mistry**  
- **Jai Doshi**

## 🧑‍🏫 Guided by
- **Prof. Dr. Stefan Elser**
