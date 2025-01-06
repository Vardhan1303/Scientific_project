# Platooning Autonomous Following Robot

This repository contains the code and documentation for the **Platooning Autonomous Following Robot** project, developed as part of a scientific project at Ravensburg-Weingarten University. 

## Project Overview
The objective of this project is to create a cost-effective, autonomous robot capable of following a lead vehicle in a platoon formation. Using computer vision and ArUco markers, the robot tracks the lead vehicle and maintains a safe distance without human intervention.

## Key Features
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

## Repository Structure
- `src/`: Source code for robot control, marker detection, and motor navigation.
- `docs/`: Project report and detailed documentation.
- `media/`: Images and videos showcasing the project in action.
- `tests/`: Scripts and data for testing individual components.

## Setup Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/Scientific_project.git
2. Follow the setup guide in the `docs/` folder to configure your Raspberry Pi and install necessary dependencies.

## Contributors
- **Vardhan Vinodbhai Mistry**  & **Jai Doshi**

## Guided by
- **Prof. Dr. Stefan Elser** 
