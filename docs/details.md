# Platooning Autonomous Following Robot

## Project Overview
The **Platooning Autonomous Following Robot** is designed to autonomously follow a lead vehicle, maintaining a safe distance using cost-effective hardware and advanced computer vision techniques. The project eliminates the need for expensive LiDAR systems by leveraging ArUco markers for tracking and navigation. Developed using Raspberry Pi 4 and OpenCV, the robot combines precise control algorithms with real-time camera processing to achieve reliable and scalable autonomous navigation.

---

## Objectives
- **Autonomous Following:** Enable the follower robot to track and follow a lead vehicle autonomously.
- **Cost-Effective Design:** Exclude expensive components like LiDAR to enhance scalability.
- **Safe and Reliable Operation:** Maintain consistent performance across diverse conditions.
- **Technology Integration:** Employ advanced computer vision, robotics, and control systems.

---

## Key Technologies and Components
- **ArUco Markers:** Used for visual positioning and tracking.
- **Raspberry Pi 4:** Acts as the main processing unit.
- **Computer Vision (OpenCV):** Handles marker detection and pose estimation.
- **DC Motors and Motor Controller:** Facilitate movement and precise navigation.
- **Kalman Filter:** Reduces noise for improved movement accuracy.

---

## Implementation Details

### Hardware Setup
The robot integrates the following components:
- **Raspberry Pi 4:** Processes data from sensors and executes control algorithms.
- **Camera Module:** Captures real-time visuals for marker detection.
- **DC Motors and Motor Shield:** Provides movement capabilities.
- **Power Sources:** A power bank for Raspberry Pi and a dedicated battery for motors.

### Software Environment
- **Operating System:** Raspbian GNU/Linux 11 (Bullseye).
- **Libraries:** OpenCV, GPIO Zero, and Python 3.
- **Algorithms:**
  - Camera calibration to correct lens distortions.
  - Marker detection and pose estimation using OpenCV.
  - Control algorithms for autonomous navigation.

---

## Key Features

### Camera Calibration
Uses a checkerboard pattern to estimate intrinsic and extrinsic camera parameters, ensuring precise image correction.

#### Calibration Setup
- **Checkerboard Size:** 100 mm by 100 mm.
- **Square Size:** 10 mm by 10 mm.
- **Number of Inner Corners:** 9 x 9.

#### Image Acquisition
Images of the checkerboard were captured from various angles to enhance calibration robustness.

**Sample Images:**

- Distorted Images:
<p float="left">
  <img src="../media/cameraCalibration/captured_images/images1.jpg" width="20%" />
  <img src="../media/cameraCalibration/captured_images/images2.jpg" width="20%" />
  <img src="../media/cameraCalibration/captured_images/images3.jpg" width="20%" />
  <img src="../media/cameraCalibration/captured_images/images4.jpg" width="20%" />
</p>

- Undistorted Images:
<p float="left">
  <img src="../media/cameraCalibration/undistorted_images/1.png" width="20%" />
  <img src="../media/cameraCalibration/undistorted_images/2.png" width="20%" />
  <img src="../media/cameraCalibration/undistorted_images/3.png" width="20%" />
  <img src="../media/cameraCalibration/undistorted_images/4.png" width="20%" />
</p>

#### Calibration Parameters
- **Intrinsic Matrix (K)** and **Extrinsic Parameters (R, T)**.
- **Distortion Coefficients:** Radial (k1, k2, k3) and Tangential (p1, p2).

---

### Marker Detection
Employs OpenCV’s ArUco library to identify and track markers.

#### Generating and Detecting ArUco Markers
- **Using Python:** Generate markers using OpenCV functions.
- **Online Tools:** Use tools like the University of Oxford’s ArUco Generator.

**ArUco Marker Example:**

![ArUco Marker](../media/arucoMarker/6x6_1000-13.png)

#### Pose Estimation
Uses `cv2.solvePnP()` to estimate the position and orientation of detected markers.

**Sample Detection Images:**
<p float="left">
  <img src="../media/arucoMarkerDetection/frame1.png" width="33%" />
  <img src="../media/arucoMarkerDetection/frame2.png" width="33%" />
  <img src="../media/arucoMarkerDetection/frame3.png" width="33%" />
</p>

---

### Distance Estimation
Calculates marker distance using pose estimation.

#### Multi-Marker Detection
Three markers ensure continuous detection during turns.

**Distance Calculation Samples:**
<p float="left">
  <img src="../media/depthEstimation/three_aruco/one_ArUco Marker Distance Measurement1.png" width="33%" />
  <img src="../media/depthEstimation/three_aruco/two_ArUco Marker Distance Measurement2.png" width="33%" />
  <img src="../media/depthEstimation/three_aruco/three_ArUco Marker Distance Measurement3.png" width="33%" />
</p>

#### Evaluation
- **Error Metrics:** MAE, RMSE, and error percentage.
- **Graphical Analysis:**
<img src="../media/depthEstimation/distance_comparison_plot.png" width="600" />

---

### Movement Control
Adjusts motor speeds dynamically to maintain alignment and distance.

#### Motor Control Functionality
Implements Kalman Filter for refined measurements. Handles marker loss by initiating controlled rotations based on the last known position.

---

## Results and Evaluation
- **Testing Scenarios:** Conducted under diverse conditions.
- **Accuracy:** Achieved an average error of less than 2 cm in distance estimation.
- **Reliability:** Consistent performance across different lighting and surface conditions.
- **Testing Video:** [GitHub Link](https://github.com/user-attachments/assets/9fe479ac-1420-4cd1-aefd-35c88cfe8cb4)

---

## Future Work
- Enhance detection algorithms to address marker occlusion.
- Improve scalability for multi-vehicle platooning.
- Incorporate additional sensors for complex navigation tasks.

---
## References
1. A. Smith, B. Johnson, and C. Lee, "Visual marker systems for autonomous convoy operations: A review," *Journal of Robotics and Autonomous Systems*, vol. 115, pp. 123–135, 2020.
2. D. Chen, Y. Zhang, and H. Wang, "Integrating lidar and vision for autonomous vehicle navigation," *IEEE Transactions on Intelligent Vehicles*, vol. 4, no. 3, pp. 456–467, 2019.
3. K. Patel, R. Gupta, and P. Sharma, "Cost-effective sensor integration and real-time data fusion for autonomous vehicles," *International Journal of Advanced Robotic Systems*, vol. 18, no. 4, pp. 678–689, 2021.
4. R. Müller and S. Behnke, "Stereo vision and monocular depth estimation for autonomous vehicle following," in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, pp. 678–687, 2018.
5. A. Garcia, J. Soto, and J. Villanueva, "Overcoming occlusion and lighting challenges in visual marker systems for autonomous navigation," *Robotics and Autonomous Systems*, vol. 123, pp. 101–110, 2020.
6. Raspberry Pi Foundation, "Raspberry Pi 4," [Online]. Available: [Raspberry Pi](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/). Accessed: 08-August-2024.
7. J. Feßler, "Motor shield schematic diagram," RWU Robot Car Racing Competition, November 2020. Email: fessler@rwu.de.
8. RWU Moodle Course, "Motor control algorithm," *Robot Car Racing Competition*, Accessed: 08-August-2024.
9. Raspberry Pi Foundation, "Getting started with PiCamera," [Online]. Available: [PiCamera Guide](https://projects.raspberrypi.org/en/projects/getting-started-with-picamera). Accessed: 16-August-2024.
10. A. Name, "PiCamera test algorithm," Python Script. Available from: [Repository Link](https://yourrepositorylink.com).
11. R. Hartley and A. Zisserman, *Multiple View Geometry in Computer Vision*, 2nd ed., Cambridge University Press, 2004.
12. R. Szeliski, *Computer Vision: Algorithms and Applications*, 1st ed., Springer, 2010.
13. OpenCV, "Camera calibration with OpenCV," [Online]. Available: [OpenCV Documentation](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html). Accessed: 16-August-2024.
14. N. Nielsen, "Camera calibration in less than 5 minutes with OpenCV," YouTube, Available: [YouTube Tutorial](https://www.youtube.com/watch?v=_-BTKiamRTg). Accessed: 16-August-2024.
15. OpenCV, "ArUco marker detection and generation," [Online]. Available: [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html). Accessed: 09-August-2024.
16. Stanford University, "Pinhole camera model illustration," [Online]. Available: [Lecture Material](https://web.stanford.edu/class/ee267/lectures/lecture1.pdf). Accessed: 20-August-2024.
17. C. J. Willmott and K. Matsuura, "Advantages of the mean absolute error (MAE) over the root mean square error (RMSE)," *Climate Research*, vol. 30, pp. 79–82, 2005.
18. T. Chai and R. Draxler, "Root mean square error (RMSE) or mean absolute error (MAE)? Arguments against avoiding RMSE in the literature," *Geoscientific Model Development*, vol. 7, no. 3, pp. 1247–1250, 2014.
19. G. Bradski, "The OpenCV library," *Dr. Dobb's Journal of Software Tools*, 2000.
20. S. Garrido-Jurado, R. Muñoz-Salinas, F. J. Madrid-Cuevas, and M. J. Marín-Jiménez, "Automatic generation and detection of highly reliable fiducial markers under occlusion," *Pattern Recognition*, vol. 47, no. 6, pp. 2280–2292, 2014.
21. C. Schlegel, M. Waibel, and R. Siegwart, "Fast and robust localization with arbitrary landmark relations," in *IEEE/RSJ International Conference on Intelligent Robots and Systems*, IEEE, pp. 2723–2728, 2004.
22. A. Krause and C. S. Ong, "Contextual Gaussian process bandit optimization," in *Advances in Neural Information Processing Systems*, vol. 20, pp. 19–24, 2008.
23. G. Welch and G. Bishop, "An introduction to the Kalman filter," University of North Carolina, Department of Computer Science, Tech. Rep., 1995.

---

## Authors
- **Vardhan Vinodbhai Mistry**
- **Jai Doshi**

## Guided by
- **Prof. Dr. Stefan Elser**
