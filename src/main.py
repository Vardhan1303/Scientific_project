import cv2
import numpy as np
import time
from gpiozero import Robot, Motor
import yaml
import RPi.GPIO as GPIO
import logging
from collections import deque

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Set up GPIO pins for motor control
left_motor = Motor(17, 22)
right_motor = Motor(4, 24)
robot = Robot(left=left_motor, right=right_motor)

# Load camera calibration parameters from YAML file
def load_calibration_parameters(file_path):
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            camera_matrix = np.array(data['camera_matrix'])
            dist_coeffs = np.array(data['dist_coeff'])
        return camera_matrix, dist_coeffs
    except Exception as e:
        logging.error(f"Error loading calibration parameters: {e}")
        raise

# Load camera calibration parameters
camera_matrix, dist_coeffs = load_calibration_parameters('/home/vardhan/scout/cameraCalibration/cameraCalibration.yaml')

# Define ArUco marker parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
parameters = cv2.aruco.DetectorParameters_create()

def calculate_distance(tvec):
    distance_meters = tvec[0][0][2]
    distance_cm = int(distance_meters * 100)
    return distance_cm

def initialize_camera():
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise Exception("Camera could not be opened.")
        return cap
    except Exception as e:
        logging.error(f"Error initializing camera: {e}")
        raise

def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    return corners, ids

def control_robot(distance, desired_distance, lateral_deviation, correction_factors, predicted_deviation):
    left_motor_correction, right_motor_correction = correction_factors

    if distance > desired_distance + 5:
        logging.info("Moving forward")
        left_motor.forward(0.4 * left_motor_correction)
        right_motor.forward(0.4 * right_motor_correction)
        time.sleep(0.2)
        if predicted_deviation > 10:
            logging.info("Slightly forward and right")
            left_motor.forward(0.4 * left_motor_correction)
            right_motor.forward(0.35 * right_motor_correction)
            time.sleep(0.1)
        elif predicted_deviation < -10:
            logging.info("Slightly forward and left")
            left_motor.forward(0.3 * left_motor_correction)
            right_motor.forward(0.3 * right_motor_correction)
            time.sleep(0.1)
    elif distance < desired_distance - 5:
        logging.info("Moving backward")
        left_motor.backward(0.3 * left_motor_correction)
        right_motor.backward(0.3 * right_motor_correction)
        time.sleep(0.2)
        if predicted_deviation > 10:
            logging.info("Slightly backward and right")
            left_motor.backward(0.3 * left_motor_correction)
            right_motor.backward(0.25 * right_motor_correction)
            time.sleep(0.1)
        elif predicted_deviation < -10:
            logging.info("Slightly backward and left")
            left_motor.backward(0.25 * left_motor_correction)
            right_motor.backward(0.3 * right_motor_correction)
            time.sleep(0.1)
    else:
        logging.info("Stopping")
        left_motor.stop()
        right_motor.stop()

def draw_annotations(frame, corners, ids, distance):
    if ids is not None and len(ids) > 0:
        for i, corner in enumerate(corners):
            marker_id = ids[i][0]
            if marker_id == 13:  # Use the specified marker ID
                int_corners = np.int0(corner)
                cv2.polylines(frame, int_corners, True, (0, 255, 0), 2)
                center = tuple(np.mean(corner[0], axis=0).astype(int))
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {marker_id} Dist: {distance}cm", (center[0], center[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    else:
        cv2.putText(frame, "No ArUco detected", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return frame

# Kalman filter class
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_estimate):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.estimate = initial_estimate
        self.kalman_gain = 0

    def update(self, measurement):
        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.estimate += self.kalman_gain * (measurement - self.estimate)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.estimate

# Initialize camera
cap = initialize_camera()

# Define desired distance from the marker
desired_distance = 30

# Correction factors for motor speeds
left_motor_correction = 1.0
right_motor_correction = 0.8  # Adjusted value based on your measurements

# Initialize Kalman filter for lateral deviation prediction
kalman_filter = KalmanFilter(process_variance=1e-5, measurement_variance=0.1**2, estimation_error=1, initial_estimate=0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            logging.error("Failed to capture frame.")
            break

        corners, ids = process_frame(frame)
        logging.info(f"Detected IDs: {ids}")

        if ids is not None and len(ids) > 0:
            for i, corner in enumerate(corners):
                marker_id = ids[i][0]
                logging.info(f"Detected marker ID: {marker_id}")

                if marker_id == 13:  # Use the specified marker ID
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)
                    distance = calculate_distance(tvec)

                    lateral_deviation = int(np.mean(corner[0][:, 0]) - (frame.shape[1] / 2))
                    logging.info(f"Distance: {distance} cm, Lateral Deviation: {lateral_deviation} pixels")

                    # Update the Kalman filter with the new lateral deviation
                    predicted_deviation = kalman_filter.update(lateral_deviation)

                    control_robot(distance, desired_distance, lateral_deviation, (left_motor_correction, right_motor_correction), predicted_deviation)

                    # Draw annotations on the frame
                    frame = draw_annotations(frame, corners, ids, distance)
                    break
        else:
            logging.info("No marker detected. Stopping.")
            robot.stop()

            # Draw annotations on the frame
            frame = draw_annotations(frame, None, None, None)

        cv2.imshow('Frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
