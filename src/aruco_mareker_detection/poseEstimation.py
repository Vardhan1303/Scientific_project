import cv2
import numpy as np
import time
import yaml
from libcamera import Transform
from picamera2 import Picamera2, Preview

marker_size = 0.15
marker_id = cv2.aruco.DICT_4X4_50, cv2.aruco.DICT_5X5_50, cv2.aruco.DICT_6X6_50

# Function to load camera calibration parameters from a YAML file
def load_calibration_parameters(yaml_file):
    with open(yaml_file, 'r') as f:
        calibration_data = yaml.load(f, Loader=yaml.FullLoader)
    
    camera_matrix = np.array(calibration_data['camera_matrix'])
    dist_coeffs = np.array(calibration_data['dist_coeff'])
    
    return camera_matrix, dist_coeffs

if __name__ == "__main__":
    # Load camera calibration parameters from YAML file
    camera_matrix, dist_coeffs = load_calibration_parameters('/home/vardhan/scout/cameraCalibration/cameraCalibration.yaml')

    # Initialize the webcam
    picam2 = Picamera2()
    picam2.start_preview()
    picam2.start()

    # Load the ARUCO Dictionary and Parameters
    arucoParams = cv2.aruco.DetectorParameters()

    while True:
        # Capture frame-by-frame
        capture_result = picam2.capture_array()
        time.sleep(1)
        print("Capture Result:", capture_result)
        frame = capture_result
        print("frame capture successfull", frame)
        
        # Convert frame to RGB format if it has four channels
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2RGB)
        
        # Detect ARUCO markers in the image
        for dict_type in [cv2.aruco.DICT_5X5_50]:
            arucoDict = cv2.aruco.getPredefinedDictionary(dict_type)
            arucoParams = cv2.aruco.DetectorParameters_create()
            grayimg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            (corners, ids, rejected) = cv2.aruco.detectMarkers(grayimg, arucoDict, parameters=arucoParams)
    
            # Draw detected markers and bounding boxes
            if ids is not None:
                frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            # Draw bounding boxes around the detected markers
                for corner in corners:
                    x, y, w, h = cv2.boundingRect(corner.reshape(-1, 2))
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        
            else:
                print("No ARUCO markers detected.")

            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    picam2.release()
    cv2.destroyAllWindows()
