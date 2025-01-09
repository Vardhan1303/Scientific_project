import cv2
import numpy as np

# Load the dictionary for ArUco markers
dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

# Initialize the detector parameters correctly
parameters = cv2.aruco.DetectorParameters_create()

# Define marker size (in meters)
marker_size = 0.05  # Adjust this based on your actual marker size

# Camera calibration parameters
camera_matrix = np.array([[1013.759674997803, 0, 320], 
                          [0, 1290.099400876822, 240], 
                          [0, 0, 1]])  # fx, fy, cx, cy

# Distortion coefficients from your calibration
dist_coeffs = np.array([[ 0.16505935, -0.17688996,  0.00816178,  0.11956643,  0.46090252 ]])

# Initialize the video capture from webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

    if ids is not None:
        for i in range(len(ids)):
            # Define 3D points corresponding to the corners of the marker
            marker_points_3d = np.array([[0, 0, 0], 
                                         [marker_size, 0, 0], 
                                         [marker_size, marker_size, 0], 
                                         [0, marker_size, 0]])

            # Estimate pose of each marker if enough corners are detected
            if len(corners[i][0]) >= 4:
                # Estimate pose of each marker
                rvecs, tvecs, _ = cv2.solvePnP(
                    objectPoints=marker_points_3d,
                    imagePoints=corners[i][0],
                    cameraMatrix=camera_matrix,
                    distCoeffs=dist_coeffs
                )

                # Check if solvePnP was successful
                if isinstance(rvecs, np.ndarray):
                    # Draw axis manually with thicker lines
                    axis_length = marker_size
                    axis_points = np.float32([[axis_length, 0, 0], 
                                              [0, axis_length, 0], 
                                              [0, 0, axis_length]]).reshape(-1, 3)
                    image_points, _ = cv2.projectPoints(axis_points, rvecs, tvecs, camera_matrix, dist_coeffs)

                    corner = tuple(corners[i][0][0].ravel().astype(int))
                    cv2.line(frame, corner, tuple(image_points[0].ravel().astype(int)), (0, 0, 255), 5)  # X axis in red
                    cv2.line(frame, corner, tuple(image_points[1].ravel().astype(int)), (0, 255, 0), 5)  # Y axis in green
                    cv2.line(frame, corner, tuple(image_points[2].ravel().astype(int)), (255, 0, 0), 5)  # Z axis in blue

    # Draw detected markers
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
