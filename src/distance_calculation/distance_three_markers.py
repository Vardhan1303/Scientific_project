import cv2
import numpy as np

# Define Aruco dictionary and ID filter
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
target_id = 13  # Target marker ID

# Intrinsic camera matrix K
camera_matrix = np.array([
    [1013.7597, 0, 640],  # Assuming cx is 640 (half of 1280)
    [0, 1290.0994, 360],  # Assuming cy is 360 (half of 720)
    [0, 0, 1]
])

# Distortion coefficients
dist_coeffs = np.array([0.16505935, -0.17688996, 0.00816178, 0.11956643, 0.46090252])

def estimate_marker_pose(frame, camera_matrix, distortion_coefficients):
    """
    Estimates the pose of detected Aruco markers in a frame.

    Args:
        frame: The input frame (BGR image).
        camera_matrix: Camera calibration matrix.
        distortion_coefficients: Distortion coefficients from camera calibration.

    Returns:
        A tuple containing:
            - Detected Aruco corners (list of lists) or None if no markers found.
            - Detected Aruco marker IDs (list) or None if no markers found.
    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_points = cv2.aruco.detectMarkers(gray_frame, aruco_dict)

    return corners, ids

def calculate_distance(tvec):
    """
    Calculates the distance based on the Z-component of the translation vector (assuming Z represents depth).

    Args:
        tvec: Translation vector from marker pose estimation.

    Returns:
        The calculated distance in centimeters.
    """
    # Convert meters to centimeters
    distance_meters = tvec[0][0][2]
    distance_cm = distance_meters * 100
    return distance_cm

def calculate_centroid(corners):
    """
    Calculates the centroid of the given marker corners.

    Args:
        corners: List of corner points for each marker.

    Returns:
        The (x, y) coordinates of the centroid.
    """
    sum_x, sum_y = 0, 0
    for corner in corners:
        # Get the center of each marker
        center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
        center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)
        sum_x += center_x
        sum_y += center_y

    centroid_x = int(sum_x / len(corners))
    centroid_y = int(sum_y / len(corners))
    
    return centroid_x, centroid_y

# Main loop
def main():
    cap = cv2.VideoCapture(0)  # Capture from the default camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    try:
        while True:
            # Capture frame
            ret, frame = cap.read()
            if not ret:
                print("Failed to capture image")
                break

            # Detect Aruco marker
            aruco_corners, aruco_ids = estimate_marker_pose(frame, camera_matrix, dist_coeffs)

            if aruco_corners is not None and aruco_ids is not None:
                valid_corners = []
                valid_tvecs = []

                for i, corner in enumerate(aruco_corners):
                    # Process only the markers with ID 13
                    if aruco_ids[i][0] == target_id:
                        # Estimate pose for each detected marker
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)
                        
                        if tvec.shape[0] == 1:  # Ensure tvec has expected shape
                            valid_corners.append(corner)
                            valid_tvecs.append(tvec)
                            
                            # Draw bounding box around the detected marker with a thicker line
                            x, y, w, h = cv2.boundingRect(corner.reshape(-1, 2))
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

                # If 1, 2, or 3 markers are detected, calculate and display average distance and centroid
                if 1 <= len(valid_corners) <= 3:
                    # Calculate the average distance
                    avg_distance = np.mean([calculate_distance(tvec) for tvec in valid_tvecs])

                    # Calculate the centroid of the markers
                    centroid_x, centroid_y = calculate_centroid(valid_corners)
                    
                    # Draw a red dot at the centroid
                    cv2.circle(frame, (centroid_x, centroid_y), 5, (0, 0, 255), -1)

                    # Display average distance in the top-left corner in white font
                    cv2.putText(frame, f"Avg Distance: {avg_distance:.2f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            # Display frame
            cv2.imshow('ArUco Marker Distance Measurement', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
