import cv2
import numpy as np

# Define Aruco dictionary (adjust if using a different one)
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)

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
                for i, corner in enumerate(aruco_corners):
                    # Estimate pose for each detected marker
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, 0.05, camera_matrix, dist_coeffs)
                   
                    if tvec.shape[0] == 1:  # Ensure tvec has expected shape
                        distance = calculate_distance(tvec)
                        marker_id = aruco_ids[i][0]

                        # Draw bounding box around the detected marker with a thicker line
                        x, y, w, h = cv2.boundingRect(corner.reshape(-1, 2))
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

                        # Display ID and distance in the top-left corner in white font
                        cv2.putText(frame, f"ID: {marker_id}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        cv2.putText(frame, f"Distance: {distance:.2f} cm", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

                        # Calculate the center of the marker
                        center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
                        center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)
                        
                        # Draw a red dot at the center of the marker
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            # Display frame
            cv2.imshow('ArUco Marker Distance Measurement', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
