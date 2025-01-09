import cv2
import numpy as np

# Set marker size and dictionary
marker_size = 50
marker_id = 13
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_1000)

# Create an empty canvas
canvas_size = 3 * marker_size  # Adjusted canvas size for better visualization
canvas = np.ones((canvas_size, canvas_size), dtype=np.uint8) * 255  # White canvas

# Draw the marker on the canvas
marker = cv2.aruco.generateImageMarker(dictionary, marker_id, marker_size)
marker_center = (canvas_size // 2 - marker_size // 2, canvas_size // 2 - marker_size // 2)
canvas[marker_center[1]:marker_center[1] + marker_size, marker_center[0]:marker_center[0] + marker_size] = marker

# Define the file path
file_path = r'F:\MSc Mechatronics\Project\2. Zombie truck\Python_code\aruco_marker\aruco_marker.jpg'

# Save marker to the specified file path
cv2.imwrite(file_path, canvas)

# Display marker
cv2.imshow("ArUco Marker", canvas)
cv2.waitKey(0)
