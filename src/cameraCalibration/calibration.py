import numpy as np
import cv2 as cv
import glob
import pickle
import yaml

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

# Chessboard specifications
chessboardSize = (9, 9)  # Number of inner corners per a chessboard row and column
frameSize = (640, 480)  # Size of the images used for calibration

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

size_of_chessboard_squares_mm = 10  # Update this to match your square size
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

# Modify this path accordingly
images = glob.glob('/home/vardhan/scout/cameraCalibration/images/*.jpg')

for image in images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(1000)

cv.destroyAllWindows()

# Check if there are valid images for calibration
if len(objpoints) == 0:
    print("No valid images found for calibration.")
    exit()

############## CALIBRATION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

# Save the camera calibration result for later use (we won't worry about rvecs / tvecs)
save_path = '/home/vardhan/scout/cameraCalibration/'
with open(save_path + "calibration.pkl", "wb") as f:
    pickle.dump((cameraMatrix, dist), f)
with open(save_path + "cameraMatrix.pkl", "wb") as f:
    pickle.dump(cameraMatrix, f)
with open(save_path + "dist.pkl", "wb") as f:
    pickle.dump(dist, f)

# Save the camera calibration result for later use
calibration_data = {
    'camera_matrix': cameraMatrix.tolist(),
    'dist_coeff': dist.tolist()
}

with open(save_path + "cameraCalibration.yaml", "w") as yaml_file:
    yaml.dump(calibration_data, yaml_file)

############## UNDISTORTION #####################################################

# Modify this path accordingly
save_undistorted_path = '/home/vardhan/scout/cameraCalibration/undistorted_images/'
for image_path in images:
    img = cv.imread(image_path)
    h, w = img.shape[:2]
    newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w, h), 1, (w, h))

    # Undistort
    dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

    # Crop the image
    x, y, w, h = roi
    dst = dst[y:y + h, x:x + w]

    cv.imwrite(save_undistorted_path + 'undistorted_' + image_path.split('/')[-1], dst)

# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print("Total error: {}".format(mean_error / len(objpoints)))

# Load the camera matrix from the calibration result
cameraMatrix = pickle.load(open(save_path + "cameraMatrix.pkl", "rb"))

# Extract focal length values (in pixels)
fx = cameraMatrix[0, 0]  # Focal length along x-axis
fy = cameraMatrix[1, 1]  # Focal length along y-axis

print("Focal length (fx):", fx)
print("Focal length (fy):", fy)