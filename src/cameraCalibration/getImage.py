from picamera2 import Picamera2, Preview
import time
from time import sleep

# Initialize PiCamera object
picam2 = Picamera2()
capture_config = picam2.create_still_configuration()
picam2.start(show_preview=True)

# Set resolution (optional)
picam2.resolution = (640, 480)  # Set your desired resolution

# Capture and save images
for i in range(20):  # Capture 13 images
    # Capture image
    sleep(5)  # Optional delay to allow the camera to adjust to light levels
    picam2.capture_file(f'/home/vardhan/scout/cameraCalibration/images/image_{i}.jpg')  # Save image with a unique filename
    print(f"Image {i} saved")


# Release PiCamera resources
picam2.close()
