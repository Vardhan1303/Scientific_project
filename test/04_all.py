# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
from gpiozero import Robot
from time import sleep
import numpy as np

import os
from math import cos, sin, pi, floor
import pygame
from adafruit_rplidar import RPLidar

def process_data(lidar_output, camera_output, robot):
    pass

if __name__ == "__main__":
    try:
        # initialize the camera and grab a reference to the raw camera capture
        camera = PiCamera()
        camera.resolution = (320, 240)
        time.sleep(2)
        camera_output = np.empty((440, 320, 3), dtype=np.uint8)
        
        # initialize the Robo
        robot = Robot(right = (4,24,19), left = (17,22,18))
        
        # initialize the RPLidar
        PORT_NAME = '/dev/ttyUSB0'
        lidar = RPLidar(None, PORT_NAME)
        scan_data = [0]*360
        
        for scan in lidar.iter_scans():
            for (_, angle, distance) in scan:
                scan_data[min([359, floor(angle)])] = distance
                
            camera.capture(camera_output, 'rgb')        
            
            
            process_data(scan_data, camera_output)
    finally:
        lidar.stop()
        lidar.disconnect()