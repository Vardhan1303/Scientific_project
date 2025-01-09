from gpiozero import Robot
from time import sleep

robot = Robot(right = (4,24,19), left = (17,22,18))

robot.forward(0.5, curve_left = 0.9)
sleep(5)
robot.forward(0)