from gpiozero import Robot
from time import sleep

robot = Robot(right = (4,24,19), left = (17,22,18))

robot.left(0.5)
sleep(2)

robot.right(0.5)
sleep(2)

robot.forward(0.5)
sleep(2)

robot.stop()