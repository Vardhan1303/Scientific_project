from time import sleep
from gpiozero import Robot
import pygame

#Pins where motors are connected at (see Motor controller schaltplan for more)
robot = Robot(right = (4,24,19), left = (17,22,18))

#Initial values for the motors, start motors at no speed
InitMotor = 0

#DC Motor values
MotorMax = 1 #Full speed
MotorMin = 0 #No speed

# Acceleration value
MotorAccelerationSpeed = 0.1 #to be changed as needed

#pygame initialization
pygame.init()
#pygame display mode
screen=pygame.display.set_mode((100,100))

running = True

#Dictionary containing command values, used to control the motors at the same time; KEY UP, DOWN, LEFT AND RIGHT
CommandValues = {'K_UP':0,
                      'K_DOWN':0,
                      'K_LEFT':0,
                      'K_RIGHT':0,
                      'Break':0
                     }

print("READY")
print("Press ESC to Quit")


while running:
    for event in pygame.event.get():
        # if key down is pressed
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_DOWN:
                # both motors decelerating
                CommandValues['K_DOWN'] = 1
            if event.key == pygame.K_UP:
                # both motors accelerating
                CommandValues['K_UP'] = 1
            if event.key == pygame.K_RIGHT:
                # only right motor accelerates
                CommandValues['K_RIGHT'] = 1
            if event.key == pygame.K_LEFT:
                # only left motor accelerates
                CommandValues['K_LEFT'] = 1

            if event.key == pygame.K_r:  # Press r to bring to initial values servo and motor
                robot.forward(speed=0)

            if event.key == pygame.K_b:
                CommandValues['Break'] = 1

            if event.key == pygame.K_ESCAPE: # press esc to exit
                print("EXIT")
                running = False

        if event.type == pygame.KEYUP:

            if event.key == pygame.K_DOWN:
                CommandValues['K_DOWN'] = 0
            if event.key == pygame.K_UP:
                CommandValues['K_UP'] = 0
            if event.key == pygame.K_RIGHT:  
                CommandValues['K_RIGHT'] = 0
            if event.key == pygame.K_LEFT:
                CommandValues['K_LEFT'] = 0
            if event.key == pygame.K_b:
                CommandValues['Break'] = 0

        # Translate values into motor movements
        if CommandValues['K_RIGHT'] == 1:
            InitMotor = InitMotor + MotorAccelerationSpeed
            if InitMotor >= MotorMax:
                InitMotor = MotorMax
            robot.right(speed=InitMotor)

        if CommandValues['K_LEFT'] == 1:
            InitMotor = InitMotor + MotorAccelerationSpeed
            if InitMotor >= MotorMax:
                InitMotor = MotorMax
            robot.left(speed=InitMotor)

        if CommandValues['K_DOWN'] == 1:
            InitMotor = InitMotor - MotorAccelerationSpeed

            if InitMotor <= MotorMin:  # if positional value exeeds the set limit
                InitMotor = MotorMin  # set it back to its limit value
                
            robot.backward(speed=InitMotor)

        if CommandValues['K_UP'] == 1:

            InitMotor = InitMotor + MotorAccelerationSpeed

            if InitMotor >= MotorMax:  # if positional value exeeds the set limit
                InitMotor = MotorMax  # set it back to its limit value
            

            robot.forward(speed=InitMotor)

pygame.quit()
print("End.")