#! /usr/bin/env python
import rospy
import numpy as np
import cv2
import math
from hardware_package.communication_mainboard import ComportMainboard
from image_processing_package.msg import BallPoint
from general_package.msg import MoveSpeed

# Ball parameters position in a frame (maximum and minimum)
CENTER_POINT = 320
BALL_LARGE = 20
LEFT_PERMITTED_FOR_BALL = CENTER_POINT - BALL_LARGE
RIGHT_PERMITTED_FOR_BALL = CENTER_POINT + BALL_LARGE
LEFT_BOUND = 0
RIGHT_BOUND = 640

# Parameters that changes depending on the position of the ball.
BALL_ON_THE_RIGHT = "ball is on the right"
BALL_ON_THE_LEFT = "ball is on the left"
BALL_ON_CENTER = "ball is on the center"
BALL_UNKNOWN = "ball is unknown"

BALL_CLOSE = "ball is close enough to robot."
BALL_FAR = "ball is far away"
BALL_DISTANCE_UNKNOWN = "ball is in an unknown position"

# Ball close distance
BALL_DISTANCE_PERMITTED = 410

# Robot default speed
ROBOT_SPEED = 10

# Angles of each wheel in order to use Omion
WHEEL_LEFT_ANGLE = 60 # 120
WHEEL_RIGHT_ANGLE = 300 # 210
WHEEL_BACK_ANGLE = 180 # 0

# Week 3: Two types of task for going through a ball.
TASK_NUMBER = 1

# Printing message first sentence
PRINT_SENTENCE1 = "game_logic_package -> game_logic_package_main :  "

class GameLogic():

    def __init__(self):
	self.ball_x = 0
	self.ball_y = 0
        rospy.init_node("game_logic_package", anonymous=True) #may be game_logic(?)
        rospy.Subscriber("ball_coordinates", BallPoint, self.callback)
        self.ballSeen = False
        self.status = BALL_UNKNOWN
        self.ballDistance = BALL_DISTANCE_UNKNOWN
        self.speed_pub = rospy.Publisher("move_speed", MoveSpeed, queue_size=10)

    def callback(self, ballPoint):
        print(str(ballPoint))
        self.ball_x = ballPoint.x
        self.ball_y = ballPoint.y
        # Try to be more precise and detect the ball in different parts of the frame.
        if (LEFT_PERMITTED_FOR_BALL <= ballPoint.x <= RIGHT_PERMITTED_FOR_BALL):
            print("game_logic_package_main -> STOP ROBOT. BALL IS IN THE CENTER.")
            #self.speed_pub.publish(MoveSpeed(0,0,0,0))
            self.ballSeen = True
            self.status = BALL_ON_CENTER
        elif (RIGHT_PERMITTED_FOR_BALL < ballPoint.x <= RIGHT_BOUND):
            print("game_logic_package_main -> Should rotate to the right.")
            self.status = BALL_ON_THE_RIGHT
        elif (LEFT_BOUND <= ballPoint.x < LEFT_PERMITTED_FOR_BALL):
            print("game_logic_package_main -> Should rotate to the left")
            self.status = BALL_ON_THE_LEFT
        else:
            print("game_logic_package_main -> Should rotate still rotate (no matter direction)")
            self.status = BALL_UNKNOWN
	if (BALL_DISTANCE_PERMITTED <= ballPoint.y <= 460):
	    self.ballDistance = BALL_CLOSE
	else:
	    self.ballDistance = BALL_FAR

def move_forward(speed):
    return MoveSpeed(speed, (-1) * speed, 0, 0)

def move_backwards(speed):
    return MoveSpeed((-1)*speed, speed, 0, 0)

def rotate(speed):
    return MoveSpeed(speed, speed, speed, 10)

def circle(speed):
    return MoveSpeed(0, 0, speed, 0)

def calculate_speed(robotSpeed, robotDirectionAngle, wheelAngle):
    return (robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle)))

def calculate_robot_angle(x):
    return ((70 - ((x*70)/640)) + 55)

if __name__ == '__main__':
    try:
        gameLogic = GameLogic()
        rate = rospy.Rate(4)
        while not rospy.is_shutdown():
            if (TASK_NUMBER == 1):
                if (gameLogic.status == BALL_ON_CENTER):
		    if (gameLogic.ballDistance == BALL_FAR):
			# T1 - Robot moves side to side
                    	left_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_LEFT_ANGLE),2)
                    	right_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_RIGHT_ANGLE),2)
                    	back_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_BACK_ANGLE),2)
                    	#gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                    	print (PRINT_SENTENCE1 + BALL_ON_CENTER)
                    	#gameLogic.speed_pub.publish(move_forward(6))
		    	gameLogic.speed_pub.publish(move_backwards(6))
		    elif (gameLogic.ballDistance == BALL_CLOSE):
			gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                elif(gameLogic.status == BALL_ON_THE_LEFT):
                    # T1 - Robot moves side to side
                    left_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_LEFT_ANGLE),2)
                    right_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_RIGHT_ANGLE),2)
                    back_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_BACK_ANGLE),2)
                    gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                    print(PRINT_SENTENCE1 + BALL_ON_THE_LEFT)
                    # gameLogic.speed_pub.publish(rotate(7))
                elif (gameLogic.status == BALL_ON_THE_RIGHT):
                    # T1 - Robot moves side to side
                    left_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_LEFT_ANGLE),2)
                    right_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_RIGHT_ANGLE),2)
                    back_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_BACK_ANGLE),2)
                    gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                    print(PRINT_SENTENCE1 + BALL_ON_THE_RIGHT)
                    # gameLogic.speed_pub.publish(rotate(-7))
                elif (gameLogic.status == BALL_UNKNOWN):
                    gameLogic.speed_pub.publish(rotate(-6))
                    print(PRINT_SENTENCE1 + BALL_UNKNOWN)
                rate.sleep()
            elif (TASK_NUMBER == 2):
                if (gameLogic.status != BALL_UNKNOWN):
                    # T2 - The robot must go directly to the ball once ball is seen
                    # NEEDS TO BE DONE!!
                    robot_angle = calculate_robot_angle(gameLogic.ball_x)
                    left_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_LEFT_ANGLE),2)
                    right_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_RIGHT_ANGLE),2)
                    back_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_BACK_ANGLE),2)
                    gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                else:
                    gameLogic.speed_pub.publish(rotate(-10))
            else:
		gameLogic.speed_pub.publish(MoveSpeed(10,10,10,0))
	    rate.sleep()
    except rospy.ROSInterruptException:
        pass
