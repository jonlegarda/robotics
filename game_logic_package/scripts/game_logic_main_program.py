#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import math
from hardware_package.communication_mainboard import ComportMainboard
from image_processing_package.msg import BallPoint
from image_processing_package.msg import BasketPoint
from general_package.msg import MoveSpeed

#  Ball & Basket global parameters position in a frame (maximum and minimum)
LEFT_BOUND = 0
RIGHT_BOUND = 640
CENTER_POINT = 395
HEIGHT = 480;

# Ball parameters
BALL_LARGE = 15
LEFT_PERMITTED_FOR_BALL = CENTER_POINT - BALL_LARGE
RIGHT_PERMITTED_FOR_BALL = CENTER_POINT + BALL_LARGE

PERM_DISTANCE = 8;

# Basket parameters
BASKET_LARGE = 10
LEFT_PERMITTED_FOR_BASKET = CENTER_POINT - BASKET_LARGE
RIGHT_PERMITTED_FOR_BASKET = CENTER_POINT + BASKET_LARGE

# Parameters that changes depending on the position of the ball.
BALL_ON_THE_RIGHT = "ball is on the right"
BALL_ON_THE_LEFT = "ball is on the left"
BALL_ON_CENTER = "ball is on the center"
BALL_UNKNOWN = "ball is unknown"

CALIBRATE_BALL = "ball calibration in progress..."
CALIBRATE_BASKET = "basket is unknown"
RECALIBRATE_BASKET_AND_BALL = "try to recalibrate a little bit more basket, ball, robot line."
THROW_BALL = "throw the ball!"

# Parameters that changes depending on the position of the basket
BASKET_ON_THE_RIGHT = "basket is on the right"
BASKET_ON_THE_LEFT = "basket is on the left"
BASKET_ON_THE_CENTER = "basket is on the center!"
BASKET_UNKNOWN = "basket is unknown"

# Parameters to control the ball distance.
BALL_CLOSE = "ball is close enough to robot."
BALL_FAR = "ball is far away"
BALL_DISTANCE_UNKNOWN = "ball is in an unknown position"

# Ball close distance
BALL_DISTANCE_PERMITTED = 440

# Robot default speed
ROBOT_SPEED = 8
ROBOT_SPEED_REDUCED = 3

# Robot must stop inmediatelly.
BALL_CALIBRATED_YES = "Robot is perfectly calibrated."

# Angles of each wheel in order to use Omion
WHEEL_LEFT_ANGLE = 60 # 120
WHEEL_RIGHT_ANGLE = 300 # 210
WHEEL_BACK_ANGLE = 180 # 0

# Default: 1
TASK_NUMBER = 1

# Printing message first sentence
PRINT_SENTENCE1 = "game_logic_package -> game_logic_package_main :  "

class GameLogic():

    def __init__(self):
        self.ball_x = 0
        self.ball_y = 0
        self.basket_x = -1
        self.basket_y = -1
        rospy.init_node("game_logic_package", anonymous=True) #may be game_logic(?)
        rospy.Subscriber("ball_coordinates", BallPoint, self.callback)
        # Basket calibration
        rospy.Subscriber("basket_coordinates", BasketPoint, self.callback_basket)
        self.ballSeen = False
        self.status = BALL_UNKNOWN
        self.status_basket = BASKET_UNKNOWN
        self.ballDistance = BALL_DISTANCE_UNKNOWN
        self.next_task = CALIBRATE_BALL
        self.speed_pub = rospy.Publisher("move_speed", MoveSpeed, queue_size=10)

    def callback(self, ballPoint):
            print(str(ballPoint))
            self.ball_x = ballPoint.x
            self.ball_y = ballPoint.y
            # Try to be more precise and detect the ball in different parts of the frame.
            if (LEFT_PERMITTED_FOR_BALL <= ballPoint.x <= RIGHT_PERMITTED_FOR_BALL):
                    print("game_logic_package_main -> STOP ROBOT. BALL IS IN THE CENTER.")
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
            # make sure where the ball is...
            if (BALL_DISTANCE_PERMITTED <= ballPoint.y <= HEIGHT):
                    self.ballDistance = BALL_CLOSE
            else:
                    self.ballDistance = BALL_FAR
            if (self.status == BALL_ON_CENTER and self.ballDistance == BALL_CLOSE):
                    self.status = BALL_CALIBRATED_YES

    def callback_basket(self, basketPoint):
            print(str(basketPoint))
            self.basket_x = basketPoint.x
            self.basket_y = basketPoint.y
            if (LEFT_BOUND <= basketPoint.x < LEFT_PERMITTED_FOR_BASKET):
                    self.status_basket = BASKET_ON_THE_LEFT
                    #print("game_logic_package_main -> BASKET. Basket is on the left.")
            elif (RIGHT_PERMITTED_FOR_BASKET < basketPoint.x <= RIGHT_BOUND):
                    self.status_basket = BASKET_ON_THE_RIGHT
                    #print("game_logic_package_main -> BASKET. Basket is on the right.")
            elif (LEFT_PERMITTED_FOR_BASKET <= basketPoint.x <= RIGHT_PERMITTED_FOR_BASKET):
                    self.status_basket = BASKET_ON_THE_CENTER
                    #print("game_logic_package_main -> BASKET. Basket is on the CENTER. STOP!")
            else:
                    self.status_basket = BASKET_UNKNOWN
                    #print("game_logic_package_main -> BASKET. Where is the fucking basket?")

def move_forward(speed):
    return MoveSpeed(speed, (-1) * speed, 0, 0)

def move_backwards(speed):
    return MoveSpeed((-1)*speed, speed, 0, 0)

def rotate(speed):
    return MoveSpeed(speed, speed, speed, 10)

def circle_left(speed):
    return MoveSpeed(0, 0, speed, 0)

def circle_right(speed):
    return MoveSpeed(0, 0, (-1) * speed, 0)

def calculate_speed(robotSpeed, robotDirectionAngle, wheelAngle):
    return (robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle)))

def calculate_robot_angle(x):
    return ((70 - ((x*70)/640)) + 55)

if __name__ == '__main__':
    try:
            gameLogic = GameLogic()
            rate = rospy.Rate(10)
            i = 0
            while not rospy.is_shutdown():
                    i = i+1
                    if (TASK_NUMBER == 1):
                            if (gameLogic.next_task == CALIBRATE_BALL ):
                                    if (gameLogic.status == BALL_ON_CENTER):
                                            if (gameLogic.ballDistance == BALL_FAR):
                                                    # T1 - Robot moves side to side
                                                    left_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_LEFT_ANGLE),2)
                                                    right_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_RIGHT_ANGLE),2)
                                                    back_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_BACK_ANGLE),2)
                                                    gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                                                    print (PRINT_SENTENCE1 + BALL_ON_CENTER)
                                            elif(gameLogic.ballDistance == BALL_CLOSE):
                                                    gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                                    elif (gameLogic.status == BALL_ON_THE_LEFT):
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
                                    elif (gameLogic.status == BALL_CALIBRATED_YES):
                                            gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                                            print(PRINT_SENTENCE1 + BALL_CALIBRATED_YES)
                                            gameLogic.next_task = CALIBRATE_BASKET
                            # NOW, THIS CODE IS TO MOVE DEPENDING ON WHERE THE BASKET IS...
                            elif (gameLogic.next_task == CALIBRATE_BASKET):
                                    if (gameLogic.status_basket == BASKET_ON_THE_CENTER):
                                            gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                                            gameLogic.next_task = RECALIBRATE_BASKET_AND_BALL
                                    elif (gameLogic.status_basket == BASKET_ON_THE_LEFT):
                                            print("BASKET on the LEFT. -> circle to the right")
                                            gameLogic.speed_pub.publish(circle_right(6))
                                    elif (gameLogic.status_basket == BASKET_ON_THE_RIGHT):
                                            gameLogic.speed_pub.publish(circle_left(6))
                                            print("BASKET on the RIGHT. -> circle to the left")
                                    elif (gameLogic.status_basket == BASKET_UNKNOWN):
                                            print("BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ")
                                            gameLogic.speed_pub.publish(circle_left(5))
                            # Let's recalibrate a bit in order to throw the ball!
                            elif (gameLogic.next_task == RECALIBRATE_BASKET_AND_BALL):
                                    gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                                    if (abs(gameLogic.ball_x-gameLogic.basket_x)<=PERM_DISTANCE):
                                            # GO THROUGH THE ROBOT!
                                            gameLogic.next_task = THROW_BALL
                                    else:
                                            if (gameLogic.basket_x-gameLogic.ball_x > 0):
                                                # move a little bit to the right
                                                left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_LEFT_ANGLE), 2)
                                                right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_RIGHT_ANGLE), 2)
                                                back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_BACK_ANGLE), 2)
                                                gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                                            elif(gameLogic.ball_x-gameLogic.basket_x > 0):
                                                # move a little bit to the left
                                                left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_LEFT_ANGLE), 2)
                                                right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_RIGHT_ANGLE), 2)
                                                back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_BACK_ANGLE), 2)
                                                gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                            elif (gameLogic.next_task == THROW_BALL):
                                    gameLogic.speed_pub.publish(MoveSpeed(ROBOT_SPEED, (-1) * ROBOT_SPEED, 0, 1200))
                    rate.sleep()
                    '''elif (TASK_NUMBER == 2):
                        if (gameLogic.status != BALL_UNKNOWN):
                            # T2 - The robot must go directly to the ball once ball is seen
                            # NEEDS TO BE DONE!!
                            if (gameLogic.ballDistance == BALL_FAR):
                                robot_angle = calculate_robot_angle(gameLogic.ball_x)
                                left_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_LEFT_ANGLE),2)
                                right_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_RIGHT_ANGLE),2)
                                back_wheel = round(calculate_speed(ROBOT_SPEED, robot_angle, WHEEL_BACK_ANGLE),2)
                                gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
                        elif (gameLogic.ballDistance == BALL_CLOSE):
                                gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
                        else:
                            gameLogic.speed_pub.publish(rotate(-10))
                    else:
                        gameLogic.speed_pub.publish(circle_left(5))
                    rate.sleep()'''
    except rospy.ROSInterruptException:
            pass
