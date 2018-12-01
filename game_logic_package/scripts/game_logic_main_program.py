#!/usr/bin/env python
import rospy
import numpy as np
import cv2
import math
from hardware_package.communication_mainboard import ComportMainboard
from image_processing_package.msg import BallPoint
from image_processing_package.msg import BasketPoint
from general_package.msg import MoveSpeed
from enum import Enum

class State(Enum):
    IDLE = 1
    FIND_BALL = 2
    DRIVE_TO_BALL = 3
    ROTATE_TO_BASKET = 4

#  Ball & Basket global parameters position in a frame (maximum and minimum)
LEFT_BOUND = 0
RIGHT_BOUND = 640
CENTER_POINT = 355
HEIGHT = 480

# Ball parameters
BALL_LARGE = 50
LEFT_PERMITTED_FOR_BALL = CENTER_POINT - BALL_LARGE
RIGHT_PERMITTED_FOR_BALL = CENTER_POINT + BALL_LARGE

PERM_DISTANCE = 6

# Basket parameters
BASKET_LARGE = 30
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
BALL_DISTANCE_PERMITTED = 430

# Robot default speed
ROBOT_SPEED = 8
ROBOT_SPEED_REDUCED = 4

# Robot must stop inmediatelly.
BALL_CALIBRATED_YES = "Robot is perfectly calibrated."

WHEEL_LEFT_ANGLE = 60
WHEEL_RIGHT_ANGLE = 300
WHEEL_BACK_ANGLE = 180

# Default: 1
TASK_NUMBER = 1

# Printing message first sentence
PRINT_SENTENCE1 = "game_logic_package -> game_logic_package_main :  "

class GameLogic():

    def __init__(self):
        self.currentState = State.FIND_BALL
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
            print("ball: " + str(ballPoint))
            self.ball_x = ballPoint.x
            self.ball_y = ballPoint.y
            # Try to be more precise and detect the ball in different parts of the frame.
            # if (LEFT_PERMITTED_FOR_BALL <= ballPoint.x <= RIGHT_PERMITTED_FOR_BALL):
            #         print("game_logic_package_main -> STOP ROBOT. BALL IS IN THE CENTER.")
            #         self.ballSeen = True
            #         self.status = BALL_ON_CENTER
            # elif (RIGHT_PERMITTED_FOR_BALL < ballPoint.x <= RIGHT_BOUND):
            #         print("game_logic_package_main -> Should rotate to the right.")
            #         self.status = BALL_ON_THE_RIGHT
            # elif (LEFT_BOUND <= ballPoint.x < LEFT_PERMITTED_FOR_BALL):
            #         print("game_logic_package_main -> Should rotate to the left")
            #         self.status = BALL_ON_THE_LEFT
            # else:
            #         print("game_logic_package_main -> Should rotate still rotate (no matter direction)")
            #         self.status = BALL_UNKNOWN
            # # make sure where the ball is...
            # if (BALL_DISTANCE_PERMITTED <= ballPoint.y <= HEIGHT):
            #         self.ballDistance = BALL_CLOSE
            # else:
            #         self.ballDistance = BALL_FAR
            # if (self.status == BALL_ON_CENTER and self.ballDistance == BALL_CLOSE):
            #         self.status = BALL_CALIBRATED_YES

    def callback_basket(self, basketPoint):
            #print("basket: " + str(basketPoint))
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

    def handle_state(self):
        if self.currentState == State.IDLE:
            print("IDLE")
            gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))

        elif self.currentState == State.FIND_BALL:
            print("FIND_BALL")
            rotation_speed = 10
            min_rotation_speed = 3

            if self.ball_x != -1:
                self.currentState = State.DRIVE_TO_BALL

            left_wheel = round(calculate_speed(0, 0, WHEEL_LEFT_ANGLE, rotation_speed), 2)
            right_wheel = round(calculate_speed(0, 0, WHEEL_RIGHT_ANGLE, rotation_speed), 2)
            back_wheel = round(calculate_speed(0, 0, WHEEL_BACK_ANGLE, rotation_speed), 2)

            #print("lrb: {} {} {}".format(left_wheel, right_wheel, back_wheel))

            gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))

        elif self.currentState == State.DRIVE_TO_BALL:
            print("DRIVE_TO_BALL")
            speed = 0
            min_speed = 5
            rotation_speed = 10
            min_rotation_speed = 2

            if self.ball_x != -1:
                ball_offset_x = self.ball_x - 480
                rotation_speed = ball_offset_x / 50
                #print("ball_offset_x: {}".format(ball_offset_x))
                #print("rotation_speed: {}".format(rotation_speed))

                if 0 < rotation_speed < min_rotation_speed:
                    rotation_speed = min_rotation_speed
                elif -min_rotation_speed < rotation_speed < 0:
                    rotation_speed = -min_rotation_speed

                if abs(ball_offset_x) < 20:
                    self.currentState = State.DRIVE_TO_BALL
            else:
                self.currentState = State.FIND_BALL

            if self.ball_y != -1:
                ball_offset_y = 480 - self.ball_y
                speed = ball_offset_y / 10
                #print("ball_offset_y: {}".format(ball_offset_y))
                #print("speed: {}".format(speed))

                if 0 < speed < min_speed:
                    speed = min_speed
                elif -min_speed < speed < 0:
                    speed = -min_speed

                if abs(ball_offset_y) < 10:
                    self.currentState = State.ROTATE_TO_BASKET

            left_wheel = round(calculate_speed(speed, 90, WHEEL_LEFT_ANGLE, rotation_speed), 2)
            right_wheel = round(calculate_speed(speed, 90, WHEEL_RIGHT_ANGLE, rotation_speed), 2)
            back_wheel = round(calculate_speed(speed, 90, WHEEL_BACK_ANGLE, rotation_speed), 2)

            #print("lrb: {} {} {}".format(left_wheel, right_wheel, back_wheel))

            gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))

        elif self.currentState == State.ROTATE_TO_BASKET:
            print("ROTATE_TO_BASKET")

            speed = -10
            min_speed = 5
            rotation_speed = 5
            min_rotation_speed = 2

            if self.basket_x != -1:
                basket_offset_x = self.basket_x - 480
                speed = basket_offset_x / 50

            if self.ball_x == -1:
                self.currentState = State.FIND_BALL

            rotation_speed = rotation_speed / -2

            #move sideways and rotate at the same time
            left_wheel = round(calculate_speed(speed, 0, WHEEL_LEFT_ANGLE, rotation_speed), 2)
            right_wheel = round(calculate_speed(speed, 0, WHEEL_RIGHT_ANGLE, rotation_speed), 2)
            back_wheel = round(calculate_speed(speed, 0, WHEEL_BACK_ANGLE, rotation_speed), 2)

            gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))


def move_forward(speed):
    return MoveSpeed((-1)*speed, (1) * speed, 0, 0)

def move_backwards(speed):
    return MoveSpeed((1)*speed, (-1)*speed, 0, 0)

def rotate(speed):
    return MoveSpeed((-1)*speed, (-1)*speed, (-1)*speed, 0)

def circle_left(speed):
    return MoveSpeed(0, 0, speed, 0)

def circle_right(speed):
    return MoveSpeed(0, 0, (-1) * speed, 0)

def calculate_speed(robotSpeed, robotDirectionAngle, wheelAngle, rotation_speed):
    return robotSpeed * math.cos(math.radians(robotDirectionAngle - wheelAngle)) + rotation_speed

def calculate_robot_angle(x):
    return ((70 - ((x*70)/640)) + 55)

if __name__ == '__main__':
    try:
        gameLogic = GameLogic()
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            gameLogic.handle_state()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

# if __name__ == '__main__':
#     try:
#         gameLogic = GameLogic()
#         rate = rospy.Rate(30)
#         i=0
#         while not rospy.is_shutdown():
#             i=i+1
#             if (gameLogic.next_task == CALIBRATE_BALL):
#                 if (gameLogic.status == BALL_ON_CENTER):
#                     if (gameLogic.ballDistance == BALL_FAR):
#                         gameLogic.speed_pub.publish(MoveSpeed(10,-10,0,0))
#                         print (PRINT_SENTENCE1 + BALL_ON_CENTER)
#                     elif (gameLogic.ballDistance == BALL_CLOSE):
#                         gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                         print('close to ball')
#                 elif (gameLogic.status == BALL_ON_THE_LEFT):
#                     gameLogic.speed_pub.publish(MoveSpeed(10, 0, 10, 0))
#                 elif (gameLogic.status == BALL_ON_THE_RIGHT):
#                     gameLogic.speed_pub.publish(MoveSpeed(0, 10, 10, 0))
#                 elif (gameLogic.status == BALL_UNKNOWN):
#                     gameLogic.speed_pub.publish(rotate(-6))
#                     print(PRINT_SENTENCE1 + BALL_UNKNOWN)
#                 elif (gameLogic.status == BALL_CALIBRATED_YES):
#                     gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                     print(PRINT_SENTENCE1 + BALL_CALIBRATED_YES)
#                     gameLogic.next_task = CALIBRATE_BASKET
#             elif (gameLogic.next_task == CALIBRATE_BASKET):
#                 if (gameLogic.status_basket == BASKET_ON_THE_CENTER):
#                     gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                     gameLogic.next_task = RECALIBRATE_BASKET_AND_BALL
#                 elif (gameLogic.status_basket == BASKET_ON_THE_LEFT):
#                     print("BASKET on the LEFT. -> circle to the right")
#                     gameLogic.speed_pub.publish(circle_right(6))
#                 elif (gameLogic.status_basket == BASKET_ON_THE_RIGHT):
#                     gameLogic.speed_pub.publish(circle_left(6))
#                     print("BASKET on the RIGHT. -> circle to the left")
#                 elif (gameLogic.status_basket == BASKET_UNKNOWN):
#                     print("BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ")
#                     gameLogic.speed_pub.publish(circle_left(5))
#                 elif (gameLogic.next_task == RECALIBRATE_BASKET_AND_BALL):
#                     gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                     if (abs(gameLogic.ball_x-gameLogic.basket_x)<=PERM_DISTANCE):
#                         print("******* BALL=" + str(gameLogic.ball_x) + " . BASKET= " + str(gameLogic.basket_x) + " ******** ")
#                         gameLogic.next_task = THROW_BALL
#                     else:
#                         if (gameLogic.basket_x-gameLogic.ball_x > 0):
#                             left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_LEFT_ANGLE), 2)
#                             right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_RIGHT_ANGLE), 2)
#                             back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_BACK_ANGLE), 2)
#                             gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                         elif(gameLogic.ball_x-gameLogic.basket_x > 0):
#                             left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_LEFT_ANGLE), 2)
#                             right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_RIGHT_ANGLE), 2)
#                             back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_BACK_ANGLE), 2)
#                             gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                         elif (gameLogic.next_task == THROW_BALL):
#                             gameLogic.speed_pub.publish(MoveSpeed(ROBOT_SPEED, (-1) * ROBOT_SPEED, 0, 1300))
#
#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass

# if __name__ == '__main__':
#     try:
#             gameLogic = GameLogic()
#             rate = rospy.Rate(60)
#             i = 0
#             while not rospy.is_shutdown():
#                     i = i+1
#                     if (TASK_NUMBER == 1):
#                             if (gameLogic.next_task == CALIBRATE_BALL ):
#                                     if (gameLogic.status == BALL_ON_CENTER):
#
#                                         if (gameLogic.ballDistance == BALL_FAR):
#                                                     # T1 - Robot moves side to side
#                                                     gameLogic.speed_pub.publish(MoveSpeed(-10,10 ,0, 0))
#                                                     # left_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_LEFT_ANGLE),2)
#                                                     # right_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_RIGHT_ANGLE),2)
#                                                     # back_wheel = round(calculate_speed(ROBOT_SPEED, 90, WHEEL_BACK_ANGLE),2)
#                                                     # gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                                                     print (PRINT_SENTENCE1 + BALL_ON_CENTER)
#                                         elif(gameLogic.ballDistance == BALL_CLOSE):
#                                                 gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                                     elif (gameLogic.status == BALL_ON_THE_LEFT):
#                                             # T1 - Robot moves side to side
#                                             left_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_LEFT_ANGLE),2)
#                                             right_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_RIGHT_ANGLE),2)
#                                             back_wheel = round(calculate_speed(ROBOT_SPEED, 180, WHEEL_BACK_ANGLE),2)
#                                             gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                                             print(PRINT_SENTENCE1 + BALL_ON_THE_LEFT)
#                                             # gameLogic.speed_pub.publish(rotate(7))
#                                     elif (gameLogic.status == BALL_ON_THE_RIGHT):
#                                             # T1 - Robot moves side to side
#                                             left_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_LEFT_ANGLE),2)
#                                             right_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_RIGHT_ANGLE),2)
#                                             back_wheel = round(calculate_speed(ROBOT_SPEED, 0, WHEEL_BACK_ANGLE),2)
#                                             gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                                             print(PRINT_SENTENCE1 + BALL_ON_THE_RIGHT)
#                                             # gameLogic.speed_pub.publish(rotate(-7))
#                                     elif (gameLogic.status == BALL_UNKNOWN):
#                                             gameLogic.speed_pub.publish(rotate(-6))
#                                             print(PRINT_SENTENCE1 + BALL_UNKNOWN)
#                                     elif (gameLogic.status == BALL_CALIBRATED_YES):
#                                             gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                                             print(PRINT_SENTENCE1 + BALL_CALIBRATED_YES)
#                                             gameLogic.next_task = CALIBRATE_BASKET
#                             # NOW, THIS CODE IS TO MOVE DEPENDING ON WHERE THE BASKET IS...
#                             elif (gameLogic.next_task == CALIBRATE_BASKET):
#                                     if (gameLogic.status_basket == BASKET_ON_THE_CENTER):
#                                             gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                                             gameLogic.next_task = RECALIBRATE_BASKET_AND_BALL
#                                     elif (gameLogic.status_basket == BASKET_ON_THE_LEFT):
#                                             print("BASKET on the LEFT. -> circle to the right")
#                                             gameLogic.speed_pub.publish(circle_right(15))
#                                     elif (gameLogic.status_basket == BASKET_ON_THE_RIGHT):
#                                             gameLogic.speed_pub.publish(circle_left(15))
#                                             print("BASKET on the RIGHT. -> circle to the left")
#                                     elif (gameLogic.status_basket == BASKET_UNKNOWN):
#                                             print("BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ")
#                                             gameLogic.speed_pub.publish(circle_left(5))
#                             # Let's recalibrate a bit in order to throw the ball!
#                             elif (gameLogic.next_task == RECALIBRATE_BASKET_AND_BALL):
#                                     gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                                     if (abs(gameLogic.ball_x-gameLogic.basket_x)<=PERM_DISTANCE):
#                                             # GO THROUGH THE ROBOT!
#                                             print("******* BALL=" + str(gameLogic.ball_x) + " . BASKET= " + str(gameLogic.basket_x) + " ******** ")
#                                             gameLogic.next_task = THROW_BALL
#                                     else:
#                                             if (gameLogic.basket_x-gameLogic.ball_x > 0):
#                                                 # move a little bit to the right
#                                                 left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_LEFT_ANGLE), 2)
#                                                 right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_RIGHT_ANGLE), 2)
#                                                 back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 180, WHEEL_BACK_ANGLE), 2)
#                                                 gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                                             elif(gameLogic.ball_x-gameLogic.basket_x > 0):
#                                                 # move a little bit to the left
#                                                 left_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_LEFT_ANGLE), 2)
#                                                 right_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_RIGHT_ANGLE), 2)
#                                                 back_wheel = round(calculate_speed(ROBOT_SPEED_REDUCED, 0, WHEEL_BACK_ANGLE), 2)
#                                                 gameLogic.speed_pub.publish(MoveSpeed(left_wheel, right_wheel, back_wheel, 0))
#                             elif (gameLogic.next_task == THROW_BALL):
#                                     gameLogic.speed_pub.publish(MoveSpeed(ROBOT_SPEED, (-1) * ROBOT_SPEED, 0, 1300))
#                     rate.sleep()
#                     elif (TASK_NUMBER == 2):
#                         if (gameLogic.status != BALL_UNKNOWN):
#                             if (gameLogic.next_task == CALIBRATE_BALL):
#                                 if (gameLogic.status == BALL_ON_CENTER):
#                                     if (gameLogic.ballDistance == BALL_FAR):
#                                         # T1 - Robot moves side to side
#                                         move_forward(10)
#                                         print (PRINT_SENTENCE1 + BALL_ON_CENTER)
#                                     elif (gameLogic.ballDistance == BALL_CLOSE):
#                                         gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                                 elif (gameLogic.status == BALL_ON_THE_LEFT):
#                                     # T1 - Robot moves side to side
#                                     circle_left(2)
#                                     print(PRINT_SENTENCE1 + BALL_ON_THE_LEFT)
#                                     # gameLogic.speed_pub.publish(rotate(7))
#                                 elif (gameLogic.status == BALL_ON_THE_RIGHT):
#                                     # T1 - Robot moves side to side
#                                     circle_right(2)
#                                     print(PRINT_SENTENCE1 + BALL_ON_THE_RIGHT)
#                                     # gameLogic.speed_pub.publish(rotate(-7))
#                                 elif (gameLogic.status == BALL_UNKNOWN):
#                                     gameLogic.speed_pub.publish(rotate(-6))
#                                     print(PRINT_SENTENCE1 + BALL_UNKNOWN)
#                                 elif (gameLogic.status == BALL_CALIBRATED_YES):
#                                     gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                                     print(PRINT_SENTENCE1 + BALL_CALIBRATED_YES)
#                                     gameLogic.next_task = CALIBRATE_BASKET
#                                 # NOW, THIS CODE IS TO MOVE DEPENDING ON WHERE THE BASKET IS...
#                             elif (gameLogic.next_task == CALIBRATE_BASKET):
#                                 if (gameLogic.status_basket == BASKET_ON_THE_CENTER):
#                                     gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                                     gameLogic.next_task = RECALIBRATE_BASKET_AND_BALL
#                                 elif (gameLogic.status_basket == BASKET_ON_THE_LEFT):
#                                     print("BASKET on the LEFT. -> circle to the right")
#                                     gameLogic.speed_pub.publish(circle_right(6))
#                                 elif (gameLogic.status_basket == BASKET_ON_THE_RIGHT):
#                                     gameLogic.speed_pub.publish(circle_left(6))
#                                     print("BASKET on the RIGHT. -> circle to the left")
#                                 elif (gameLogic.status_basket == BASKET_UNKNOWN):
#                                     print(
#                                         "BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ********* BASKET UNKNOWN ")
#                                     gameLogic.speed_pub.publish(circle_left(5))
#                                 # Let's recalibrate a bit in order to throw the ball!
#                             elif (gameLogic.next_task == RECALIBRATE_BASKET_AND_BALL):
#                                 gameLogic.speed_pub.publish(MoveSpeed(0, 0, 0, 0))
#                                 if (abs(gameLogic.ball_x - gameLogic.basket_x) <= PERM_DISTANCE):
#                                     # GO THROUGH THE ROBOT!
#                                     print("******* BALL=" + str(gameLogic.ball_x) + " . BASKET= " + str(
#                                         gameLogic.basket_x) + " ******** ")
#                                     gameLogic.next_task = THROW_BALL
#                                 else:
#                                     if (gameLogic.basket_x - gameLogic.ball_x > 0):
#                                         # move a little bit to the right
#                                         circle_right(2)
#                                     elif (gameLogic.ball_x - gameLogic.basket_x > 0):
#                                         # move a little bit to the left
#                                         circle_left(2)
#                             elif (gameLogic.next_task == THROW_BALL):
#                                 gameLogic.speed_pub.publish(MoveSpeed(ROBOT_SPEED, (-1) * ROBOT_SPEED, 0, 1650))
#
#                             # T2 - The robot must go directly to the ball once ball is seen
#                             # NEEDS TO BE DONE!!
#                             if (gameLogic.ballDistance == BALL_FAR):
#                                 move_forward(10)
#                         elif (gameLogic.ballDistance == BALL_CLOSE):
#                                 gameLogic.speed_pub.publish(MoveSpeed(0,0,0,0))
#                         else:
#                             gameLogic.speed_pub.publish(rotate(-10))
#                     else:
#                         gameLogic.speed_pub.publish(circle_left(5))
#                     rate.sleep()
#     except rospy.ROSInterruptException:
#             pass
