#! /usr/bin/env python
import rospy
import numpy as np
import cv2
from hardware_package.communication_mainboard import ComportMainboard
from image_processing_package.msg import BallPoint
from general_package.msg import MoveSpeed

class GameLogic():

	def __init__(self):
		rospy.init_node("game_logic_package", anonymous=True) #may be game_logic(?)
		rospy.Subscriber("ball_coordinates", BallPoint, self.callback)
		self.ballSeen = False
		self.speed_pub = rospy.Publisher("move_speed", MoveSpeed, queue_size=10)

	def callback(self, ballPoint):
		print(str(ballPoint))
		if (ballPoint.x < 350 and ballPoint.x > 290):
			print("- STOP ROBOT! -")
			self.speed_pub.publish(MoveSpeed(0,0,0,0))
			self.ballSeen = True

def move_forward(speed):
	return MoveSpeed(speed, (-1) * speed, 0, 0)

def move_backwards(speed):
	return MoveSpeed((-1)*speed, speed, 0, 0)

def rotate(speed):
	return MoveSpeed(speed, speed, speed, 10)

def circle(speed):
	return MoveSpeed(0, 0, speed, 0)

if __name__ == '__main__':
	try:
		gameLogic = GameLogic()
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			if not gameLogic.ballSeen:
				print("X - Ball not seen.")
				gameLogic.speed_pub.publish(rotate(-10))
			else:
				print("X - BALL SEEN")
			rate.sleep()
	except rospy.ROSInterruptException:
		pass
