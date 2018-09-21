#! /usr/bin/env python

import rospy
import numpy as np
import cv2
from hardware_package.communication_mainboard import ComportMainboard
from image_processing_package.msg import BallPoint

# class is for 

class GameLogic():

	def __init__(self):
		rospy.init_mode("game_logic_package", anonymous=True) #may be game_logic(?)
		rospy.Subscriber("ballpoint_coordinates", BallPoint, self.callback)

	def callback(self, ballPoint):
		print(str(ballPoint))
		if (ballPoint.x < 380 and ballPoint.x > 260):
			print("- STOP ROBOT! -")

if __name__ == '__main__':
	try:
		gameLogic = GameLogic()
		rate = rospy.Rate(60)
		while not rospy.is_shutdown():
			rate.sleep()
	except rospy.ROSInterruptException:
		pass