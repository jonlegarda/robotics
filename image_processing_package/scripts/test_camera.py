#!/usr/bin/env python
import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
# own packages
from std_msgs.msg import String
from image_processing_package.image_colors import Detector
from image_processing_package.msg import BallPoint

PRINT_SENTENCE = "image_processing_package -> test_camera : "

class RealsenseProcessing():
    def __init__(self):
        rospy.init_node("realsense_processing", anonymous=True)
        self.pipeline = None
        self.align = None
        self.depth_image = None
        self.regular_image = None
        self.yuv = None
        self.hsv = None
	self.publisher = rospy.Publisher("ball_coordinates", BallPoint, queue_size=10)

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 60)
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()  # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()
        if not aligned_depth_frame or not color_frame:
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.regular_image = np.asanyarray(color_frame.get_data())
        self.yuv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)
        self.hsv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2HSV)

def it_is_ball(x,y,w,h,contour_area):
    if (x<0 or y<0 or w<0 or h<0 or contour_area<0):
	return False
    ball_shape = round((float(min(w,h))/max(w,h))*100, 2) if (w>0 and h>0) else 0.0
    if (ball_shape < 60.0 or contour_area > 3000):
	return False
    return True


if __name__ == '__main__':
    try:
        camera_proc = RealsenseProcessing()
        camera_proc.run()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            camera_proc.get_frame()
            test = np.array(camera_proc.hsv)
	    detector = Detector('/home/superuser/catkin_ws/src/image_processing_package/scripts/configuration/ball_color_parameters.txt', 'ball_color_parameters')
	    cap = None
	    res, mask, x, y, contour_area, w, h = detector.detect(camera_proc.hsv, camera_proc.regular_image)
	    print("x coordinate - cx:", x)
	    if (it_is_ball(x,y,w,h,contour_area)):
            	camera_proc.publisher.publish(BallPoint(x,y,0))
            	if (270<x<370):
                	print(PRINT_SENTENCE + "ball seen. Stop robot!")
            	else:
                	print(PRINT_SENTENCE + "ball seen. No centered.")
            else:
		print(PRINT_SENTENCE + "ball NOT seen.")
	    	camera_proc.publisher.publish(BallPoint(-1,-1,0))
	    rate.sleep()
    except rospy.ROSInterruptException:
        pass
