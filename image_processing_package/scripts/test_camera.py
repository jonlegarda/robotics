#!/usr/bin/env python
import rospy
import cv2
import pyrealsense2 as rs
from collections import deque
import numpy as np
from geometry_msgs.msg import Point
# own packages
from std_msgs.msg import String
from image_processing_package.image_colors import Detector
from image_processing_package.msg import BallPoint
from image_processing_package.msg import BasketPoint

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
        self.publisher_basket = rospy.Publisher("basket_coordinates", BasketPoint, queue_size=10)
        # self.green_lower_bond = [40, 80, 50]
        # self.green_upper_bond = [90,255, 125]

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 360, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 60)
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
        #self.yuv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)
        self.hsv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2HSV)

    def detect_ball(self):
        green_lower_bound = np.array([42, 61, 52])
        green_upper_bound = np.array([83, 254, 206])
        mask = cv2.inRange(self.hsv, green_lower_bound, green_upper_bound)

        # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # cv2.imshow('image', mask)
        # cv2.waitKey(1)

        # cv2.circle(self.hsv,(center_x,center_y), 10, (0,0,255), -1)
        # cv2.circle(self.hsv,(480,270), 10, (0,255,0), -1)
        # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        # cv2.imshow('image',self.hsv)
        # cv2.waitKey(10)


        im2, contours, hierarchie = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        biggest_ball_rect = None
        biggest_ball_size = 0
        for contour in contours:
            contour_size = cv2.contourArea(contour)
            if contour_size > biggest_ball_size:
                biggest_ball_rect = cv2.boundingRect(contour)
                biggest_ball_size = contour_size
        if biggest_ball_rect is not None:
            x, y, w, h = biggest_ball_rect
            center_x = x + w / 2
            center_y = y + h / 2
            depth = self.depth_image[center_y][center_x]
            rospy.loginfo("Ball: {} {}".format(center_x, center_y,depth))
            depth = self.depth_image[center_y][center_x]
            return Point(center_x, center_y, depth)


def it_is_ball(x,y,w,h,contour_area):
    if (x<0 or y<0 or w<0 or h<0 or contour_area<10):
        return False
    #ball_shape = round((float(min(w,h))/max(w,h))*100, 2) if (w>0 and h>0) else 0.0
    #if (ball_shape < 60.0 or contour_area > 3000):
    #print("ball shape {}".format(ball_shape))
    #if (ball_shape < 60.0):
    #    return False
    return True

if __name__ == '__main__':
    try:
        camera_proc = RealsenseProcessing()
        camera_proc.run()
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            camera_proc.get_frame()
            camera_proc.detect_ball()
            test = np.array(camera_proc.hsv)
            detector = Detector('/home/superuser/catkin_ws/src/image_processing_package/scripts/configuration/ball_color_parameters.txt', 'ball_color_parameters')
            # code for calibrating basket:
            detector_basket = Detector('/home/superuser/catkin_ws/src/image_processing_package/scripts/configuration/basket_color_parameters.txt', 'basket_color_parameters')
            cap = None
            res, mask, x, y, contour_area, w, h = detector.detect(camera_proc.hsv, camera_proc.regular_image)
            # code for calibrating basket:
            res_basket, mask_basket, x_basket, y_basket, contour_area_basket, w_basket, h_basket = detector_basket.detect(camera_proc.hsv, camera_proc.regular_image)
            PRINT_X_VALUES = "BALL  X=" + str(x) + " Y=" + str(y);
            print(PRINT_X_VALUES)

            # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
            # cv2.imshow('image', mask)
            # cv2.waitKey(1)

            # BALL DETECTION.
            if (it_is_ball(x,y,w,h,contour_area)):
                camera_proc.publisher.publish(BallPoint(x,y,0))
                # if (427<x<511):
                #     print(PRINT_SENTENCE + "BALL seen. YES centered.")
                # else:
                #     print(PRINT_SENTENCE + "BALL seen. NO centered.")
                # if (400<y<460):
                #     print(PRINT_SENTENCE + "BALL is near-STOP!")
                # else:
                #     print(PRINT_SENTENCE + "BALL is far-GO!")
            else:
                print(PRINT_SENTENCE + "BALL NOT seen.")
                camera_proc.publisher.publish(BallPoint(-1,-1,0))
            # BASKET DETECTION.
            camera_proc.publisher_basket.publish(BasketPoint(x_basket, y_basket, 0))
            print(PRINT_SENTENCE + "BASKET x=" + str(x_basket) + " y=" + str(y_basket) )
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
