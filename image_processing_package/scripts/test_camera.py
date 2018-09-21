#!/usr/bin/env python

import cPickle as pickle
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import diploaf_robot.config_client as config_client
import cv2
import time
import image_processing.ps3eye_conf as conf
from image_processing.msg import RawObjectPos
from sensor_msgs.msg import CompressedImage
from diploaf_robot.msg import KeyValue
from geometry_msgs.msg import Point
from diploaf_utils.key_listener import KeyListener
from image_processing.msg import RawObjectPos
from image_processing.msg import RawObjectPositions
import os
import errno
import pyrealsense2 as rs
import numpy as np


MAGENTA = 0
BLUE = 1


class RealsenseProcessing(config_client.ConfigClient, KeyListener):
    def __init__(self):

        # init
        config_client.ConfigClient.__init__(self)
        KeyListener.__init__(self)
        rospy.init_node("realsense_processing")
        self.init_config_client()
        self.running = False
        self.bridge = CvBridge()
        self.brush_size = 1
        self.noise = 1
        self.chosen_object = conf.COLORS['balls']
        self.color_name = conf.COLORS.keys()[conf.COLORS.values().index(self.chosen_object)]
        self.current_message = ""
        self.goal = self.config['target_color']
        self.goals = [RawObjectPos(), RawObjectPos()]
        self.frame_balls = []
        self.fps = 0
        self.frame_count = 0
        self.start_time = time.time()
        self.show_fps_every = 1
        try:
            with open(conf.COLORS_CONF_PATH, 'rb') as fh:
                self.colors_lookup = pickle.load(fh)
        except:
            self.colors_lookup = np.zeros(0x1000000, dtype=np.uint8)
        rospy.Timer(rospy.Duration(1), self.publish_monitoring_info)

        # frames init
        self.yuv = np.zeros((conf.HEIGHT, conf.WIDTH, 3), dtype=np.uint8)
        self.fragmented_frame = np.zeros((conf.HEIGHT, conf.WIDTH, 3), dtype=np.uint8)
        self.t_debug = np.zeros((conf.HEIGHT, conf.WIDTH, 3), dtype=np.uint8)
        self.regular_image = np.zeros((conf.HEIGHT, conf.WIDTH, 3), dtype=np.uint8)
        self.depth_image = None
        # conf
        self.is_calibrate = rospy.get_param("/realsense_processing/realsense_calibrate")
        self.is_monitoring = self.config['monitoring']
        self.min_ball_area = rospy.get_param("/front_cam_min_ball_area")

        # publishers
        self.debug_img_pub = rospy.Publisher("image_processing/realsense_debug/compressed", CompressedImage,
                                             queue_size=10)
        self.regular_img_pub = rospy.Publisher("image_processing/realsense/compressed", CompressedImage,
                                               queue_size=10)
        self.fragmented_img_pub = rospy.Publisher("image_processing/realsense_frag/compressed", CompressedImage,
                                                  queue_size=10)
        self.gui_publisher = rospy.Publisher("rqt_diploaf_dashboard/status", KeyValue, queue_size=10)
        self.obj_publisher = rospy.Publisher("image_processing/ps3eye_raw_object_positions", RawObjectPositions,
                                             queue_size=10)
        if self.is_calibrate:
            rospy.Subscriber("/image_processing/realsense/compressed_mouse_left", Point,
                             self.mouse_click_callback)
            rospy.Subscriber("/image_processing/realsense_frag/compressed_mouse_left", Point,
                             self.mouse_click_callback)
            rospy.Subscriber("/image_processing/realsense_debug/compressed_mouse_left", Point,
                             self.mouse_click_callback)

    def key_event_callback(self, key_event_msg):
        if not self.is_calibrate:
            return

        if key_event_msg.pressed:
            return

        message = ""

        if key_event_msg.char == "Up":
            self.brush_size = self.brush_size + 1

        if key_event_msg.char == "Down":
            self.brush_size = max(self.brush_size - 1, 0)

        if key_event_msg.char == "s":
            with self.open_or_create(conf.COLORS_CONF_PATH) as fh:
                pickle.dump(self.colors_lookup, fh, -1)
            message += "Color calibration saved; "

        if key_event_msg.char == "e":
            self.colors_lookup[self.colors_lookup == self.chosen_object] = 0
            message += "Erased color {}; ".format(str(self.color_name))

        if key_event_msg.char == "r":
            self.colors_lookup[:] = 0
            message += "Reset all colors"

        try:
            nr = int(key_event_msg.char)
            self.chosen_object = nr
            self.color_name = conf.COLORS.keys()[conf.COLORS.values().index(self.chosen_object)]
        except ValueError:
            pass

        self.current_message = message + "Selected color: {}; Brush size: {}".format(self.color_name, self.brush_size)
        rospy.loginfo(self.current_message)

    def open_or_create(self, filename):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        rospy.loginfo("current dir path:{}".format(dir_path))
        if not os.path.exists(os.path.dirname(filename)):
            try:
                rospy.loginfo("creating new location")
                os.makedirs(os.path.dirname(filename))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise
        rospy.loginfo("opening path")
        return open(filename, 'wb')

    def publish_monitoring_info(self, args):
        if self.is_calibrate:
            msg = KeyValue()
            msg.MsgType = "realsense_calib"
            msg.MsgKey = "realsense_calib"
            msg.MsgValue = self.current_message
            self.gui_publisher.publish(msg)

        if self.config['monitoring']:
            msg = KeyValue()
            msg.MsgType = "fps"
            msg.MsgKey = "realsense"
            msg.MsgValue = str(self.fps)
            self.gui_publisher.publish(msg)
            rospy.loginfo("Realsense fps: {}".format(self.fps))
        else:
            self.fps = 0

    def mouse_click_callback(self, point):
        self.change_color(int(point.x), int(point.y))
        rospy.loginfo("Point: {}".format(point))

    def run(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, conf.DEPTH_WIDTH, conf.DEPTH_HEIGHT, rs.format.z16, 60)
        config.enable_stream(rs.stream.color, conf.WIDTH, conf.HEIGHT, rs.format.bgr8, 60)
        self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def spin_once(self):
        self.get_frame()

        if self.yuv is not None:
            yuv_uint32 = self.yuv.astype('uint32')
            fragmented = self.colors_lookup[
                yuv_uint32[:, :, 0] + yuv_uint32[:, :, 1] * 0x100 + yuv_uint32[:, :, 2] * 0x10000]

            if self.config['debug_mode']:
                frame = np.zeros(yuv_uint32.shape)
                frame[fragmented == conf.COLORS['balls']] = np.array([0, 0, 255], dtype=np.uint8)
                frame[fragmented == conf.COLORS['target_magenta']] = np.array([0, 255, 255], dtype=np.uint8)
                frame[fragmented == conf.COLORS['target_blue']] = np.array([255, 0, 0], dtype=np.uint8)
                frame[fragmented == conf.COLORS['field']] = np.array([0, 255, 0], dtype=np.uint8)
                frame[fragmented == conf.COLORS['white']] = np.array([255, 255, 255], dtype=np.uint8)
                frame[fragmented == conf.COLORS['black']] = np.array([255, 255, 0], dtype=np.uint8)
                self.fragmented_frame = frame

            balls_mask = np.zeros((conf.HEIGHT, conf.WIDTH, 1), dtype=np.uint8)
            balls_mask[fragmented == conf.COLORS['balls']] = 255

            magenta_goal_mask = np.zeros((conf.HEIGHT, conf.WIDTH, 1), dtype=np.uint8)
            magenta_goal_mask[fragmented == conf.COLORS['target_magenta']] = 255

            blue_goal_mask = np.zeros((conf.HEIGHT, conf.WIDTH, 1), dtype=np.uint8)
            blue_goal_mask[fragmented == conf.COLORS['target_blue']] = 255

            self.analyze_balls(balls_mask)
            self.analyze_goals(magenta_goal_mask, MAGENTA)
            self.analyze_goals(blue_goal_mask, BLUE)

            raw_object_positions = RawObjectPositions()
            raw_object_positions.OwnGatePos = self.goals[1 - self.goal]
            raw_object_positions.OpponentGatePos = self.goals[self.goal]
            raw_object_positions.Balls = self.frame_balls
            self.obj_publisher.publish(raw_object_positions)

        self.publish_debug_img()
        self.publish_calibration_images()

        if self.config['monitoring']:
            self.frame_count += 1
            if (time.time() - self.start_time) > self.show_fps_every:
                self.fps = self.frame_count / (time.time() - self.start_time)
                self.frame_count = 0
                self.start_time = time.time()

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            return

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.regular_image = np.asanyarray(color_frame.get_data())
        self.yuv = cv2.cvtColor(self.regular_image, cv2.COLOR_BGR2YUV)

    def analyze_balls(self, t_ball):
        # if not self.is_wide_angle_camera:
        #     erode_kernel = np.ones((4, 4), np.uint8)
        #     t_ball = cv2.erode(t_ball, erode_kernel, iterations=1)
        #     dilate_kernel = np.ones((10, 10), np.uint8)
        #     t_ball = cv2.dilate(t_ball, dilate_kernel, iterations=1)

        img, contours, hierarchy = cv2.findContours(t_ball, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.frame_balls = []
        bounding_rectangles = []

        for contour in contours:
            s = cv2.contourArea(contour)

            if s < self.min_ball_area:
                continue

            rect = cv2.boundingRect(contour)
            x_img, y_img, w_img, h_img = rect
            radius_expand=0
            x_start = np.max([0, x_img - w_img - radius_expand])
            x_end = np.min([x_img + w_img + radius_expand, conf.WIDTH])
            y_start = np.max([0, y_img - h_img - radius_expand])
            y_end = np.min([y_img + h_img + radius_expand, conf.HEIGHT])
            depth_points = self.depth_image[y_start:y_end, x_start:x_end]

            rospy.loginfo("ball distance: {}".format(depth_points))
            #TODO get median from depth_points to receive actual depth of the object

            x, y, w, h = self.get_corrected_bounding_rect(rect)

            bounding_rectangles.append(rect)

            ball = self.build_object_pos(w, h, x, y, s)
            self.frame_balls.append(ball)

            if self.config['debug_mode']:
                x_debug = x_img + w_img / 2
                y_debug = y_img + h_img / 2
                cv2.circle(self.t_debug, (x_debug, y_debug), max(1, w_img / 2), [0, 0, 255], 2)
                y_top = y_img + h_img
                y_bottom = max(y_debug - h_img, 0)
                cv2.circle(self.t_debug, (x_debug, y_top), 5, [255, 0, 255], 2)
                cv2.circle(self.t_debug, (x_debug, y_bottom), 5, [0, 255, 255], 2)
                (rows, cols, d) = self.t_debug.shape
                cv2.circle(self.t_debug, (cols / 2, rows / 2), 4, [0, 255, 255], 4)

    def analyze_goals(self, t_goal, goal_nr):
        erode_kernel = np.ones((3, 3), np.uint8)
        t_goal = cv2.erode(t_goal, erode_kernel, iterations=1)
        dilate_kernel = np.ones((5, 5), np.uint8)
        t_goal = cv2.dilate(t_goal, dilate_kernel, iterations=1)

        img, contours, hierarchy = cv2.findContours(t_goal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.goals[goal_nr] = RawObjectPos()
        self.goals[goal_nr].x = -1
        s_max = 0
        original_rect = None
        for contour in contours:
            s = cv2.contourArea(contour)
            if s < 150:  # too small area
                continue
            if s > s_max:
                original_rect = cv2.boundingRect(contour)
                x, y, w, h = self.get_corrected_bounding_rect(original_rect)
                s_max = s
                goal = self.build_object_pos(w, h, x, y, s)
                self.goals[goal_nr] = goal

        self.add_debug_info(original_rect, goal_nr, contours)

    def add_debug_info(self, rect, goal_nr, contours):
        if self.config['debug_mode'] and rect is not None:
            x_d, y_d, w_d, h_d = rect
            top_left = (x_d, y_d)
            bottom_right = (x_d + (w_d), y_d + (h_d))
            color = [255, 0, 0] if goal_nr == BLUE else [0, 255, 255]
            cv2.rectangle(self.t_debug, top_left, bottom_right, color, 2)
            cv2.drawContours(self.t_debug, contours, -1, [0, 0, 0])

    @staticmethod
    def build_object_pos(width, height, x, y, area):
        object_pos = RawObjectPos()
        object_pos.width = width
        object_pos.height = height
        object_pos.x = x
        object_pos.y = y
        object_pos.y_top = y + height
        object_pos.y_bottom = max(y - height, 0)
        object_pos.area = 0

        return object_pos

    def get_corrected_bounding_rect(self, rect):
        # The camera is turned 90 degrees
        y, x, h, w = rect
        x = x + w / 2
        y = y + h / 2
        y = conf.WIDTH - y

        return x, y, w, h

    def change_color(self, mouse_x, mouse_y):
        ob = self.yuv[max(0, mouse_y - self.brush_size):min(conf.HEIGHT, mouse_y + self.brush_size + 1),
             max(0, mouse_x - self.brush_size):min(conf.WIDTH, mouse_x + self.brush_size + 1), :].reshape(
            (-1, 3)).astype('int32')
        noises = xrange(-self.noise, self.noise + 1)
        for y in noises:
            for u in noises:
                for v in noises:
                    self.colors_lookup[
                        ((ob[:, 0] + y) + (ob[:, 1] + u) * 0x100 + (ob[:, 2] + v) * 0x10000).clip(0,
                                                                                                  0xffffff)] = self.chosen_object

    def publish_calibration_images(self):
        if self.is_calibrate or self.config['debug_mode']:
            self.publish_img(self.regular_image, self.regular_img_pub)
            self.publish_img(self.fragmented_frame, self.fragmented_img_pub)

    def publish_debug_img(self):
        if self.config['debug_mode'] and self.t_debug is not None and len(self.t_debug) > 0:
            self.publish_img(self.t_debug, self.debug_img_pub)
            self.t_debug = np.zeros((conf.WIDTH, conf.HEIGHT, 3), dtype=np.uint8)
            self.t_debug = self.regular_image

    def publish_img(self, image, publisher):
        if image is None:
            image = np.zeros((512, 512, 3), np.uint8)
            cv2.putText(image, 'No image', (10, 500), cv2.FONT_HERSHEY_SIMPLEX, 4,
                        (255, 255, 255), 2, cv2.LINE_AA)

        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        publisher.publish(msg)


if __name__ == '__main__':
    try:
        camera = RealsenseProcessing()
        camera.run()
        rate = rospy.Rate(60)

        while not rospy.is_shutdown():
            camera.spin_once()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
