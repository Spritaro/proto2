#!/usr/bin/env python
# coding: utf-8

import actionlib
import cv2
import rospy

from copy import deepcopy
from cv_bridge import CvBridge
from proto2_msgs.msg import ObjectDetectionAction, ObjectDetectionGoal, ObjectDetectionResult
from proto2_msgs.msg import Rect
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy


class StrategyForPreliminary(object):
    def __init__(self):
        self.is_color_received = False
        self.is_depth_received = False
        self.is_busy = False
        self.color_msg = Image()
        self.depth_msg = Image()
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        self.pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.client = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)
        rospy.loginfo("waiting for action server")
        self.client.wait_for_server()
        self.goal = ObjectDetectionGoal()

        self.bridge = CvBridge()

        # Realsense D435 camera data
        self.depth_horizontal_fov = 91.2
        self.depth_vertical_fov   = 65.5
        self.color_horizontal_fov = 69.4
        self.color_vertical_fov   = 42.5

        rospy.loginfo("waiting for image")
        while (rospy.is_shutdown() == False and
               self.is_color_received == False and self.is_depth_received == False):
            rospy.sleep(0.1)


    def color_callback(self, msg):
        if self.is_busy is False:
            self.is_color_received = True
            self.color_msg = deepcopy(msg)

    def depth_callback(self, msg):
        if self.is_busy is False:
            self.is_depth_received = True
            self.depth_msg = deepcopy(msg)


    def detect_single_object(self):
        # start object detection
        self.is_busy = True

        self.goal.image = self.color_msg
        self.client.send_goal(self.goal)
        rospy.loginfo("waiting for action")
        self.client.wait_for_result()

        # parse result
        rects = []
        result = self.client.get_result()
        for rect in result.rects:
            if rect.score > 0.5:
                rects.append(rect)

        if len(rects) == 0:
            rospy.loginfo("object not found")
            return None

        # choose closest object
        depth_img = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding="passthrough")
        depth_w = depth_img.shape[1]
        depth_h = depth_img.shape[0]

        closest_rect = Rect()
        closest_dist = 2000.0
        for rect in rects:
            xmid = (rect.xmin + rect.xmax) / 2.0
            ymid = (rect.ymin + rect.ymax) / 2.0

            xmid = depth_w * ((xmid - 0.5) * self.color_horizontal_fov / self.depth_horizontal_fov + 0.5)
            ymid = depth_h * ((ymid - 0.5) * self.color_vertical_fov / self.depth_vertical_fov + 0.5)
            dist = depth_img[int(ymid)][int(xmid)]

            print(xmid, ymid, dist)

            if dist < closest_dist:
                closest_dist = dist
                closest_rect = rect

        rospy.loginfo(closest_rect)

        # end object detection
        self.is_busy = False

        return closest_rect


    def spin(self):
        while not rospy.is_shutdown():
            detected_rect = self.detect_single_object()

            if detected_rect == None:
                rospy.sleep(0.5)
                continue

            # send motion
            joy = Joy()
            joy.axes = [0.0] * 6
            joy.buttons = [0,0,0,0, 0,0,0,0, 0,0, 0,0]
            if detected_rect.class_name == "robot":
                joy.buttons = [0,0,0,0, 0,1,1,0, 0,0, 0,0]
            elif detected_rect.class_name == "robot_down":
                joy.buttons = [1,0,0,0, 0,0,1,0, 0,0, 0,0]
            elif detected_rect.class_name == "referee":
                joy.buttons = [0,0,1,0, 0,0,1,0, 0,0, 0,0]

            rospy.loginfo(joy)
            for i in range(5):
                self.pub.publish(joy)
                rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('strategy_for_preliminary')

    sfp = StrategyForPreliminary()
    sfp.spin()

