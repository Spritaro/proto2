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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from strategy_for_preliminary import StrategyForPreliminary


class StrategyForBattle(StrategyForPreliminary):
    def __init__(self):
        super(StrategyForBattle, self).__init__()

        self.imu_sub = rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)
        self.imu_msg = Imu()

        self.head_angle = 60.0


    def imu_callback(self, msg):
        if self.is_busy is False:
            self.imu_msg = deepcopy(msg)


    def spin(self):
        while not rospy.is_shutdown():
            ### check front ###
            rospy.loginfo("checking front")
            self.request_next_detection()
            if self.check_imu_and_vision(0.0):
                continue

            ### check right ###
            rospy.loginfo("checking right")
            self.request_head_right()
            if self.check_imu_and_vision(-self.head_angle):
                continue

            ### check left
            rospy.loginfo("checking left")
            self.request_head_left()
            if self.check_imu_and_vision(self.head_angle):
                continue

            ### turn around if no enemy found
            self.request_turn()


    def check_imu_and_vision(self, head_angle_tmp):
        ### check IMU ###
        rospy.loginfo(self.imu_msg)
        if self.imu_msg.linear_acceleration.z > 5.0:
            self.request_getup_front()
            return True

        if self.imu_msg.linear_acceleration.z < -5.0:
            self.request_getup_back()
            return True

        ### check vision ###
        detected_rect, detected_dist = self.detect_single_object()
        if detected_rect != None:

            if detected_rect.class_name == "robot":
                detected_angle = head_angle_tmp - self.color_horizontal_fov * ((detected_rect.xmin+detected_rect.xmax)/2.0 - 0.5)
                self.request_attack(detected_angle, detected_dist)
                return True

            if detected_rect.class_name == "robot_down":
                self.request_next_detection()
                return True

            # if detected_rect.class_name == "referee":
            #     self.request_next_detection()
            #     return True
        
        return False


    def request_getup_front(self):
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,1, 0,0,1,0, 0,0, 0,0], 10)

    def request_getup_back(self):
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,1,0,0, 0,0,1,0, 0,0, 0,0], 10)


    def request_next_detection(self):
        # look front
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,0,0,0, 0,0, 0,0], 10)

    def request_head_right(self):
        # look right
        self.send_joy([-1.,0., 0.,0., 0.,0.], [0,0,0,0, 0,0,1,0, 0,0, 0,0], 10)

    def request_head_left(self):
        # look left
        self.send_joy([1.,0., 0.,0., 0.,0.], [0,0,0,0, 0,0,1,0, 0,0, 0,0], 10)


    def request_attack(self, angle, dist):
        # turn to enemy direction
        if angle > 90:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 1,0,0,0, 0,0, 0,0], 30)
        elif angle > 60:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 1,0,0,0, 0,0, 0,0], 20)
        elif angle > 30:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 1,0,0,0, 0,0, 0,0], 10)
        elif angle > 10:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 1,0,0,0, 0,0, 0,0], 5)
        elif angle < -90:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,1,0,0, 0,0, 0,0], 30)
        elif angle < -60:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,1,0,0, 0,0, 0,0], 20)
        elif angle < -30:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,1,0,0, 0,0, 0,0], 10)
        elif angle < -10:
            self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,1,0,0, 0,0, 0,0], 5)
        
        # move to enemy position
        if dist == 0:
            dist = 101
        if dist > 1000:
            self.send_joy([0.,1., 0.,0., 0.,0.], [0,0,0,0, 0,0,0,0, 0,0, 0,0], 40)
        elif dist > 500:
            self.send_joy([0.,1., 0.,0., 0.,0.], [0,0,0,0, 0,0,0,0, 0,0, 0,0], 30)
        elif dist > 100:
            self.send_joy([0.,1., 0.,0., 0.,0.], [0,0,0,0, 0,0,0,0, 0,0, 0,0], 20)
        
        # wait
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,0,0,0, 0,0, 0,0], 20)

        # attack
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,1, 0,0,0,0, 0,0, 0,0], 5)


    def request_turn(self):
        self.send_joy([0.,0., 0.,0., 0.,0.], [0,0,0,0, 0,1,0,0, 0,0, 0,0], 10)

    # publish joy message for a specified period of time in [0.1s]
    def send_joy(self, axes, buttons, time):
        joy = Joy()
        joy.axes = axes
        joy.buttons = buttons

        rospy.loginfo(joy)
        for i in range(time):
            self.pub.publish(joy)
            rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('strategy_for_preliminary')

    sfp = StrategyForBattle()
    sfp.spin()

