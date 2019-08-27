#!/usr/bin/env python
# coding: utf-8

import actionlib
import rospy

from copy import deepcopy
from proto2_msgs.msg import ObjectDetectionAction, ObjectDetectionGoal, ObjectDetectionResult
from proto2_msgs.msg import Rect
from sensor_msgs.msg import Image
from sensor_msgs.msg import Joy


class StrategyForPreliminary(object):
    def __init__(self):
        self.is_image_received = False
        self.is_busy = False
        self.image = Image()
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pub = rospy.Publisher('/joy', Joy, queue_size=1)

        self.client = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)
        rospy.loginfo("waiting for action server")
        self.client.wait_for_server()
        self.goal = ObjectDetectionGoal()

        rospy.loginfo("waiting for image")
        while (rospy.is_shutdown() == False) and (self.is_image_received == False):
            rospy.sleep(0.1)


    def image_callback(self, msg):
        if self.is_busy is False:
            self.is_image_received = True
            self.image = deepcopy(msg)


    def spin(self):
        while not rospy.is_shutdown():
            # start object detection
            self.is_busy = True

            self.goal.image = self.image
            self.client.send_goal(self.goal)
            rospy.loginfo("waiting for action")
            self.client.wait_for_result()

            # end object detection
            self.is_busy = False

            # parse result
            rects = []
            result = self.client.get_result()
            for rect in result.rects:
                if rect.score > 0.5:
                    rects.append(rect)
            
            highest_score_rect = Rect()
            highest_score_rect.score = 0.0
            for rect in rects:
                if rect.score > highest_score_rect.score:
                    highest_score_rect = rect

            rospy.loginfo(highest_score_rect)

            # send motion
            joy = Joy()
            joy.axes = [0.0] * 6
            joy.buttons = [0,0,0,0, 0,0,0,0, 0,0, 0,0]
            if highest_score_rect.class_name == "robot":
                joy.buttons = [0,0,0,0, 0,1,1,0, 0,0, 0,0]
            elif highest_score_rect.class_name == "robot_down":
                joy.buttons = [1,0,0,0, 0,0,1,0, 0,0, 0,0]
            elif highest_score_rect.class_name == "referee":
                joy.buttons = [0,0,1,0, 0,0,1,0, 0,0, 0,0]

            rospy.loginfo(joy)
            for i in range(5):
                self.pub.publish(joy)
                rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('strategy_for_preliminary')

    sfp = StrategyForPreliminary()
    sfp.spin()




    print()


