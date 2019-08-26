#!/usr/bin/env python
# coding: utf-8

import actionlib
import rospy

from copy import deepcopy
from proto2_msgs.msg import ObjectDetectionAction, ObjectDetectionGoal, ObjectDetectionResult
from sensor_msgs.msg import Image

is_image_received = False
image = Image()

def callback(msg):
    global is_image_received
    global image

    print("received")
    is_image_received = True
    image = deepcopy(msg)

if __name__ == '__main__':
    rospy.init_node('object_detection_client')
    sub = rospy.Subscriber('/camera/color/image_raw', Image, callback)

    client = actionlib.SimpleActionClient('object_detection_action', ObjectDetectionAction)
    rospy.loginfo("waiting for action server")
    client.wait_for_server()
    goal = ObjectDetectionGoal()

    while (rospy.is_shutdown() == False) and (is_image_received == False):
        print(rospy.is_shutdown())
        print(is_image_received)
        rospy.loginfo("waiting for image")
        rospy.sleep(0.1)

    goal.image = image
    client.send_goal(goal)
    rospy.loginfo("waiting for action")
    client.wait_for_result()

    print(client.get_result())
    rospy.loginfo("done")

