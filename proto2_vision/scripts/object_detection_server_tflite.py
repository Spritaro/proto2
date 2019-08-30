#!/usr/bin/env python
# coding: utf-8

import actionlib
import cv2
import numpy as np
import os
import rospkg
import rospy
import sys
import tensorflow as tf

sys.path.append("/usr/src/tensorflow/models/research")
print(sys.path)

from cv_bridge import CvBridge
from object_detection.utils import ops as utils_ops
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from proto2_msgs.msg import ObjectDetectionAction, ObjectDetectionFeedback, ObjectDetectionResult
from proto2_msgs.msg import Rect
from sensor_msgs.msg import Image

class ObjectDetectionServer(object):

    def __init__(self, action_name, model_path, label_path):
        #self.w, self.h = 424, 240
        #self.w, self.h = 212, 120
        # self.w, self.h = 160, 120
        self.w, self.h = 300, 300

        # Initialization
        self.init_action(action_name)

        self.init_model(model_path, label_path)

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/object_detection/image', Image, queue_size=1)

        # Run first inference
        rospy.loginfo("running first inference")
        image_np_expanded = np.zeros((1, self.w, self.h, 3), dtype=np.float32)
        self.interpreter.set_tensor(self.input_details[0]['index'], image_np_expanded)
        self.interpreter.invoke()

        # Start action server
        self.object_detection_server.start()


    def init_action(self, action_name):
        self.object_detection_server = actionlib.SimpleActionServer(
            action_name,
            ObjectDetectionAction,
            execute_cb = self.execute_object_detection,
            auto_start = False
        )


    def init_model(self, model_path, label_path):
        # Load Tensorflow model into memory
        rospy.loginfo("loading model")
        self.interpreter = tf.contrib.lite.Interpreter(model_path=model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        rospy.loginfo(self.input_details)
        rospy.loginfo(self.output_details)

        # Load label map
        rospy.loginfo("loading label map")
        self.category_index = label_map_util.create_category_index_from_labelmap(label_path)


    def execute_object_detection(self, goal):
        rospy.loginfo("start executing object detection")

        # Convert image message to numpy array
        image_np = self.bridge.imgmsg_to_cv2(goal.image, desired_encoding="rgb8")
        image_np = cv2.resize(image_np, (self.w, self.h))
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_np_expanded = image_np_expanded.astype('float32')
        image_np_expanded = (2.0 / 255.0) * image_np_expanded - 1.0

        # Detection
        rospy.loginfo("detecting objects")
        self.interpreter.set_tensor(self.input_details[0]['index'], image_np_expanded)
        self.interpreter.invoke()

        # all outputs are float32 numpy arrays, so convert types as appropriate
        detection_boxes   = self.interpreter.get_tensor(self.output_details[0]['index'])[0]
        detection_classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0].astype(np.int64)
        detection_scores  = self.interpreter.get_tensor(self.output_details[2]['index'])[0]
        num_detections    = int(self.interpreter.get_tensor(self.output_details[3]['index']))

        for i in range(num_detections):
            detection_classes[i] += 1

        # Visualization of the results of a detection.
        rospy.loginfo("visualizing")
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            detection_boxes,
            detection_classes,
            detection_scores,
            self.category_index,
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=.5)

        image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding='rgb8')
        self.pub.publish(image_msg)

        # Set result
        object_detection_result = ObjectDetectionResult()
        object_detection_result.rects = []
        for i in range(num_detections):
            if detection_classes[i] not in self.category_index:
                continue
            rect = Rect()
            rect.class_id = detection_classes[i]
            rect.class_name = self.category_index[detection_classes[i]]['name']
            rect.score = detection_scores[i]
            rect.ymin = detection_boxes[i][0]
            rect.xmin = detection_boxes[i][1]
            rect.ymax = detection_boxes[i][2]
            rect.xmax = detection_boxes[i][3]
            object_detection_result.rects.append(rect)
        self.object_detection_server.set_succeeded(object_detection_result)
        return


if __name__ == '__main__':
    rospy.init_node('object_detection_server')

    rospack = rospkg.RosPack()
    rospack.list()
    ods = ObjectDetectionServer(
        action_name = 'object_detection_action',
        model_path = os.path.join(rospack.get_path('proto2_vision'), 'models', 'detect.tflite'),
        label_path = os.path.join(rospack.get_path('proto2_vision'), 'models', 'tf_label_map.pbtxt')
    )

    rospy.loginfo("ready for action")
    rospy.spin()

