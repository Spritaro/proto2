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
from sensor_msgs.msg import Image

class ObjectDetectionServer(object):

    def __init__(self, graph, sess, action_name, model_path, label_path):
        #self.w, self.h = 424, 240
        #self.w, self.h = 212, 120
        self.w, self.h = 160, 120

        # Initialization
        self.init_action(action_name)

        self.init_model(graph, model_path, label_path)
        self.detection_graph = graph
        self.detection_sess = sess

        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/object_detection/image', Image, queue_size=1)

        # Run first inference
        rospy.loginfo("running first inference")
        image_np_expanded = np.zeros((1, self.w, self.h, 3), dtype=np.uint8)
        output_dict = self.run_inference_for_single_image(image_np_expanded, self.detection_graph, self.detection_sess)

        # Start action server
        self.object_detection_server.start()


    def init_action(self, action_name):
        self.object_detection_result = ObjectDetectionResult()
        self.object_detection_server = actionlib.SimpleActionServer(
            action_name,
            ObjectDetectionAction,
            execute_cb = self.execute_object_detection,
            auto_start = False
        )


    def init_model(self, graph, model_path, label_path):
        # Load Tensorflow model into memory
        rospy.loginfo("loading model")
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # Load label map
        rospy.loginfo("loading label map")
        self.category_index = label_map_util.create_category_index_from_labelmap(label_path)


    def execute_object_detection(self, goal):
        rospy.loginfo("start executing object detection")

        # Convert image message to numpy array
        image_np = self.bridge.imgmsg_to_cv2(goal.image, desired_encoding="rgb8")
        image_np = cv2.resize(image_np, (self.w, self.h))
        image_np_expanded = np.expand_dims(image_np, axis=0)

        # Detection
        rospy.loginfo("detecting objects")
        output_dict = self.run_inference_for_single_image(image_np_expanded, self.detection_graph, self.detection_sess)
        rospy.loginfo(output_dict)

        # Visualization of the results of a detection.
        rospy.loginfo("visualizing")
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict['detection_boxes'],
            output_dict['detection_classes'],
            output_dict['detection_scores'],
            self.category_index,
            instance_masks=output_dict.get('detection_masks'),
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=.5)

        image_msg = self.bridge.cv2_to_imgmsg(image_np, encoding='rgb8')
        self.pub.publish(image_msg)

        # Set result
        self.object_detection_result.rects = []
        self.object_detection_server.set_succeeded(self.object_detection_result)
        return


    def run_inference_for_single_image(self, image, graph, sess):
        # Get handles to input and output tensors
        #ops = tf.get_default_graph().get_operations()
        ops = graph.get_operations()
        all_tensor_names = {output.name for op in ops for output in op.outputs}
        tensor_dict = {}
        for key in [
            'num_detections', 'detection_boxes', 'detection_scores',
            'detection_classes', 'detection_masks'
        ]:
            tensor_name = key + ':0'
            if tensor_name in all_tensor_names:
                #tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                tensor_dict[key] = graph.get_tensor_by_name(
                    tensor_name)
        if 'detection_masks' in tensor_dict:
            # The following processing is only for single image
            detection_boxes = tf.squeeze(tensor_dict['detection_boxes'], [0])
            detection_masks = tf.squeeze(tensor_dict['detection_masks'], [0])
            # Reframe is required to translate mask from box coordinates to image coordinates and fit the image size.
            real_num_detection = tf.cast(tensor_dict['num_detections'][0], tf.int32)
            detection_boxes = tf.slice(detection_boxes, [0, 0], [real_num_detection, -1])
            detection_masks = tf.slice(detection_masks, [0, 0, 0], [real_num_detection, -1, -1])
            detection_masks_reframed = utils_ops.reframe_box_masks_to_image_masks(
                detection_masks, detection_boxes, image.shape[1], image.shape[2])
            detection_masks_reframed = tf.cast(
                tf.greater(detection_masks_reframed, 0.5), tf.uint8)
            # Follow the convention by adding back the batch dimension
            tensor_dict['detection_masks'] = tf.expand_dims(
                detection_masks_reframed, 0)
        #image_tensor = tf.get_default_graph().get_tensor_by_name('image_tensor:0')
        image_tensor = graph.get_tensor_by_name('image_tensor:0')

        # Run inference
        output_dict = sess.run(tensor_dict,
                                feed_dict={image_tensor: image})

        # all outputs are float32 numpy arrays, so convert types as appropriate
        output_dict['num_detections'] = int(output_dict['num_detections'][0])
        output_dict['detection_classes'] = output_dict[
            'detection_classes'][0].astype(np.int64)
        output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
        output_dict['detection_scores'] = output_dict['detection_scores'][0]
        if 'detection_masks' in output_dict:
            output_dict['detection_masks'] = output_dict['detection_masks'][0]
        return output_dict


if __name__ == '__main__':
    rospy.init_node('object_detection_server')

    rospy.loginfo("creating graph and session")
    graph = tf.Graph()
    with graph.as_default():
        sess = tf.Session(graph=graph)

    rospack = rospkg.RosPack()
    rospack.list()
    ods = ObjectDetectionServer(
        graph = graph,
        sess = sess,
        action_name = 'object_detection_action',
        model_path = os.path.join(rospack.get_path('proto2_vision'), 'models', 'frozen_inference_graph.pb'),
        label_path = os.path.join(rospack.get_path('proto2_vision'), 'models', 'tf_label_map.pbtxt')
    )

    rospy.loginfo("ready for action")
    rospy.spin()

    sess.close()
