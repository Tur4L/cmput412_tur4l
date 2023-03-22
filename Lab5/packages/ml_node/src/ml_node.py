#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
import yaml
from sensor_msgs.msg import CompressedImage, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from dt_apriltags import Detector
import rospkg

import tf2_ros
import geometry_msgs.msg
import tags
from std_msgs.msg import String
from duckietown.dtros import DTROS, TopicType, NodeType


HOST_NAME = os.environ["VEHICLE_NAME"]
IGNORE_DISTANCE = 1.0
PUBLISH_IMAGES = False

class MLNode(DTROS):

    def __init__(self, node_name):
        super(MLNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        rospack = rospkg.RosPack()
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.pose_pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/pos_estimate', geometry_msgs.msg.TransformStamped, queue_size=10)
        self.image = None
        self.seq = 0
        self.intrinsic = self.readYamlFile(rospack.get_path('augmented_reality_apriltag') + '/src/camera_intrinsic.yaml')
        self.detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
        self.target_led_color = 'WHITE'
        self.last_detection_counter = 0
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.continue_run = True
        def general_callback(msg):
            if msg.data == 'shutdown':
                rospy.signal_shutdown('received shutdown message')
            elif msg.data == 'stop':
                self.continue_run = False
        rospy.Subscriber('/general', String, general_callback)

        if PUBLISH_IMAGES:
            self.pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/compressed', CompressedImage, queue_size=10)
            # self.info_pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/camera_info', CameraInfo, queue_size=10)
            self.seq = 0

    
    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        compressed_image = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
        self.image = im
    
    def run(self):
        rate = rospy.Rate(5)  # in Hz

        while not rospy.is_shutdown() and self.continue_run:
            if self.image is not None:

                camera_matrix = np.array(self.intrinsic["camera_matrix"]["data"]).reshape(3,3)
                camera_proj_mat = np.concatenate((camera_matrix, np.zeros((3, 1), dtype=np.float32)), axis=1)
                distort_coeff = np.array(self.intrinsic["distortion_coefficients"]["data"]).reshape(5,1)
                fx = camera_matrix[0][0].item()
                fy = camera_matrix[1][1].item()
                cx = camera_matrix[0][2].item()
                cy = camera_matrix[1][2].item()
                tag_size = 0.065  # in meters

                width = self.image.shape[1]
                height = self.image.shape[0]

                newmatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distort_coeff, (width,height), 1, (width,height))
                undistort_im = cv2.undistort(self.image, camera_matrix, distort_coeff, None, newmatrix)
                input_image = cv2.cvtColor(undistort_im, cv2.COLOR_BGR2GRAY)
                detected_tags = self.detector.detect(input_image, estimate_tag_pose=True, camera_params=(fx, fy, cx, cy), tag_size=tag_size)

                dst = undistort_im
                for det in detected_tags:
                    pass
                    # TODO: publish tag ids and detect digits with trained ML model

                if PUBLISH_IMAGES:
                    x,y,w,h = roi 
                    dst = dst[y:y+h, x:x+w]

                    msg = CompressedImage()
                    msg.header.seq = self.seq
                    msg.header.stamp = rospy.Time.now()
                    msg.header.frame_id = f'{HOST_NAME}/camera_optical_frame'
                    msg.format = 'jpeg'
                    ret, buffer = cv2.imencode('.jpg', dst)
                    if not ret:
                        print('failed to encode image!')
                    else:
                        msg.data = np.array(buffer).tostring()
                        self.pub.publish(msg)
                        self.seq += 1

            rate.sleep()

    def readYamlFile(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def onShutdown(self):
        super(MLNode, self).onShutdown()


if __name__ == '__main__':
    ar_node = MLNode('ar_node')
    rospy.spin()
    ar_node.run()


