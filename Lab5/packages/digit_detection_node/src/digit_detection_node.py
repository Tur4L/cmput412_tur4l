#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2
import matplotlib.pyplot as plt
import tensorflow as tf

import rospy
from duckietown.dtros import DTROS, TopicType, NodeType
from sensor_msgs.msg import CompressedImage

HOST_NAME = os.environ["VEHICLE_NAME"]


class Digit_Detection(DTROS):

    def __init__(self,node_name):

        self.sub = rospy.Subscriber(f'/{HOST_NAME}/camera_node/image/compressed', CompressedImage, self.callback)
        self.pub = rospy.Publisher(f'/{HOST_NAME}/april_tag_node/compressed', CompressedImage, queue_size=10)
        self.veh = str(os.environ["VEHICLE_NAME"])

        self.image = None
        self.model = None

        mnist = tf.keras.datasets.mnist
        (self.x_train,self.y_train),(self.x_test,self.y_test) = mnist.load_data()
        self.x_train = tf.keras.utils.normalize(self.x_train,axis=1)
        self.x_test = tf.keras.utils.normalize(self.x_test,axis=1)

    def callback(self, msg):
        # how to decode compressed image
        # reference: http://wiki.ros.org/rospy_tutorials/Tutorials/WritingImagePublisherSubscriber
        compressed_image = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(compressed_image, cv2.IMREAD_COLOR)
        self.image = im

    def train_data(self):

        self.model = tf.keras.models.Sequential()
        self.model.add(tf.keras.layers.Flatten(input_shape=(28,28)))
        self.model.add(tf.keras.layers.Dense(128,activation="relu"))
        self.model.add(tf.keras.layers.Dense(128,activation="relu"))
        self.model.add(tf.keras.layers.Dense(10,activation="softmax"))

        self.model.compile(optimizer='adam',loss='sparse_categorical_crossentropy',metrics=['accuracy'])

        self.model.fit(self.x_train, self.y_train, epochs=10)
        self.model.save("digit_detection.model")

    def test_data(self):
        loss,accuracy = self.model.evaluate(self.x_test,self.y_test)
        print(f"Loss: {loss}")
        print(f"Accuracy: {accuracy}")

if __name__ == '__main__':
    digit_node = Digit_Detection("digit_detection_node")
