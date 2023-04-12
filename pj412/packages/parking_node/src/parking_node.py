#!/usr/bin/env python3
import numpy as np
import os
import math
import cv2

import rospy
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Int32, String, Float32
from duckietown.dtros import DTROS, TopicType, NodeType
from duckietown_msgs.msg import AprilTagDetectionArray, Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from geometry_msgs.msg import Transform, Vector3, Quaternion

HOST_NAME = os.environ["VEHICLE_NAME"]
PARKING_1 = 207
PARKING_2 = 226
PARKING_3 = 228
PARKING_4 = 75

class Parking(DTROS):

    def __init__(self, node_name):
        super(Parking, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        self.PARKING_SLOT = int(rospy.get_param("~parking_slot"))


        #Static Parameters
        self._radius = 0.0318
        self._robot_width = 0.1

        #Subscibers
        self.sub = rospy.Subscriber(f'/{HOST_NAME}/apriltag_detector_node/detections', AprilTagDetectionArray,self.callback)
        self.sub_left_wheel = rospy.Subscriber(f'/{HOST_NAME}/left_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback,callback_args="left")
        self.sub_right_wheel = rospy.Subscriber(f'/{HOST_NAME}/right_wheel_encoder_node/tick',WheelEncoderStamped,self.wheel_callback,callback_args="right")

        #Publishers
        self.pub = rospy.Publisher(f"/{HOST_NAME}/wheels_driver_node/wheels_cmd", WheelsCmdStamped,queue_size=1)
        self.pub_cmd = rospy.Publisher(f"/{HOST_NAME}/wheels_driver_node/wheels_cmd_executed", WheelsCmdStamped, queue_size=1)
        self.genreal_shutdown = rospy.Publisher('/general', String, queue_size = 1)
        
        #Encoder variables
        self.left_tick = None
        self.right_tick = None

        self.delta_left = 0
        self.delta_right = 0

        #Move and Turning variables
        self.left_distance = 0
        self.right_distance = 0
        self.angle = 0

        #Apriltag variables
        self.tr_x = None
        self.tr_z = None

        self.tr_x_helper = None
        self.tr_z_helper = None
        
        #Parking variables
        self.parking_slot = None
        self.helper_slot = None

    def assign_ids(self):
        if self.PARKING_SLOT == 1:
            self.parking_slot_id = PARKING_1
            self.helper_slot_id = PARKING_3

        elif self.PARKING_SLOT == 2:
            self.parking_slot_id = PARKING_2
            self.helper_slot_id = PARKING_4

        elif self.PARKING_SLOT == 3:
            self.parking_slot_id = PARKING_3
            self.helper_slot_id = PARKING_1

        elif self.PARKING_SLOT == 4:
            self.parking_slot_id = PARKING_4
            self.helper_slot_id = PARKING_1

    def callback(self,msg):
        self.assign_ids()
        for i in msg.detections:
            if i.tag_id == self.parking_slot_id:
                self.tr_x = i.transform.translation.x
                self.tr_z = i.transform.translation.z
            elif i.tag_id == self.helper_slot_id:
                self.tr_x_helper = i.transform.translation.x
                self.tr_z_helper = i.transform.translation.z

    def wheel_callback(self,msg,wheel):
        if wheel == "left":
            if self.left_tick == None:
                self.left_tick = abs(msg.data)
            orig_tick = self.left_tick

        elif wheel == "right":
            if self.right_tick == None:
                self.right_tick = abs(msg.data)
            orig_tick = self.right_tick

        delta = 2 * math.pi * self._radius*(abs(msg.data) - orig_tick)/ msg.resolution
        
        if wheel == "left":
            self.left_tick = abs(msg.data)
            self.delta_left = delta
            self.left_distance += abs(delta)

        else:
            self.right_tick = abs(msg.data)
            self.delta_right = delta
            self.right_distance += abs(delta)

    def reset_variables(self):
        self.angle = 0
        self.left_distance = 0
        self.right_distance = 0
    
    def stop(self):
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = 0
        msg.vel_right = 0
        self.pub.publish(msg)
        self.pub_cmd.publish(msg)

    def turn(self,direction,angle=(math.pi),v1=0.5,v2=0.5):
        self.reset_variables()

        if direction == "left":
            v1 *= -1

        else:
            v2 *= -1
        
        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v1
        msg.vel_right = v2

        while self.angle < angle:
            self.pub.publish(msg)
            self.angle += (self.right_distance + self.left_distance)/(self._robot_width)
            rospy.sleep(0.1)

        self.stop()
        rospy.sleep(0.2)

    def move(self,distance,v1=0.2,v2=0.2):
        self.reset_variables()

        msg = WheelsCmdStamped()
        msg.header.stamp = rospy.Time.now()
        msg.vel_left = v1
        msg.vel_right = v2

        d_traveled = 0

        while d_traveled < distance:
            self.pub.publish(msg)
            self.pub_cmd.publish(msg)
            d_traveled = (self.left_distance + self.right_distance)/2
            rospy.sleep(0.1)

        self.stop()
        rospy.sleep(0.2)

    def take_position(self):
        self.reset_variables()
        if self.PARKING_SLOT == 1 or self.PARKING_SLOT == 3:
            self.move(0.5)
            if self.PARKING_SLOT == 1:
                self.turn("left",(math.pi/2))
            else:
                self.turn("right",(math.pi/2))
        else:
            self.move(0.25)
            if self.PARKING_SLOT == 2:
                self.turn("left",(math.pi/2))
            else:
                self.turn("right",(math.pi/2))

        rospy.sleep(2)

    def allign(self,target = True):
        self.reset_variables()
        if target:
            while self.tr_x > 0.05 or self.tr_x < -0.05:
                print("Alligning to target")
                print(self.tr_x)
                print()
                if self.tr_x > 0.05:
                    self.turn("right",(math.pi/18))
                elif self.tr_x < -0.05:
                    self.turn("left",(math.pi/18))
        
        else:
            while self.tr_x_helper > 0.05 or self.tr_x_helper < -0.05:
                print("Alligning to the helper")
                print(self.tr_x_helper)
                print()
                if self.tr_x_helper > 0.05:
                    self.turn("right",(math.pi/18))
                elif self.tr_x_helper < -0.05:
                    self.turn("left",(math.pi/18))
        self.stop()
        rospy.sleep(1)


    def shutdown(self):
        self.genreal_shutdown.publish("shutdown")
        
        rospy.signal_shutdown("Shutting Down ...")

    def main(self):
        self.take_position()
        self.allign()
        distance = self.tr_z

        self.turn("left")
        self.allign(False)

        self.move(distance - 0.2,v1 = -0.2, v2 = -0.2)
        
        self.shutdown()
        """
        Main idea is:
                taking position -> calculate distance using translation matrix -> Do 180 turn -> Use Rotation Matrix to Allign -> Go Backwards
        
        
        """
        pass

if __name__ == '__main__':
    parking_node = Parking('parking_node')
    while not rospy.is_shutdown():
        parking_node.main()



