#!/usr/bin/env python3
import numpy as np
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class OdometryNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(OdometryNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = os.environ["VEHICLE_NAME"]

        # Get static parameters
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node', WheelEncoderStamped, self.cb_encoder_data,callback_args = "left",)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node', WheelEncoderStamped, self.cb_encoder_data,callback_args = "right")
        self.sub_executed_commands = rospy.Subscriber(f'/{self.veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)


        # Publishers
        self.pub_integrated_distance_left = rospy.Publisher(f'/{self.veh_name}/odometry_node/left_wheel_distance',Float32,queue_size = 10)
        self.pub_integrated_distance_right = rospy.Publisher(f'/{self.veh_name}/odometry_node/right_wheel_distance',Float32,queue_size = 10)

        self.log("Initialized")

        self.right_wheel_tick_n = 0
        self.right_wheel_distance = 0

        #bag
        self.left_wheel_tick_n = 0
        self.left_wheel_distance = 0
        self.dir_left = 1
        self.dir_right = 1


    def cb_encoder_data(self, wheel, msg):
        """ 
        Update encoder distance information from ticks.
        """
        direction = 0
        is_left = bool

        if wheel == "right":
            self.right_wheel_tick_n = msg.data
            is_left = False
            direction = self.dir_right

        elif wheel == "left":
            self.left_wheel_tick_n = msg.data
            is_left = True
            direction = self.dir_left

        if is_left:
            tick_n = self.left_wheel_tick_n

        else:
            tick_n = self.right_wheel_tick_n

        delta_x = 2 * math.pi * self._radius * tick_n / 135
        delta_x *= direction

        if is_left:
            self.left_wheel_distance += delta_x
            self.pub_integrated_distance_left.publish(self.left_wheel_distance)

        else:
            self.right_wheel_distance += delta_x
            self.pub_integrated_distance_right.publish(self.right_wheel_distance)

    def cb_executed_commands(self, msg):
        """ Use the executed commands to determine the direction of travel of each wheel.
        """
        if msg.vel_left > 0:
            self.dir_left = 1

        else:
            self.dir_left = -1

        if msg.vel_right > 0:
            self.dir_right = 1

        else:
            self.dir_right = -1


if __name__ == '__main__':
    node = OdometryNode(node_name='my_odometry_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")