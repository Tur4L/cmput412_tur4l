#!/usr/bin/env python3
import numpy as np
import os
import math
import rospy
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32

class DriverNode(DTROS):

    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """

        # Initialize the DTROS parent class
        super(DriverNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        
        # Get static parameters
        self._veh_name = os.environ["VEHICLE_NAME"]
        self._radius = rospy.get_param(f'/{self.veh_name}/kinematics_node/radius', 100)
        self._robot_width_half = 0.05
        self._rate = rospy.Rate(1)

        # Subscribing to the wheel encoders
        self.sub_encoder_ticks_left = rospy.Subscriber(f'/{self._veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data,callback_args = "left",)
        self.sub_encoder_ticks_right = rospy.Subscriber(f'/{self._veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped, self.cb_encoder_data,callback_args = "right")
        self.sub_executed_commands = rospy.Subscriber(f'/{self._veh_name}/wheels_driver_node/wheels_cmd_executed', WheelsCmdStamped, self.cb_executed_commands)


        # Publishers
        self.pub_vel = rospy.Publisher(f'/{self._veh_name}/wheels_driver_node/wheels_cmd',WheelsCmdStamped,queue_size = 10)
        self.pub_executed_cmd= rospy.Publisher(f'/{self._veh_name}/wheels_driver_node/wheels_cmd_executed',WheelsCmdStamped,queue_size = 10)

        self.log("Initialized")


        #Robot values
        self.left_wheel_distance = 0.0
        self.right_wheel_distance = 0.0
        self.robot_distance = 0.0
        self.total_ang = 0.0

    def reset_variables(self):
        self.left_wheel_distance = 0.0
        self.right_wheel_distance = 0.0
        self.robot_distance = 0.0

    def cb_param_update(self,msg,wheel):

        if wheel == "right":
            self.right_wheel_distance += msg.data
            self.total_dsitance = (self.left_wheel_distance + self.right_wheel_distance)/2
            self.total_ang = (abs(self.left_wheel_distance) + abs(self.right_wheel_distance))/(2* self._robot_width_half)
        else:
            self.left_wheel_distance += msg.data

    def send_msg(self,msg):

        self.pub_vel.publish(msg)
        self.pub_executed_cmd.publish(msg)


if __name__ == '__main__':
    node = DriverNode(node_name='driver_node')
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("wheel_encoder_node is up and running...")