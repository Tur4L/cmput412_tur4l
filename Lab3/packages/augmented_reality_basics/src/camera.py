#!/usr/bin/env python3

"""
TODO: Shutdown nodes (send shutdown request to odometry node).
"""

import math
import os
import subprocess
import time

import rosbag
import rospy
from std_msgs.msg import Float32, String

from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped
from duckietown_msgs.srv import ChangePattern


class DriverNode(DTROS):
    def __init__(
        self,
        node_name,
        init_frame_bag_name,
        world_frame_bag_name
    ):
        super(DriverNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC
        )
        
        # Static variables
        self._veh = os.environ["VEHICLE_NAME"]
        self._rate = rospy.Rate(100)

        # Service client
        rospy.wait_for_service(f"/{self._veh}/led_emitter_node/set_pattern")
        
        self.srv_led = rospy.ServiceProxy(
            f"/{self._veh}/led_emitter_node/set_pattern",
            ChangePattern
        )

        # Publisher
        self.pub_vel = rospy.Publisher(
            f"{self._veh}/wheels_driver_node/wheels_cmd",
            WheelsCmdStamped,
            queue_size=10
        )
        self.pub_executed_cmd = rospy.Publisher(
            f"/{self._veh}/wheels_driver_node/wheels_cmd_executed",
            WheelsCmdStamped,
            queue_size=10
        )
        
        # Subscribers
        self.delta_dist_left = rospy.Subscriber(
            f"{self._veh}/odometry_node/left_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="left"
        )
        self.delta_dist_right = rospy.Subscriber(
            f"{self._veh}/odometry_node/right_wheel_delta",
            Float32,
            self.cb_param_update,
            callback_args="right"
        )

        def readYamlFile(self,fname):
            """
            Reads the YAML file in the path specified by 'fname'.
            E.G. :
            the calibration file is located in : `/data/config/calibrations/filename/DUCKIEBOT_NAME.yaml`
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
    
if __name__ == "__main__":
    # Initialize driver node
    driver = DriverNode("drive", "if_bag", "wf_bag")