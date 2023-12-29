#!/usr/bin/env python3

import json
import rospy

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Float32, Int32, String
from turbojpeg import TurboJPEG
import cv2
import numpy as np
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
import os
import threading
import math
import deadreckoning
import state_machine

HOST_NAME = os.environ["VEHICLE_NAME"]
ROAD_MASK = [(20, 60, 0), (50, 255, 255)]
DEBUG = False
ENGLISH = False
STOP_TIMER_RESET_TIME = 90

STOP_BECAUSE_RED_STOPLINE = 1
STOP_BECAUSE_BROKEN_DUCKIEBOT = 2
STOP_BECAUSE_CROSSWALK = 3

class LaneFollowNode(DTROS):

    def __init__(self, node_name):
        super(LaneFollowNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        self.node_name = node_name
        self.veh = HOST_NAME
        self.jpeg = TurboJPEG()
        self.loginfo("Initialized")

        # PID Variables
        self.proportional = None
        if ENGLISH:
            self.offset = -220
        else:
            self.offset = 220
        self.velocity = 0.34
        self.speed = .6
        self.twist = Twist2DStamped(v=self.velocity, omega=0)
        self.stop_flag = False

        self.P = 0.049
        self.D = -0.004
        self.last_error = 0
        self.last_time = rospy.get_time()

        self.timer = 0

        # handling stopping at stopline
        self.prep_turn = False  # initiate the turning when this is set to true
        self.stop_timer_reset = 0  # 0 is can stop any time, non-zero means wait a period of time and then we look for stop lines
        self.lock = threading.Lock()  # used to coordinate the subscriber thread and the main thread
        self.controller = deadreckoning.DeadReckoning()  # will handle wheel commands during turning

        # handling stopping at crosswalk
        self.stop_cause = None
        self.crosswalk_tag_detected = False
        self.broken_duckiebot_detected = False
        self.obj_class = []
        self.obj_scores = []
        self.obj_boxes = []

        # Publishers & Subscribers
        self.bot_state = state_machine.BotState(1)  # pass in 1 as placeholder; in the end self.bot_state is not used in part 3
        if DEBUG:
            self.pub = rospy.Publisher("/{self.veh}/output/image/mask/compressed",
                                   CompressedImage,
                                   queue_size=1)
        self.sub = rospy.Subscriber(f"/{self.veh}/camera_node/image/compressed", CompressedImage,
                                    self.callback, queue_size=1, buff_size="20MB")
        self.obj_sub = rospy.Subscriber(f'/{HOST_NAME}/detectron2_duckiebot/detected_objects', String, self.object_callback)
        self.tag_distance_sub = rospy.Subscriber(f'/{HOST_NAME}/detected_tag_distance', String, self.tag_distance_callback)
        self.general_sub = rospy.Subscriber('/general', String, self.general_callback)
        self.general_pub = rospy.Publisher('/general', String, queue_size=3)

        self.vel_pub = rospy.Publisher(f"/{self.veh}/car_cmd_switch_node/cmd",
                                       Twist2DStamped,
                                       queue_size=1)

    def tag_distance_callback(self, msg):
        words = msg.data.split()
        tagid = int(words[0])
        distance = float(words[1])
        if tagid == 163 and distance < .5:
            self.lock.acquire()
            self.crosswalk_tag_detected = True
            self.lock.release()

    def object_callback(self, msg):
        msg_json = json.loads(msg.data) or False
        if msg_json:
            self.lock.acquire() 
            self.obj_class = msg_json["class"]
            self.obj_scores = msg_json["scores"]     
            self.obj_boxes = msg_json["pred_boxes"]   
            for idx, val in enumerate(self.obj_class):
                if val == 1 and self.obj_scores[idx] > 0.9:
                    max_y = self.obj_boxes[idx][3]
                    if max_y > 200 and (self.obj_boxes[idx][0] <= 320 <= self.obj_boxes[idx][2]):
                        print('setting broken detected to true')
                        self.broken_duckiebot_detected = True
            self.lock.release()

    def general_callback(self, msg):
        if msg.data == 'shutdown':
            rospy.signal_shutdown('received shutdown message')
        # elif msg.data == 'part3_start':
        #     rospy.signal_shutdown('lane following node shutting down because first two parts completed')

    def reset_pid(self):
        self.proportional = None
        self.last_error = 0
        self.last_time = rospy.get_time()

    def is_turning(self):
        self.lock.acquire()
        is_turning = self.prep_turn
        self.lock.release()
        return is_turning
    
    def on_stopline(self, stop_cause):
        self.lock.acquire()
        self.stop_timer_reset = STOP_TIMER_RESET_TIME
        self.prep_turn = True
        self.stop_cause = stop_cause
        self.lock.release()
    
    def after_stopline(self):
        self.lock.acquire()
        self.stop_cause = None
        self.prep_turn = False
        self.lock.release()

    def callback(self, msg):
        # update stop timer/timer reset and skip the callback if the vehicle is stopped
        self.lock.acquire()
        stop_timer_reset = self.stop_timer_reset
        # if self.stop_timer_reset > 0:
        #     print(self.stop_timer_reset)
        self.stop_timer_reset = max(0, stop_timer_reset - 1)
        self.lock.release()
        if not self.bot_state.get_lane_following_flag():
            self.proportional = None
            return

        img = self.jpeg.decode(msg.data)
        flags = self.bot_state.get_flags()

        # print(self.bot_state.stateid, flags['is_expecting_crosswalk'])
        if flags['is_expecting_red_stopline']:
            if stop_timer_reset == 0:
                self.red_stopline_processing(img)
        elif flags['is_expecting_crosswalk']:
            # two cases: crosswalk or broken duckiebot
            # test for broken duckiebot first
            self.lock.acquire()
            broken_duckiebot_detected = self.broken_duckiebot_detected
            self.broken_duckiebot_detected = False
            self.lock.release()
            # print(broken_duckiebot_detected, stop_timer_reset)
            if broken_duckiebot_detected and stop_timer_reset == 0:
                self.on_stopline(STOP_BECAUSE_BROKEN_DUCKIEBOT)
                
            # recognize crosswalk by the apriltag detection
            self.lock.acquire()
            crosswalk_tag_detected = self.crosswalk_tag_detected
            self.crosswalk_tag_detected = False
            self.lock.release()
            if crosswalk_tag_detected and stop_timer_reset == 0:
                self.on_stopline(STOP_BECAUSE_CROSSWALK)


        crop = img[300:-1, :, :]
        crop_width = crop.shape[1]
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ROAD_MASK[0], ROAD_MASK[1])
        crop = cv2.bitwise_and(crop, crop, mask=mask)
        contours, hierarchy = cv2.findContours(mask,
                                               cv2.RETR_EXTERNAL,
                                               cv2.CHAIN_APPROX_NONE)

        # Search for lane in front
        max_area = 20
        max_idx = -1
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if area > max_area:
                max_idx = i
                max_area = area

        if max_idx != -1:
            M = cv2.moments(contours[max_idx])
            try:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                threshold = 200

                self.proportional = min(threshold, max(-threshold, cx - int(crop_width / 2) + self.offset))
                if DEBUG:
                    cv2.drawContours(crop, contours, max_idx, (0, 255, 0), 3)
                    cv2.circle(crop, (cx, cy), 7, (0, 0, 255), -1)
            except:
                pass
        else:
            self.proportional = -100 # assume off to the right

        if DEBUG:
            rect_img_msg = CompressedImage(format="jpeg", data=self.jpeg.encode(crop))
            self.pub.publish(rect_img_msg)

    def drive(self):
        if self.is_turning():
            if self.stop_cause == STOP_BECAUSE_RED_STOPLINE:
                self.controller.stop(20)
                self.controller.reset_position()

                turn_idx = self.bot_state.decide_turn_at_red_stopline()
                new_stateid = self.bot_state.advance_state()
                print(f'turn_idx:{turn_idx}, new_stateid:{new_stateid}')

                if new_stateid == state_machine.P3_ENTER:
                    self.general_pub.publish(String('part3_start'))
                    self.stop_flag = True
                    self.controller.stop_flag = True
                else:
                    self.controller.set_turn_flag(True)
                    self.controller.driveForTime(.6, .6, 6)
                    if turn_idx == 0:
                        self.controller.driveForTime(.65 * self.speed, 1.35 * self.speed, 60)
                    elif turn_idx == 1:
                        self.controller.driveForTime(.96 * self.speed, 1.04 * self.speed, 84)
                    elif turn_idx == 2:
                        self.controller.driveForTime(1.47 * self.speed, .53 * self.speed, 15)
                    self.controller.set_turn_flag(False)
                    self.reset_pid()
                if new_stateid == state_machine.P2_CROSSWALK_0:
                    self.velocity = 0.22
                    
            elif self.stop_cause == STOP_BECAUSE_CROSSWALK:
                # wait for duckies  
                timer = 20
                while timer > 0:
                    timer -=1
                    self.controller.stop(1)
                    self.lock.acquire()
                    if 0 in self.obj_class:
                        timer = 20
                    self.lock.release()
                    self.stop_timer_reset = STOP_TIMER_RESET_TIME
                new_stateid = self.bot_state.advance_state()
                # if new_stateid == state_machine.P2_CROSSWALK_2:
                #     self.velocity = 0.34
            elif self.stop_cause == STOP_BECAUSE_BROKEN_DUCKIEBOT:
                self.stop_timer_reset = math.inf
                self.controller.stop(20)
                self.controller.driveForTime(-.96 * self.speed, -1.04 * self.speed, 30)
                self.controller.stop(20)
                self.controller.driveForTime(.96 * self.speed, 1.04 * self.speed, 15)
                self.stop_timer_reset = int(32 / 8 * 30)
                self.timer = 32
                
            self.after_stopline()

        else:  # PID CONTROLLED LANE FOLLOWING
            if self.timer>0:
                self.offset = -220
                self.timer -=1
            if self.timer<=0:
                self.offset = 220 
            if self.proportional is None:
                self.twist.omega = 0
            else:

                # P Term
                P = -self.proportional * self.P

                # D Term
                d_error = (self.proportional - self.last_error) / (rospy.get_time() - self.last_time)
                self.last_error = self.proportional
                self.last_time = rospy.get_time()
                D = d_error * self.D

                self.twist.v = self.velocity
                self.twist.omega = P + D
                if DEBUG:
                    self.loginfo(self.proportional, P, D, self.twist.omega, self.twist.v)

            if not self.stop_flag:
                self.vel_pub.publish(self.twist)

    def red_stopline_processing(self, im):
        hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
        lower_range = np.array([0,70,120])
        upper_range = np.array([5,180,255])
        red_mask = cv2.inRange(hsv, lower_range, upper_range)
        img_dilation = cv2.dilate(red_mask, np.ones((10, 10), np.uint8), iterations=1)
        contours, hierarchy = cv2.findContours(img_dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # pick the largest contour
        largest_area = 0
        largest_idx = -1

        for i in range(len(contours)):
            ctn = contours[i]
            area = cv2.contourArea(ctn)

            xmin, ymin, width, height = cv2.boundingRect(ctn)
            xmax = xmin + width
            if area > largest_area and area > 3000 and xmax > im.shape[1] * .5 and xmin < im.shape[1] * .5:
                largest_area = area
                largest_idx = i

        contour_y = 0
        if largest_idx != -1:
            largest_ctn = contours[largest_idx]
            xmin, ymin, width, height = cv2.boundingRect(largest_ctn)
            contour_y = ymin + height * 0.5

        if contour_y > 390:
            self.on_stopline(STOP_BECAUSE_RED_STOPLINE)

    def hook(self):
        print("SHUTTING DOWN")
        self.twist.v = 0
        self.twist.omega = 0
        self.vel_pub.publish(self.twist)
        for i in range(8):
            self.vel_pub.publish(self.twist)


if __name__ == "__main__":
    node = LaneFollowNode("lanefollow_node")
    rate = rospy.Rate(8)  # 8hz
    while not rospy.is_shutdown():
        node.drive()
        rate.sleep()
