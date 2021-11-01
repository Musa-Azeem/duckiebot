#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class lane_follow:
    def __init__(self):
        self.pub_cmd = rospy.Publisher("car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        self.sub_cam = rospy.Subscriber("lane_filter_node/lane_pose", LanePose, self.callback, queue_size=10)
        self.d_err = 0
        self.phi_err = 0
    def get_errors(self):
        #get errors
    def calc_der(self):
    
    def calc_next_action(self):

