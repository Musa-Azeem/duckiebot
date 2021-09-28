#!/usr/bin/env python3

import rospy

class Homework3Sub:
    def __init__(self):
        rospy.Subscriber('total', Float32, self.callback)
    def callback(self, data):
        ROS_INFO_STREAM_ONCE(data.data)
