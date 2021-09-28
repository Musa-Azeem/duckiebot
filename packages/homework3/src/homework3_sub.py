#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class Homework3Sub:
    def __init__(self):
        rospy.Subscriber('total', Float32, self.callback)
    def callback(self, data):
        rospy.loginfo(str(data.data))

if __name__ == '__main__':
    rospy.init_node('homework3_sub')
    Homework3Sub()
    rospy.spin()
