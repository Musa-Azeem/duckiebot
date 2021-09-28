#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float32

class Homework3Pub:
    def __init__(self):
        rospy.Subscriber('input', Float32, self.callback)
        self.pub = rospy.Publisher("delta", Float32, queue_size = 10)

    def callback(self, data):
        self.pub.publish(data.data)

if __name__ == '__main__':
    try:
        rospy.init_node('homework3')
        Homework3Pub()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
