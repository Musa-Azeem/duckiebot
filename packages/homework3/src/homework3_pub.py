#!/usr/bin/env python3

import rospy
import random

from std_msgs.msg import Float32

#class Homework3Pub:
#    def __init__(self):
#        #rospy.Subscriber('input', Float32, self.callback)
#        self.pub = rospy.Publisher("delta", Float32, queue_size = 10)
#
#    def callback(self, data):
#        self.pub.publish(data.data)

def publish_value():
        pub = rospy.Publisher("delta", Float32, queue_size = 10)
        rospy.init_node('homework3')
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(randint(-100,100))
            rate.sleep()

if __name__ == '__main__':
    try:
        #Homework3Pub()
        #rospy.spin()
        publish_value()
    except rospy.ROSInterruptException:
        pass
    
