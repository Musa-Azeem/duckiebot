#!/usr/bin/env python3

import rospy
import random

from std_msgs.msg import Float32

def publish_value():
        pub = rospy.Publisher("homework2/delta", Float32, queue_size = 10)
        rospy.init_node('homework3')
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            pub.publish(random.randint(0,100))
            rate.sleep()

if __name__ == '__main__':
    try:
        publish_value()
    except rospy.ROSInterruptException:
        pass
    
