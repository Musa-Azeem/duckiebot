#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32

def publish_value():
    pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
    rospy.init_node('p2_circle')
    circle_cmd = Twist2DStamped()
    circle_cmd.v = .2
    circle_cmd.omega = 4
    while not rospy.is_shutdown():
        pub.publish(circle_cmd)

if __name__ == '__main__':
    try:
        publish_value()
    except rospy.ROSInterruptException:
        rospy.loginfo("FAIL")

