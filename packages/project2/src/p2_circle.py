#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32

def publish_value():
    pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
    rospy.init_node('p2_circle')
    circle_cmd = Twist2DStamped()
    circle_cmd.v = .2
    circle_cmd.omega = .3631
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        pub.publish(circle_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_value()
    except rospy.ROSInterruptException:
        #stop robot
        rospy.loginfo("FAIL")
        stop_cmd = Twist2DStamped()
        stop_cmd.v = 0
        stop_cmd.omega = 0
        pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size=10)
        pub.publish(stop_cmd)
