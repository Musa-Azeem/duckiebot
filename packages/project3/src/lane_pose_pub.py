#!/usr/bin/env python3

import rospy
from random import randint
from duckietown_msgs.msg import LanePose
from std_msgs.msg import Float32

def publish():
    pub = rospy.Publisher('lane_pose', LanePose, queue_size=10)
    dur = rospy.Duration(2,0);
    while True:
        pos = LanePose()
        pos.d = randint(-2,2)
        pos.phi = randint(-2,2)
        pos.in_lane = True
        pub.publish(pos)
        rospy.sleep(dur)
if __name__ == '__main__':
    try:
        rospy.init_node('lane_pose_pub')
        publish()
    except rospy.ROSInterruptException:
        pass

