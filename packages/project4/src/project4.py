#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LanePose

class lane_follow:
	def __init__(self):
 		#self.pub = rospy.Publisher("cmd",  
 		self.vel = rospy.get_param("~project4/vel_max", '.5')
		self.vel_min = rospy.get_param("~project4/vel_min", 0)
#	def callback(self, pos_data):
	        
	def stop(self):
		rospy.loginfo("stopping duckiebot")
		vel_right = self.vel_min
		vel_left = self.vel_min
		self.pub.publish(vel_right, vel_left)

if __name__ == '__main__':
	try:
 		rospy.init_node('csce274project4')
		#lane_follow()
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("ros interrupt exception") 

