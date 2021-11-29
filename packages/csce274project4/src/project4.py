#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import LanePose
from duckietwon_msgs.msg import WheelsCmdStamped

class lane_follow:
	def __init__(self):
 		self.pub = rospy.Publisher("./duck28/wheels_driver_node/wheels_cmd", WheelsCmdStamped, queue_size=1)
		self.sub = rospy.Subscriber("./duck28/lane_filter_node/lane_pos", LanePose, self.callback, queue_size=1) 
 		self.vel = rospy.get_param("project4/vel_max", '.5')
		self.vel_min = rospy.get_param("project4/vel_min", '0')
		self.kp = rospy.get_param("project4/p",1)
		self.on_shutdown(self.stop)
		init_pub()
	def init_pub(self):
		msg = WheelsCmdStamped()
		msg.vel_left = self.vel
		msg.vel_right = self.vel
		self.pub.publish(msg)
	def callback(self, data):
		phi = data.phi
		msg = WheelsCmdStamped()
		if phi < 0:
			msg.vel_right = self.vel
			msg.vel_left = self.vel + phi*self.kp
		elif phi > 0:
			msg.vel_left = self.vel
			msg.vel_right = self.vel - phi*self.kp
		else:
			msg.vel_left = self.vel
			msg.vel_right = self.vel
		rospy.loginfo("Data: duck28, vel_min:{}, vel_max:{}, vel_left:{}, vel_right:{}, p: {}, i: 0, d: 0".format(self.vel_min, self.vel, msg.vel_left, msg.vel_right, self.kp))
		self.pub.publish(msg)
		        
	def stop(self):
		rospy.loginfo("stopping duckiebot")
		msg = WheelsCmdStamped()
		msg.vel_right = self.vel_min
		msg.vel_left = self.vel_min
		self.pub.publish(msg)

if __name__ == '__main__':
	try:
 		rospy.init_node('csce274project4')
		lane_follow()
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("ros interrupt exception") 

