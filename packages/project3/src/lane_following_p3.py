#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class lane_follow:
	def __init__(self):
		self.pub_cmd = rospy.Publisher("cmd", Twist2DStamped, queue_size=10)
		rospy.Subscriber("lane_pose", LanePose, self.callback, queue_size=10)
		rospy.on_shutdown(self.stop)
		
		self.v = .5     #constant velocity during lane following
		self.kp = 1     #propertional gain
		self.kd = 0     #derivative gain
		
		self.last_d_err = 0
		self.last_phi_err = 0
		self.last_time = None
	def calc_der(self, e, last_e, dt):
		dterm = self.kd * ((e-last_e)/dt)
		return dterm
        	 
	def calc_next_action(self, d_err, phi_err, dt):
		prop_d = self.kp * d_err                                #proportional term for d
		prop_phi = self.kp * phi_err                            #proportioanl term for phi
		der_d = self.calc_der(d_err, self.last_d_err, dt)       #derivative term for d
		der_phi = self.calc_der(phi_err, self.last_phi_err, dt) #derivative term for phi
		omega = prop_d + prop_phi + der_d + der_phi             #next omega, u = sum of terms
	
		#update last phi and last d
		self.last_d_err = d_err
		self.last_phi_err = phi_err
	
		return omega

	def callback(self, pos_data):
		rospy.logwarn("Running Project3 Lane Following")
	
		self.v = rospy.get_param('~v', '0.3')
		self.kp = rospy.get_param('~kp', '1')
		self.kd = rospy.get_param('~kd', '0')
		
		#find delta time
		curr_time = rospy.Time.now().to_sec()
		if self.last_time is not None:
			dt = curr_time - self.last_time
		else:
			dt = curr_time
        
		#get errors from camera node
		d_err = pos_data.d
		phi_err = pos_data.phi
	
		rospy.loginfo("d: "+str(d_err))
		rospy.loginfo("phi: "+str(phi_err))

		#get omega from controller
		omega = self.calc_next_action(d_err, phi_err, dt)
	
		#threshold error if too large
		if omega > 4:
			rospy.loginfo("thresholded to 4")
			omega=4
		elif omega < -4:
			rospy.loginfo("thresholded to -4")
			omega=-4
		
		cmd_msg = Twist2DStamped()
		cmd_msg.v = self.v
		cmd_msg.omega = .5*omega
	
		#reduce velocity by .25 if omega is high
		#if abs(omega) > 4:
		#	cmd_msg.v *= .25
        	
		#publish command
		self.pub_cmd.publish(cmd_msg)
		rospy.loginfo("published v: "+str(cmd_msg.v))
		rospy.loginfo('published omega: '+str(omega))
	
		#update last_time
		self.last_time = curr_time
	
	def stop(self):
		rospy.loginfo("stopping duckiebot")
		stop_cmd = Twist2DStamped()
		stop_cmd.v = 0
		stop_cmd.omega = 0
		self.pub_cmd.publish(stop_cmd)

if __name__ == '__main__':
	try:
		rospy.init_node('lane_following_p3.py')
		lane_follow()
		rospy.spin()
	except ROSInterruptException:
		rospy.loginfo("ros interrupt exception") 

