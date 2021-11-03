#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class lane_follow:
    def __init__(self):
        self.pub_cmd = rospy.Publisher("cmd", Twist2DStamped, queue_size=10)
        self.sub_cam = rospy.Subscriber("lane_pose", LanePose, self.callback, queue_size=10)
        self.v = .3     #constant velocity during lane following
        self.last_d_err = 0
        self.last_phi_err = 0
        self.last_time = None
        self.kp = 1     #propertional gain
        self.kd = 1     #derivative gain
    #def get_errors(self, pos_data):
    #    self.d_err = pos_data.d
    #    self.phi_err = pos_data.phi
    def calc_der(self, e, last_e, dt):
        """
        Calculates derivates term using the expression: kd*((e_k - e_k-1)/dt)
        e and last_e represent d or phi, depending on for which variable this function is being called for
        function returns value of derivative term
        """
        dterm = self.kd * ((e-last_e)/dt)
        return dterm
         
    def calc_next_action(self, d_err, phi_err, dt):
        """
        Calculates and returns next action of robot, omega of Twist2DStamped, using PD controller:
            two terms for each variable, d and phi. One term is the proportional gain and one term is the derivative gain
        Inputs are d and phi errors, from lane_pos, and delta time
        function also uses class attributes self.last_phi_err and self.last_d_err for calculation of derivative term
        function updates last_phi_err and last_d_err for next callback
        """
        prop_d = self.kp * d_err      #proportional term for d
        prop_phi = self.kp * phi_err  #proportioanl term for phi
        der_d = self.calc_der(d_err, self.last_d_err, dt)        #derivative term for d
        der_phi = self.calc_der(phi_err, self.last_phi_err, dt)  #derivative term for phi
        omega = prop_d + prop_phi + der_d + der_phi     #next omega, u = sum of terms

        #update last phi and last d
        self.last_d_err = d_err
        self.last_phi_err = phi_err
        return omega

    def callback(self, pos_data):
        """
        Callback function for lane_pose topic
        retrieves error data from lane_pose, and uses it to calculate next robot action with a PD controller:
            
            finds delta time using current time and saved previous time
            gets error values from lane_pose
            thresholds error if too large
            calls controller function to get next robot action
            initializes v and omega of next action and publishes to cmd topic
            updates last time to current time for next callback
        """
        rospy.loginfo("callback")
        #find delta time
        curr_time = rospy.Time.now().to_sec()
        if self.last_time is not None:
            dt = curr_time - self.last_time
        else:
            dt = curr_time
        
        #get errors from camera node
        d_err = pos_data.d
        phi_err = pos_data.phi

        #rospy.loginfo("d: "+str(d_err))
        #rospy.loginfo("phi: "+str(phi_err))

        #TODO threshold error if too large
        
        #get v and omega from controller
        omega = self.calc_next_action(d_err, phi_err, dt)

        cmd_msg = Twist2DStamped()
        #cmd_msg.header = pose_data.header
        cmd_msg.v = self.v
        cmd_msg.omega = omega

        #publish command
        self.pub_cmd.publish(cmd_msg)

        #update last_time
        self.last_time = curr_time
if __name__ == '__main__':
    try:
        rospy.init_node('lane_following_p3.py')
        lane_follow()
        rospy.spin()
    except ROSInterruptException:
        pass


