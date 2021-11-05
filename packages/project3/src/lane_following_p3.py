#!/usr/bin/env python3

import rospy
from duckietown_msgs.msg import Twist2DStamped, LanePose

class lane_follow:
    """
    Class to define lane following functionality
        Implements a PD controller, using the lane position error information from the lane_pos topic
        PD controller calculates the next angular velocity the duckiebot should move at
    Publishes to "cmd" - type Twist2DStamped
    Subcribes to "lane_pose" - type LanePose
        Callback function of lane_pose calculates next action and publishes to cmd
    Class constants:
        v:  velocity of duckiebot at all times
        kp: proportional gain
        kd: derivative gain
    """
    def __init__(self):
        """
        Initialize lane_follow Publisher and Subscriber
        Set class constants and initialize variables
        """
        self.pub_cmd = rospy.Publisher("cmd", Twist2DStamped, queue_size=10)
        self.sub_cam = rospy.Subscriber("lane_pose", LanePose, self.callback, queue_size=10)
        rospy.on_shutdown(self.stop)

        #self.v = 1.5     #constant velocity during lane following
        self.v = int(rospy.get_param('v', '0.3'))
        #self.kp = 1.3     #propertional gain
        #self.kd = 1     #derivative gain
        self.kp = int(rospy.get_param('kp', '1'))
        self.kd = int(rospy.get_param('kd', '1'))


        self.last_d_err = 0
        self.last_phi_err = 0
        self.last_time = None
        rospy.loginfo('in __init__')

    def calc_der(self, e, last_e, dt):
        """
        Calculates derivate term using the expression: kd*((e_k - e_k-1)/dt)
        e and last_e represent d or phi, depending on for which variable this function is being called for
        function returns value of derivative term
        """
        dterm = self.kd * ((e-last_e)/dt)
        return dterm
         
    def calc_next_action(self, d_err, phi_err, dt):
        """
        Calculates and returns next action of robot, omega of Twist2DStamped, using PD controller:
            There are two terms for each variable, d and phi. 
            One term is the proportional gain and one term is the derivative gain
        Inputs are d and phi errors, from lane_pos, and delta time
        The function also uses class attributes self.last_phi_err and self.last_d_err for calculation of derivative term
        Before returning, the function updates last_phi_err and last_d_err for next callback
        """
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
        """
        Callback function for lane_pose topic:
            Retrieves error data from lane_pose, and calls calculate_next_action to calculate next robot action
            Finds delta time using current time and saved previous time
            Gets error d and phi values from lane_pose
            Thresholds error if too large - stops node if too large
            Calls controller function to get next robot action
            Sett
 v and omega of next action message and publishes to cmd topic
            Updates last time to current time for next callback
        """
        self.v = int(rospy.get_param('v', '0.5'))
        self.kp = int(rospy.get_param('kp', '1'))
        self.kd = int(rospy.get_param('kd', '1'))

        rospy.loginfo("in callback")
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

        #TODO threshold error if too large
        
        #get omega from controller
        omega = self.calc_next_action(d_err, phi_err, dt)

        cmd_msg = Twist2DStamped()
#        cmd_msg.header = pose_data.header
	rospy.loginfo(str(self.v))
        cmd_msg.v = self.v
        cmd_msg.omega = omega

        #publish command
        self.pub_cmd.publish(cmd_msg)
        rospy.loginfo('published: '+str(omega))

        #update last_time
        self.last_time = curr_time

    def stop(self):
        """
        Function to stop robot when node is shutdown or if called
        Sets velocity and omega of duckiebot to 0
        """
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


