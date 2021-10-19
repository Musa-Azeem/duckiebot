#!/usr/bin/python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32

class circle:
    """
    Class to hold functions to move robot in circle
    """
    def __init__(self):
        #values
        self.wheel_length = .1016   #l
        self.circle_radius = .525     #R
        self.vr = .64                #change to desired speed
        #vl = (2R-l)/(2R+l) * vr    since vr > vl, duckiebot will move in a counterclockwise rotation
        self.vl = ((2*self.circle_radius-self.wheel_length)/(2*self.circle_radius+self.wheel_length))*self.vr

        #ros elements
        self.pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
        self.circle_cmd = Twist2DStamped()
        self.circle_cmd.v = (self.vr+self.vl)/2     #v=(vr+vl)/2
        self.circle_cmd.omega = (self.vr-self.vl)/self.wheel_length #omega=(vr-vl)/l positive for counterclockwise
        self.stop_cmd = Twist2DStamped()
        self.stop_cmd.v = 0
        self.stop_cmd.omega = 0
        self.sleep_dur = rospy.Duration(8, 0) #change to time needed for perfect circle

        
        rospy.on_shutdown(self.stop)   #set function to be called if node is shutdown
    def run(self):
        """
        Publishes command to make robot go in circle
        waits enough time for circle to complete
        sends stop command
        """
        rospy.sleep(3.)                     #wait for publisher to be ready
        rospy.loginfo("starting circle")    
        self.pub.publish(self.circle_cmd)   #publish command to start circle
        rospy.sleep(self.sleep_dur)         #wait for duckiebot to complete circle
        rospy.loginfo("stopping duckiebot")
        self.stop()                         #stop duckiebot

    def stop(self):
        """
        stops duckiebot
        is automatically called when node is shutdown
        """
        rospy.loginfo("shutting down")
        self.pub.publish(self.stop_cmd)

if __name__ == '__main__':
    rospy.init_node('p2_circle')    #init node to publish circle commands
    go_circle = circle()
    try:
        go_circle.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("could not launch p2_circle.py")


