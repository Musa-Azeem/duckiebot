#!/usr/bin/python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32

class Pent:
    """
    Class to hold functions to move robot in 1m pentagon
    """
    def __init__(self):
        self.pub = self.pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)

        self.wheel_length = .1016   #l
        self.side = .2              #d
        self.angle = .6*3.14259     #theta

        self.side_v = .4            #side_v = vr = vl velocity for both wheel during sides      
        self.side_dur = rospy.Duration(self.side/self.side_v)   #t=d/v
        self.side_cmd = Twist2DStamped()
        self.side_cmd.v = self.side_v
        self.side_cmd.omega = 0     #no rotation during sides

        self.turn_vr = .4           #velocity of right wheel when turning (vl = -vr) vr is positive so sounterclockwise rotation
        self.turn_dur = rospy.Duration((self.angle*self.wheel_length)/(2*self.turn_vr)) #t=(theta*l)/(vr-vl)
        self.turn_cmd = Twist2DStamped()
        self.turn_cmd.v = 0     #(vr+vl)/2 = 0
        self.turn_cmd.omega = (2*self.turn_vr)/self.wheel_length #omega = (vr-vl)/l positive for counterclockwise rotation

        self.stop_cmd = Twist2DStamped()
        self.stop_cmd.v = 0
        self.stop_cmd.omega = 0
        rospy.on_shutdown(self.stop)   #set function to be called if node is shutdown

    def run(self):
        """
        function to make duckiebot travel in a pentagon
        """
        rospy.sleep(3.)                     #wait for publisher to be ready
        rospy.loginfo("starting pentagon")  
        for i in range(1,6):
            rospy.loginfo("starting side "+str(i))
            self.pub.publish(self.side_cmd)  #give command to go straight
            rospy.sleep(self.side_dur)        #stop after enough time to travel side length

            rospy.loginfo("starting turn "+str(i))
            self.pub.publish(self.turn_cmd)
            rospy.sleep(self.turn_dur)
        self.stop()



    def stop(self):
        """
        stops duckiebot
        is called automatically if node is stopped
        """
        rospy.loginfo("shutting down")
        self.pub.publish(self.stop_cmd)

