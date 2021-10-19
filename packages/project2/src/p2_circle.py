#!/usr/bin/python3
import rospy
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Float32

class circle:
    """
    Class to hold functions to move robot in circle
    """
    def __init__(self):
        self.pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
        self.circle_cmd = Twist2dStamped()
        self.circle_cmd.v = .4       #change these values to what we had
        self.circle_cmd.omega = 2    #change these values to what we had
        self.stop_cmd = Twist2dStamped()
        self.stop_cmd.v = 0
        self.stop_cmd.omega = 0
        self.sleep_dur = rospy.Duration(26, 0) #change to time needed for perfect circle
    def run(self):
        """
        Publishes command to make robot go in circle
        waits enough time for circle to complete
        sends stop command
        """
        rospy.loginfo("starting circle")
        self.pub.publish(self.circle_cmd)
        rospy.sleep(self.sleep_dur)
        rospy.loginfo("stopping duckiebot")
        self.stop()

    def stop(self):
        """
        stops duckiebot
        is called if node is shutdown
        """
        rospy.loginfo("shutting down")
        self.pub.publish(self.stop_cmd)



#def circle():
#    """
#    Function to init node and publish command to tell robot to move in a 1m circle
#    """
#    pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
#    rospy.init_node('p2_circle')
#    circle_cmd = Twist2dStamped()
#    cricle_cmd.v = .4       #change these values to what we had
#    circle_cmd.omega = 2    #change these values to what we had
#
#    #stop duckiebot after time needed to complete circle
#    rospy.sleep(26.)        #change this to time needed for circle
#    stop_cmd = Twist2dStamped()
#    stop_cmd.v = 0
#    stop_cmd.omega = 0
#    pub.publish(stop_cmd)


#def stop():
#    """
#    Will stop robot if node is shutdown
#    """
#    pub = rospy.Publisher("/duck28/car_cmd_switch_node/cmd", Twist2DStamped, queue_size = 10)
#    stop_cmd = Twist2DStamped()
#    stop_cmd.v = 0
#    stop_cmd.omega = 0
#    pub.publish(stop_cmd)

if __name__ == '__main__':
    rospy.init_node('p2_circle')    #init node to publish circle commands
    go_circle = circle()
    rospy.is_shutdown(go_circle.stop())   #set function to be called if node is shutdown
    try:
        go_circle.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("could not launch p2_circle.py")


