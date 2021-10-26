#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32

class Homework5:
    def __init__(self):
        rospy.init_node('homework5')
        self.pub = rospy.Publisher("total_converted", Float32, queue_size = 10)
        rospy.Subscriber("homework2/total", Float32, self.callback)

    def callback(self, data):
        unit = rospy.get_param('unit','meter')
        data_converted = self.convert(data, unit)
        self.pub.publish(data_converted)

    def convert(self, data, unit):
        """
        Converts data from feet to the given unit
        if unit is not meter or smoot, feet is assumed
        """
        if unit=='smoot':
            return data.data/5.5833
        elif unit=='meter':
            return data.data/3.281
        else:
            return data.data

if __name__=='__main__':
    Homework5()
    rospy.spin()
