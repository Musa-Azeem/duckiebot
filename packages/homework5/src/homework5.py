#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32

class Homework5:
    def __init__(self):
        """
        Initializes homework5 node as subscriber to homework2/total and publisher to total_converted
        """
        rospy.init_node('homework5')
        self.pub = rospy.Publisher("total_converted", Float32, queue_size = 10)
        rospy.Subscriber("homework2/total", Float32, self.callback)

    def callback(self, data):
        """
        Callback function of homework2/total subscriber
        Reads rosparam unit to get unit and calls convert function to convert data to that unit - defualt rosparam unit value is meter
        It then logs input and output units/data to INFO level
        Finally, it publishes the data in new units to homework2/total
        If the rosparam is not feet, meter, or smoot, it is set to meter
        """
        unit = rospy.get_param('unit','meter')
        if unit != 'feet' and unit != 'meter' and unit != 'smoot':
            unit = 'meter'
        data_converted = self.convert(data, unit)
        rospy.loginfo("input (feet): "+str(data.data))
        rospy.loginfo("output ("+unit+"): "+str(data_converted))
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
