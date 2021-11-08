#!/usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray

if __name__ == '__main__':

    rospy.init_node("Give_Commands")
    x = rospy.Publisher("/trpy", Float64MultiArray, queue_size=10)

    msg = Float64MultiArray()
    msg.data = [60.0, 0, 0, 0]

    while(not rospy.is_shutdown()):
        x.publish(msg)

    