#!/usr/bin/python3

import rospy
import math
from std_msgs.msg import Float64MultiArray

"""
[front_left, back_left, back_right, front_right]
[thrust, roll, pitch, yaw]
"""

"""
topic_to_publish = /Kwad/joint_motor_controller/command
"""

class motor_mixing:

    def __init__(self):
        
        self.motor_limits = [50.0, 50.0, 50.0, 50.0]
        self.motorPub = rospy.Publisher("/Kwad/joint_motor_controller/command", Float64MultiArray, queue_size=10)
        self.trpySub = rospy.Subscriber("/trpy", Float64MultiArray, self.push_trpy)

    def get_motor_velocities(self, thrust, roll, pitch, yaw):
        
        fr = thrust + yaw + roll + pitch
        fl = thrust - yaw - roll + pitch
        br = thrust - yaw + roll - pitch
        bl = thrust + yaw - roll - pitch

        # if(fl > self.motor_limits[0]):
        #     fl = self.motor_limits[0]

        # if(bl > self.motor_limits[1]):
        #     bl = self.motor_limits[1]

        # if(br > self.motor_limits[2]):
        #     br = self.motor_limits[2]

        # if(fr > self.motor_limits[3]):
        #     fr = self.motor_limits[3]

        return (fl, bl, br, fr)

    def push_trpy(self, msg):
        
        mv = self.get_motor_velocities(msg.data[0], msg.data[1], msg.data[2], msg.data[3])
        
        pubMsg = Float64MultiArray()

        pubMsg.data = [mv[0], -mv[1], mv[2], -mv[3]]

        self.motorPub.publish(pubMsg)


if __name__ == '__main__':

    rospy.init_node("Controller")

    mm = motor_mixing()

    rospy.spin()