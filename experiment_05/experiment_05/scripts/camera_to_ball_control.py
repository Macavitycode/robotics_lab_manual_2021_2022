#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

from tf.transformations import euler_from_quaternion
from math import atan2, sqrt

from cv_bridge import CvBridge
import cv2
import numpy as np

class Controller:
    """
    This Class is meant to explain Proportional Control using ROS.
    """

    def __init__(self, kc, kr):

        print("Initializing Node")

        # Initializes the Node.
        rospy.init_node("publisher_node")

        # Initializes a publihser within the node
        self.controlPub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=10)
        self.cameraSub = rospy.Subscriber("/follower/camera1/image_raw", Image,
                self.camera_callback)

        # K gain for angle
        self.kc = kc

        # K gain for velocity
        self.kr = kr

        # Ideal radius to be followed
        self.ideal_follow_radius = 30

        # Ideal center to be followed
        self.ideal_follow_center = 200

        self.bridge = CvBridge()


    def camera_callback(self, msg):

        new_msg = Twist()

        r = None
        c = None
        
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        try:
            
            (r, c) = self.img_proc(image)

            cv2.imshow("masked image", image)
            cv2.waitKey(1)

        except:
            print("No circles found")

            cv2.imshow("masked image", image)
            cv2.waitKey(1)
    

        if r is not None:
            new_msg.linear.x = (self.ideal_follow_radius - r) * self.kr
            new_msg.angular.z = (self.ideal_follow_center - c) * self.kc

        self.controlPub.publish(new_msg)

        
    def img_proc(self, image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_red = (0, 100, 20)
        upper_red = (10, 255, 255)

        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1,
                minDist=100, param1=30, param2=15, minRadius=5)

        np.uint16(circles)
        for i in circles[0, :]:
            center = (i[0], i[1])
            cv2.circle(image, center, 1, (0, 255, 0), 3)
            radius = i[2]
            cv2.circle(image, center, radius, (0, 255, 0), 3)
            
        radius_text = "r: " + str(radius)
        center_text = "c: " + str(center[0])

        cv2.putText(image, 
                radius_text, 
                (10, 25), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                (255, 0, 0), 
                2, 
                cv2.LINE_4)

        cv2.putText(image, 
                center_text,
                (10, 50), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, 
                (255, 0, 0), 
                2, 
                cv2.LINE_4)



        return (radius, center[0])


if __name__ == '__main__':

    p = Controller(0.1, 0.1)
    rospy.spin()
