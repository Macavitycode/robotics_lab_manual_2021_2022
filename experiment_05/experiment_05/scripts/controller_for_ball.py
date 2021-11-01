#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

    def __init__(self, ka, kx):


        print("Initializing Node")

        # Initializes the Node.
        rospy.init_node("publisher_node")

        # Initializes a publihser within the node
        self.controlPub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=10)
        #  self.odomSub = rospy.Subscriber("/follower/odom", Odometry, self.control)
        self.cameraSub = rospy.Subscriber("/follower/camera1/image_raw", Image,
                self.image_proc)

        # K gain for angle
        self.ka = ka

        # K gain for velocity
        self.kx = kx

        # Ideal radius to be followed
        self.ideal_follow_radius = 30
        self.scale_r = 1/90

        # Ideal center to be followed
        self.ideal_follow_center = 200
        self.scale_c = 1/6000

        #  self.goal = [0, 0]

        self.bridge = CvBridge()


    def image_proc(self, msg):

        
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        lower_red_1 = (0, 100, 20)
        upper_red_1 = (10, 255, 255)

        #  lower_red_2 = (160, 100, 20)
        #  upper_red_2 = (180, 255, 255)

        mask_1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
        #  mask_2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

        mask = mask_1 # + mask_2

        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1, minDist=100,
                param1=30, param2=15, minRadius=10)

        try:
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

            #  self.goal[0] += (self.ideal_follow_radius - radius) * self.scale_r
            #  self.goal[1] = (self.ideal_follow_center - center[0]) * self.scale_c

        except:
            #  print(mask.shape)
            #  print("Circle not found")
            pass

        #  goal_text = ("goal: " +
        #      str(round(self.goal[0], 1)) +
        #      ", " +
        #      str(round(self.goal[1], 1)))
        #
        #  cv2.putText(image,
        #          goal_text,
        #          (10, 75),
        #          cv2.FONT_HERSHEY_SIMPLEX, 1,
        #          (255, 0, 0),
        #          2,
        #          cv2.LINE_4)
            

        cv2.imshow("mask", image)
        
        cv2.waitKey(1)


    #  def control(self, msg):


        #  quat = msg.pose.pose.orientation
        #  (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        #
        #  print("yaw: " + str(yaw))
        #
        #  error_theta = atan2(self.goal[1], self.goal[0]) - yaw
        #
        #  error_x = msg.pose.pose.position.x - self.goal[0]
        #  error_y = msg.pose.pose.position.y - self.goal[1]
        #
        #  error_linear = sqrt(error_x**2 + error_y**2)
        #
        #  new_msg = Twist()
        #
        #  if(abs(error_theta) > 0.1):
        #      print("theta: " + str(error_theta))
        #      new_msg.angular.z = self.ka * error_theta
        #      new_msg.linear.x = 0
        #
        #  else:
        #      if(abs(error_linear) > 0.2):
        #          print("linear: " + str(error_linear))
        #          new_msg.linear.x = self.kx * error_linear
        #          new_msg.angular.z = 0
        #
        #      else:
        #          new_msg.linear.x = 0
        #          new_msg.angular.z = 0
        #
        #  #  print(self.goal)
        #  #  print(error_linear, error_theta)
        #
        #  self.controlPub.publish(new_msg)



if __name__ == '__main__':

    p = Controller(0.5, 0.02)
    rospy.spin()
