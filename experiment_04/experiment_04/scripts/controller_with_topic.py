#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Twist

from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

from experiment_04_msg.msg import coor

class Proportional:
    """
    This Class is meant to explain Proportional Control using ROS.
    """

    def __init__(self, ka, kx, x=1, y=1):

        """
        Proportional Controller:

            if error = current_position - goal_position
            Then, you would want your velocity to be:
            k*error. Where k is some real constant called gain.

            Why?

            Because, you would want to approach the goal position
            in accordance to your current position (Slope).

            Slope = d(position)/d(time)

            d(time) is the time you want to take to reach that position,
            it describes how aggresive your controller has to behave. Omiting
            d(time) and having a constant K gain, will help tune the aggresion
            of the controller.

            For example:

                If your gain is too high, then, your controller will over react
                to even small errors. If your gain is too small, then it will 
                not react in time to your errors.


        """

        print("Initializing Node")

        # Initializes the Node.
        rospy.init_node("publisher_node")

        # Initializes a publihser within the node
        self.controlPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        self.odomSub = rospy.Subscriber("/odom", Odometry, self.control)

        self.odomSub = rospy.Subscriber("/cmd_goal", coor, self.set_goal_from_msg)

        # K gain for angle
        self.ka = ka

        # K gain for velocity
        self.kx = kx

        self.set_goal(x, y)

    def set_goal_from_msg(self, msg):

        self.set_goal(msg.x, msg.y)

    def set_goal(self, x, y):

        self.goal = [x, y]

    def control(self, msg):

        error_x = msg.pose.pose.position.x - self.goal[0]
        error_y = msg.pose.pose.position.y - self.goal[1]

        quat = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        error_theta = math.atan2(self.goal[1], self.goal[0]) - yaw

        roll = 0

        error_linear = math.sqrt(error_x**2 + error_y**2)
        msg_ = Twist()

        if(abs(error_theta) > 0.1):
            msg_.angular.z = self.ka*error_theta
            msg_.linear.x = 0

        else:
            if(abs(error_linear) > 0.2):
                msg_.linear.x = self.kx*error_linear
                msg_.angular.z = 0
            
            else:
                msg_.linear.x = 0
                msg_.angular.z = 0

        self.controlPub.publish(msg_)



if __name__ == '__main__':

    p = Proportional(1, 0.3, 1, 1)
    rospy.spin()
