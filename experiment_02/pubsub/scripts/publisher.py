#!/usr/bin/python3

import rospy

from std_msgs.msg import String

class Publisher:
    """
    This Class is meant to explain Publishers in ROS.
    """

    def __init__(self, topic_name, publish_rate):
       
        """
        __init__(self) of a class is the first function that runs of the class
        when the class is initialized.

        self is a default member of a class that can store any object.
        """

        """
        Publisher theory:

        Publishers are any nodes in the system that publishs a specific msg
        over a specific topic.
        """

        print("Initializing Node")

        # Initializes the Node.
        rospy.init_node("publisher_node")

        # Initializes a publihser within the node
        self.publisher = rospy.Publisher(topic_name, String, queue_size=10)

        # Initializing a timer
        """
        Timer Theory:

        A timer periodically runs a function.

        rospy.Timer(period_in_seconds, function_to_call)

        The function being called at every period is called, Callback

        """
        period = rospy.Duration(publish_rate)

        # This timer runs timerCallback() func once every second
        self.timer = rospy.Timer(period, self.timerCallback)

        # Message to published
        self.msg = String("Hello World!")

    def timerCallback(self, event):
        print("Publishing message: ", self.msg.data)
        self.publisher.publish(self.msg)

if __name__ == '__main__':

    p = Publisher("topic1", 1)
    """
    Think of spin that does not let a program stop and let's the timer run 
    periodically
    """
    rospy.spin()    
