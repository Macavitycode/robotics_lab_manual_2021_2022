#!/usr/bin/python3

import rospy
from std_msgs.msg import String

class subscriber:

    """
    This class' purpose is to demonstrate subscribers.
    """

    def __init__(self, topic_name):

        """
        Subscriber Theory:

            Subscribers subscribe or listen to a topic and extract information
            from it in a predefined structure.

        """

        rospy.init_node("subscriber_node")

        self.sub = rospy.Subscriber(topic_name, String, self.subCallback)

    def subCallback(self, msg):

        """
        Whenever the subscriber gets a msg, the callback function runs, just
        like timers. The msg object created is local to the callback funtion.
        This is why it is important to use classes for inter function msg
        exchange. The use of global variables for the same purpose should be
        avoided, As usage of global variables are looked down upon in larger
        codebases due to collisions.
        """

        print(msg)


if __name__ == '__main__':

    s = subscriber("topic1")
    rospy.spin()
