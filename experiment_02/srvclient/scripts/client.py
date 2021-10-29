#!/usr/bin/python3

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

class client:

    """
    This class' purpose is to demonstrate clients.
    """

    def __init__(self, srv_name):

        """
        Client Theory:

            Clients use something called remote procedure calls or rpc. Client
            sends a request to the server and gets back a response.

        """

        rospy.init_node("client_node")

        rospy.wait_for_service(srv_name)
        
        try:

            self.client = rospy.ServiceProxy(srv_name, SetBool)
           
            request = SetBoolRequest()

            request.data = False

            print("Send Request: ", request)

            response = self.client(request)

            print("Got Response: ", response)


        except rospy.ServiceException as e:

            print("Service call failed: %s"%e)


if __name__ == '__main__':

    c = client("boolcall")
