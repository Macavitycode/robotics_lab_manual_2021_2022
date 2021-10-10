#!/usr/bin/python3

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

class server:

    """
    The purpose of this class is to demonstrate a server.
    """

    def __init__(self, srv_name):

        """
        Server Theory:

            Server works with Remote Procedure Call or rpc. A request call is
            made to the server and a reponse is sent back.
        """
        
        rospy.init_node("server_node")

        rospy.Service(srv_name, SetBool, self.srvHandler)

    def srvHandler(self, request):

        """
        Every single time a client makes a request, and just like callbacks,
        Handlers run. The request object is again local to this handler. The
        condition is handler's return value is sent back as a response to the
        client.
        """

        print("Recieved Data: ", request.data)
        response = SetBoolResponse()
        response.success = True
        response.message = "Data Recieved"

        return response


if __name__ == '__main__':

    s = server("boolcall")

    rospy.spin()
