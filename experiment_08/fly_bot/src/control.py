#!/usr/bin/env python

from pid import PID
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion

def control_kwad(msg, args):
    # Declare global variables as you dont want these to die, reset to zero and
    # then re-initiate when the function is called again.
    global roll, pitch, yaw, err_roll, err_pitch, err_yaw
    
    # Assign the Float64MultiArray object to 'f' as we will have to send data
    # of motor velocities to gazebo in this format
    f = Float64MultiArray()
    
    # Convert the quaternion data to roll, pitch, yaw data
    # The model_states contains the position, orientation, velocities of all 
    # objects in gazebo. In the simulation, there are objects like: ground,
    # Contruction_cone, quadcopter (named as 'Kwad') etc. So 'msg.pose[ind]'
    # will access the 'Kwad' object's pose info i.e the quadcopter's pose.
    ind = msg.name.index('Kwad')
    orientationObj = msg.pose[ind].orientation
    orientationList = [orientationObj.x, orientationObj.y, 
            orientationObj.z, orientationObj.w]
    (roll, pitch, yaw) = (euler_from_quaternion(orientationList))
    
    # Send roll, pitch, yaw data to PID() for attitude-stabilisation, along
    # with 'f', to obtain 'fUpdated'
    # Alternatively, you can add your 'control-file' with other algorithms
    # such as Reinforcement learning, and import the main function here
    # instead of PID().
    (err_roll, err_pitch, err_yaw, fUpdated) = PID(roll, pitch, yaw, f)
    
    # The object args contains the tuple of objects
    # (velPub, err_rollPub, err_pitchPub, err_yawPub)
    # publish the information to namespace.

    args[0].publish(fUpdated)
    args[1].publish(err_roll)
    args[2].publish(err_pitch)
    args[3].publish(err_yaw)


if __name__ == "__main__":

# Initiate the node that will control the gazebo model
    rospy.init_node("Control")

# Initiate publishers that publish errors (roll, pitch,yaw - setpoint) so that
# it can be plotted via rqt_plot /err_<name>  

    err_rollPub = rospy.Publisher('err_roll', Float32, queue_size=1)
    err_pitchPub = rospy.Publisher('err_pitch', Float32, queue_size=1)
    err_yawPub = rospy.Publisher('err_yaw', Float32, queue_size=1)

# Initialte publisher velPub that will publish the velocities of individual
# BLDC motors
    velPub = rospy.Publisher('/Kwad/joint_motor_controller/command',
            Float64MultiArray, queue_size=4)

# Subscribe to /gazebo/model_states to obtain the pose in quaternion form
# Upon receiveing the messages, the objects msg, velPub, err_rollPub, 
# err_pitchPub and err_yawPub are sent to "control_kwad" function.

    PoseSub = rospy.Subscriber('/gazebo/model_states',ModelStates,control_kwad,
            (velPub, err_rollPub, err_pitchPub, err_yawPub))

# Spinning 
    rospy.spin()

