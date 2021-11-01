cmake_minimum_required(VERSION 2.8.3)
project(experiment_05)

find_package(catkin REQUIRED COMPONENTS
    # OpenCV
    cv_bridge
    rospy
    geometry_msgs
    sensor_msgs
    nav_msgs
)

catkin_package(
    # INCLUDE_DIRS include
    # LIBRARIES experiment_05_msg
    # CATKIN_DEPENDS rospy std_msgs message_runtime
    CATKIN_DEPENDS rospy std_msgs 
)

# target_link_libraries ( my_exec ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} )

include_directories(
    # ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
