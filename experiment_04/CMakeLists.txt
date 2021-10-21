cmake_minimum_required(VERSION 2.8.3)
project(experiment_04)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    nav_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)
