cmake_minimum_required(VERSION 2.8.3)
project(experiment_03)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
