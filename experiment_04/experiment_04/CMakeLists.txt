cmake_minimum_required(VERSION 2.8.3)
project(experiment_04)

find_package(catkin REQUIRED COMPONENTS
    rospy
    geometry_msgs
    nav_msgs

)

catkin_package(
    # INCLUDE_DIRS include
    LIBRARIES experiment_04_msg
    CATKIN_DEPENDS rospy std_msgs message_runtime
)

# add_dependencies( TODO ADD HERE ${catkin_EXPORTED_TARGETS})

include_directories(
    ${catkin_INCLUDE_DIRS}
)
