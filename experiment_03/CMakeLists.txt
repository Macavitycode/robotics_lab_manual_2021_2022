cmake_minimum_required(VERSION 2.8.3)
project(experiment_03)

find_package(catkin REQUIRED COMPONENTS
    xacro
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)
