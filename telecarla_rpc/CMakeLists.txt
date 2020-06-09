cmake_minimum_required(VERSION 3.0)
project(telecarla_rpc)

find_package(catkin REQUIRED COMPONENTS roscpp carla_msgs std_msgs roslaunch)

catkin_package(CATKIN_DEPENDS)

add_subdirectory(source)

# Roslaunch Checks
roslaunch_add_file_check(launch)
