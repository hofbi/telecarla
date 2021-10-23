cmake_minimum_required(VERSION 3.0)
project(telecarla_scenario_runner)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs roslaunch)

# Roslaunch Checks
roslaunch_add_file_check(launch)
