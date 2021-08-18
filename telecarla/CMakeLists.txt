cmake_minimum_required(VERSION 3.0)
project(telecarla)

find_package(catkin REQUIRED COMPONENTS roslaunch)

# Roslaunch Checks
roslaunch_add_file_check(launch)
