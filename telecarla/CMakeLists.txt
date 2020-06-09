cmake_minimum_required(VERSION 3.0)
project(telecarla)

find_package(catkin REQUIRED COMPONENTS roslaunch)

# Roslaunch Checks
if(NOT DEFINED ENV{CI_PIPELINE_ID})
  roslaunch_add_file_check(launch)
endif()
