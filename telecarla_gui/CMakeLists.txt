cmake_minimum_required(VERSION 3.0)
project(telecarla_gui)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs cv_bridge roslaunch
                                        carla_msgs telecarla_msgs)

# Package
catkin_package(CATKIN_DEPENDS)

# Source
add_subdirectory(source)

# Test
add_subdirectory(test)
