cmake_minimum_required(VERSION 3.0)
project(telecarla_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation)

add_message_files(DIRECTORY msg FILES TeleopMode.msg FrameStatistics.msg)

generate_messages(DEPENDENCIES telecarla_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
