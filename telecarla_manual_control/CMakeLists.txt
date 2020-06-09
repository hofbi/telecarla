cmake_minimum_required(VERSION 3.0)
project(telecarla_manual_control)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

catkin_python_setup()

catkin_package(CATKIN_DEPENDS rospy)

catkin_install_python(
  PROGRAMS src/telecarla_manual_control_view.py
  src/telecarla_manual_control_ctrl.py DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# Roslaunch Checks
roslaunch_add_file_check(launch)
