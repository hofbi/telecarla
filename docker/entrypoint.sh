#!/bin/bash
set -e

# setup ros environment
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1090
source "/opt/carla-ros-bridge/$ROS_DISTRO/setup.bash"
# shellcheck disable=SC1091
source "/home/catkin_ws/devel/setup.bash"

exec "$@"
