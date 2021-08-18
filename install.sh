#!/usr/bin/env bash

# Supported versions ['melodic', 'noetic']
# shellcheck disable=SC1091
if [ "$(
    . /etc/os-release
    echo "$VERSION_ID"
)" = "20.04" ]; then
    ROS_VERSION='noetic'
    PYTHON_SUFFIX=3
else
    PYTHON_SUFFIX=""
    ROS_VERSION='melodic'
fi

# Create a log file of the installation process as well as displaying it
exec > >(tee telecarla-install.log)
exec 2>&1

echo "Setup TELECARLA for ROS $ROS_VERSION"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_SRC="${SCRIPT_DIR}"/..

pushd "$WS_SRC" || exit

echo "Installing General Packages"
sudo apt-get update
sudo apt-get install -y \
    git \
    software-properties-common \
    apt-utils

sudo apt-get install -y \
    python3-pip \
    clang-tidy-10

if [ -z "$PYTHON_SUFFIX" ]; then
    sudo apt-get install -y \
        python-pip \
        python-protobuf
fi

echo "Installing Carla Ros Bridge"
sudo apt-get install -y \
    ros-$ROS_VERSION-opencv-apps \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-vision-opencv \
    ros-$ROS_VERSION-rospy-message-converter \
    ros-$ROS_VERSION-rviz \
    ros-$ROS_VERSION-rqt-image-view \
    ros-$ROS_VERSION-pcl-ros
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
rm -rf "$WS_SRC"/ros-bridge/rqt_carla_control \
    "$WS_SRC"/ros-bridge/rviz_carla_plugin \
    "$WS_SRC"/ros-bridge/pcl_recorder \
    "$WS_SRC"/ros-bridge/carla_ad_demo
rosdep update
rosdep install --from-paths src --ignore-src -r
pip3 install -r "$WS_SRC"/ros-bridge/requirements.txt
if [ -z "$PYTHON_SUFFIX" ]; then
    pip install -r "$WS_SRC"/ros-bridge/requirements.txt
fi

echo "Installing Carla Scenario Runner"
git clone https://github.com/carla-simulator/scenario_runner.git
if [ -z "$PYTHON_SUFFIX" ]; then
    pip2 install -r "$WS_SRC"/scenario_runner/requirements.txt
fi
pip3 install -r "$WS_SRC"/scenario_runner/requirements.txt

echo "Installing TELECARLA GUI dependencies"
"$SCRIPT_DIR"/telecarla_gui/script/install_dependencies.sh "$PYTHON_SUFFIX"

echo "Installing TELECARLA RPC dependencies"
"$SCRIPT_DIR"/telecarla_rpc/script/install.sh

echo "Installing GStreaming dependencies"
"$SCRIPT_DIR"/gstreaming/setup/install_gstreamer.sh

echo "Finished TELECARLA Setup"
