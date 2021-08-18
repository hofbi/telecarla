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

# General Packages
sudo apt-get update
sudo apt-get install -y \
    clang-tidy-10 \
    python3-pip

if [ -z "$PYTHON_VERSION" ]; then
    sudo apt-get install -y \
        apt-utils \
        python-pip \
        python-protobuf \
        software-properties-common
fi

# Carla Ros Bridge
sudo apt-get install -y \
    ros-$ROS_VERSION-opencv-apps \
    ros-$ROS_VERSION-ackermann-msgs \
    ros-$ROS_VERSION-derived-object-msgs \
    ros-$ROS_VERSION-pcl-ros
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git
rm -rf "$WS_SRC"/ros-bridge/rqt_carla_control "$WS_SRC"/ros-bridge/rviz_carla_plugin
rosdep update
rosdep install --from-paths src --ignore-src -r
pip3 install -r "$WS_SRC"/ros-bridge/requirements.txt
if [ -z "$PYTHON_VERSION" ]; then
    pip install -r "$WS_SRC"/ros-bridge/requirements.txt
fi

# Carla Scenario Runner
git clone https://github.com/carla-simulator/scenario_runner.git
if [ -z "$PYTHON_VERSION" ]; then
    pip2 install -r "$WS_SRC"/scenario_runner/requirements.txt
fi
pip3 install -r "$WS_SRC"/scenario_runner/requirements.txt

# TELECARLA GUI
"$SCRIPT_DIR"/telecarla_gui/script/install_dependencies.sh "$PYTHON_SUFFIX"

# TELECARLA RPC
"$SCRIPT_DIR"/telecarla_rpc/script/install.sh

# GStreaming
"$SCRIPT_DIR"/gstreaming/setup/install_gstreamer.sh

echo "Finished TELECARLA Setup"
