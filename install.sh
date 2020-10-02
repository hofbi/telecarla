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

echo "Setup TeleCarla for ROS $ROS_VERSION"

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
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1AF1527DE64CB8D9
sudo add-apt-repository "deb [arch=amd64] http://dist.carla.org/carla $(lsb_release -sc) main"
sudo apt-get update
sudo apt-get install -y carla-ros-bridge

# Carla Scenario Runner
git clone https://github.com/carla-simulator/scenario_runner.git
if [ -z "$PYTHON_VERSION" ]; then
    pip2 install -r scenario_runner/requirements.txt
fi
pip3 install -r scenario_runner/requirements.txt

# TELECARLA GUI
"$SCRIPT_DIR"/telecarla_gui/script/install_dependencies.sh "$PYTHON_SUFFIX"

# TELECARLA RPC
"$SCRIPT_DIR"/telecarla_rpc/script/install.sh

# GStreaming
"$SCRIPT_DIR"/gstreaming/setup/install_gstreamer.sh

# Final Notes
echo "You should add the following commands to your shell config"
echo "source /opt/carla-ros-bridge/$ROS_VERSION/setup.bash"
