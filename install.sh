#!/usr/bin/env bash

usage() {
    echo -e "Run: $0 [OPTIONS...] [OTHER ARGUMENTS]"
    echo ""
    echo "options:"
    echo -e "\t--carla-root PATH : Absolute path to the CARLA root folder"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

if [[ $1 == "-h" || $1 == "--help" ]]; then
    usage
    exit 0
elif [[ $1 == "--carla-root" && $# -gt 1 ]]; then
    CARLA_ROOT=$2
    ROS_VERSION='melodic'

    # Create a log file of the installation process as well as displaying it
    exec > >(tee telecarla-install.log)
    exec 2>&1

    echo 'Setup TeleCarla'

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    WS_SRC="${SCRIPT_DIR}"/..

    pushd "$WS_SRC" || exit

    # General Packages
    sudo apt-get update
    sudo apt-get install -y \
        software-properties-common \
        apt-utils \
        python-pip \
        python3-pip \
        python-protobuf

    # Carla Ros Bridge
    sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 81061A1A042F527D
    sudo add-apt-repository "deb [arch=amd64 trusted=yes] http://dist.carla.org/carla-ros-bridge-melodic/ bionic main"
    sudo apt-get update
    sudo apt-get install -y carla-ros-bridge-${ROS_VERSION}

    # Carla Scenario Runner
    git clone https://github.com/carla-simulator/scenario_runner.git
    pip2 install -r scenario_runner/requirements.txt
    pip3 install -r scenario_runner/requirements.txt
    bash scenario_runner/setup_environment.sh --carla-root "$CARLA_ROOT"

    # Tele Carla GUI
    "$SCRIPT_DIR"/telecarla_gui/script/install_dependencies.sh

    # Tele Carla RPC
    "$SCRIPT_DIR"/telecarla_rpc/script/install.sh

    # GStreaming
    "$SCRIPT_DIR"/gstreaming/setup/install_gstreamer.sh

    # Final Notes
    echo "You should add the following commands to your shell config"
    echo "source /opt/carla-ros-bridge/$ROS_VERSION/setup.bash"

    exit 0
else
    usage
    exit 1
fi
