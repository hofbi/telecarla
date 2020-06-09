#!/usr/bin/env bash

install_gstreamer_component() {
    COMPONENT=$1

    echo "Install gstreamer component $COMPONENT"
    [[ ! -d ${COMPONENT} ]] && sudo git clone -b "${GST_VERSION:0:4}" https://gitlab.freedesktop.org/gstreamer/"${COMPONENT}".git

    pushd "$COMPONENT" || exit

    sudo make uninstall || true
    sudo ./autogen.sh --disable-gtk-doc
    sudo make
    sudo make install
    popd
}

set -euxo pipefail

GST_VERSION="1.16.1"
INSTALL_DIR="/opt/gst-${GST_VERSION}"

echo "GStreamer ${GST_VERSION} auto installer started"

# Create a log file of the build as well as displaying the build on the tty as it runs
exec > >(tee gstreamer-install-from-src.log)
exec 2>&1

# Go to install dir
sudo mkdir -p ${INSTALL_DIR}
cd ${INSTALL_DIR}

echo '  '
echo '=========== [Install Dependencies] ================================================================='
echo '  '

sudo apt-get install -y \
    autopoint \
    bison \
    flex \
    gtk-doc-tools \
    libasound2-dev \
    libasound2-dev \
    libavcodec-dev \
    libavformat-dev \
    libfontconfig1-dev \
    libfreetype6-dev \
    libswscale-dev \
    libtool \
    libx11-dev \
    libx11-xcb-dev \
    libx264-dev \
    libx265-dev \
    libxcb-glx0-dev \
    libxcb1-dev \
    libxext-dev \
    libxfixes-dev \
    libxi-dev \
    libxrender-dev \
    libxv-dev \
    nvidia-cuda-dev \
    nvidia-cuda-toolkit \
    yasm \
    graphviz

echo '  '
echo '=========== [Install Process] ========================================================================'
echo '  '

install_gstreamer_component gstreamer
install_gstreamer_component gst-plugins-base
install_gstreamer_component gst-plugins-good
install_gstreamer_component gst-plugins-ugly
install_gstreamer_component gst-libav
install_gstreamer_component gst-rtsp-server
install_gstreamer_component gst-plugins-bad

echo '  '
echo '=========== [Finished] ============================================================================='
echo '  '

echo '  '
echo '=========== [Test the version] ====================================================================='
echo '  '

gst-launch-1.0 --version
