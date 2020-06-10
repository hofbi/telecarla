#!/usr/bin/env bash

echo "GStreamer auto installer started"

# Create a log file of the build as well as displaying the build on the tty as it runs
exec > >(tee gstreamer-install.log)
exec 2>&1

echo '  '
echo '=========== [Install Process] ========================================================================'
echo '  '

sudo apt-get install -y \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-libav \
    libgstrtspserver-1.0-dev \
    graphviz

echo '  '
echo '=========== [Finished] ============================================================================='
echo '  '

echo '  '
echo '=========== [Test the version] ====================================================================='
echo '  '

gst-launch-1.0 --version
