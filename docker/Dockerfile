FROM ros:melodic-robot

ENV DEBIAN_FRONTEND='noninteractive'
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python-catkin-tools \
    x11-apps \
    socat \
    ubuntu-server \
    ubuntu-desktop && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/catkin_ws/src/telecarla
WORKDIR /home/catkin_ws/src/telecarla
RUN ./install.sh

WORKDIR /home/catkin_ws
RUN bash -c "source /opt/ros/melodic/setup.bash && source /opt/carla-ros-bridge/melodic/setup.bash && catkin init && catkin build"

ENTRYPOINT [ "src/telecarla/docker/entrypoint.sh" ]
