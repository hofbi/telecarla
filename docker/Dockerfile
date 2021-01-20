FROM ros:melodic-robot

ARG CARLA_MAJOR_VERSION='0.9.10'
ARG CARLA_MINOR_VERSION='1'

ENV DEBIAN_FRONTEND='noninteractive'
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python-catkin-tools \
    x11-apps \
    socat \
    wget \
    ubuntu-server \
    ubuntu-desktop && \
    rm -rf /var/lib/apt/lists/*

COPY . /home/catkin_ws/src/telecarla
WORKDIR /home/catkin_ws/src/telecarla
RUN ./install.sh

WORKDIR /home/catkin_ws
RUN wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_${CARLA_MAJOR_VERSION}.${CARLA_MINOR_VERSION}.tar.gz && \
    mkdir -p carla && tar xzf CARLA_${CARLA_MAJOR_VERSION}.${CARLA_MINOR_VERSION}.tar.gz -C carla && \
    python -m easy_install --no-deps carla/PythonAPI/carla/dist/carla-${CARLA_MAJOR_VERSION}-py2.7-linux-x86_64.egg && \
    rm -rf carla CARLA_${CARLA_MAJOR_VERSION}.${CARLA_MINOR_VERSION}.tar.gz
RUN bash -c "source /opt/ros/melodic/setup.bash && source /opt/carla-ros-bridge/melodic/setup.bash && catkin init && catkin build"

ENTRYPOINT [ "src/telecarla/docker/entrypoint.sh" ]
