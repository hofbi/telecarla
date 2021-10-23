ARG CARLA_VERSION='0.9.12'

FROM carlasim/carla:${CARLA_VERSION} as carla

FROM ros:noetic-robot

ARG CARLA_VERSION

ENV DEBIAN_FRONTEND='noninteractive'
ENV QT_X11_NO_MITSHM=1

RUN apt-get update && \
    apt-get install --no-install-recommends -y \
    python3-catkin-tools \
    python3-osrf-pycommon \
    x11-apps \
    socat \
    wget \
    ubuntu-server \
    ubuntu-desktop && \
    rm -rf /var/lib/apt/lists/*

COPY --from=carla /home/carla/PythonAPI /opt/carla/PythonAPI
COPY . /home/catkin_ws/src/telecarla
WORKDIR /home/catkin_ws/src/telecarla
RUN ./install.sh

WORKDIR /home/catkin_ws
RUN python3 -m easy_install --no-deps /opt/carla/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg && \
    ln -s /usr/bin/python3 /usr/bin/python && \
    ln -s /home/catkin_ws/src/telecarla/telecarla_scenario_runner/script/launchfile_runner_based_on_adaptation_param.sh /usr/bin/run-scenario

ENV PYTHONPATH=${PYTHONPATH}:/opt/carla/PythonAPI:/opt/carla/PythonAPI/carla/dist/carla-${CARLA_VERSION}-py3.7-linux-x86_64.egg:/opt/carla/PythonAPI/carla/agents:/opt/carla/PythonAPI/carla

RUN bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "src/telecarla/docker/entrypoint.sh" ]
