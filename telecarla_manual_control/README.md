# TELECARLA Manual Control

Extension of Carla ROS bridge manual control for our teleoperated driving package.

## Run

Make sure the CARLA simulator and [ros-bridge](https://github.com/carla-simulator/ros-bridge) is installed and the simulator is running.

### Ros-bridge

```shell
roslaunch telecarla_manual_control carla_ros_bridge_with_ego_vehicle.launch
```

This launches the carla ros bridge and spawns an ego vehicle using one of our sensor configurations (`config/*.json`)

### Manual Control Client

We split the default `manual_control.py`, one for viewing the camera data and the other for reading control commands. Control commands from keyboard and G29 steering wheel are supported. Both or individuals can be launched running

```shell
# Both
roslaunch telecarla_manual_control telecarla_manual_control.launch

# View only
roslaunch telecarla_manual_control telecarla_manual_control.launch

# Control only
roslaunch telecarla_manual_control telecarla_manual_control.launch
```
