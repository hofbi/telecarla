# TELECARLA

The main module for carla teleoperated driving contains a collection of launch files for running in different modes (local and remote) and camera (single front camera and 6 cameras) setups.

## Run

### Local Driving

The *local* setup runs the `carla_ros_bridge` and our teleop GUI locally.

```shell
# Single front camera
roslaunch telecarla telecarla_single_local.launch

# Multiple cameras
roslaunch telecarla telecarla_multi_local.launch
```

### Remote Driving

The *remote* setup launches the `carla_ros_bridge` and our streaming and RPC servers on the server side. On the client side, the respective clients connect to them and the teleop GUI subscribes to these topics.

```shell
# Single front camera
roslaunch telecarla telecarla_single_server.launch
roslaunch telecarla telecarla_single_client.launch host:=<server_ip>

# Multiple cameras
roslaunch telecarla telecarla_multi_server.launch
roslaunch telecarla telecarla_multi_client.launch  host:=<server_ip>
```
