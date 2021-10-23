# TELECARLA ROS Scenario Evaluation Runner

This node's function is to run the different scenarios in a ROS node to allow the communication between the TeleCarla node and the scenarios that need evaluation.

## Setup

1. Download the [Carla Simulator](https://github.com/carla-simulator/carla/releases/latest)
2. Download the [scenario runner](https://github.com/carla-simulator/scenario_runner) and setup the environment

## Run

### Run Node Locally

To use this ROS node locally, these steps should be followed:

1. Run the simulator: `CarlaUE4`
1. Launch the evaluation node while specifying the arguments if necessary: `roslaunch telecarla_scenario_runner telecarla_scenario_evaluation_runner.launch port:=2000 timeout:=30`
1. Launch the telecarla node. The argument **town** is required. It has to match the value used by the scenario which is guaranteed by the rosparam **/town**:

```shell
# Single View
roslaunch telecarla_scenario_runner telecarla_single_local_scenario_evaluation.launch town:="$(rosparam get /town)" role_name:="hero"

# Multiple Views
roslaunch telecarla_scenario_runner telecarla_multi_local_scenario_evaluation.launch town:="$(rosparam get /town)" role_name:="hero"
```

### Run Node On Server

To use this ROS node on a remote server, these steps should be followed:

1. Ssh to a remote server.
1. Run the simulator: `CarlaUE4 -RenderOffScreen`
1. From the same remote server, run the docker container: `docker-compose run telecarla`
1. Launch the evaluation node while specifying the arguments if necessary: `roslaunch telecarla_scenario_runner telecarla_scenario_evaluation_runner.launch timeout:=30`
1. From another terminal on the same remote server, connect to the docker container on which the node from step 4. is running in order to run the telecarla server node:

```shell
docker ps
docker exec -it <name-of-telecarla-container> /bin/bash

# Single View
roslaunch telecarla_scenario_runner telecarla_single_server_scenario_evaluation.launch town:="$(rosparam get /town)" role_name:="hero"

# Multiple Views
roslaunch telecarla_scenario_runner telecarla_multi_server_scenario_evaluation.launch town:="$(rosparam get /town)" role_name:="hero"
```

6. On the local machine, run the client:

```shell
# Single View
roslaunch telecarla telecarla_single_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero"

# Multiple Views
roslaunch telecarla telecarla_multi_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero"
```

### Further Usages

* It's possible to mix and match the scenarios to include view adaptation or not. This is provided through the use of `script/launchfile_runner_based_on_adaptation_param.sh` and the **scenario_splitting_mode** argument defined in the launch file. `script/launchfile_runner_based_on_adaptation_param.sh` will automatically run the correct server version based on whether view adaptation is required or not.

| Value of scenario_splitting_mode  | Splitting Method |
| ----------- | ----------- |
| 0    | No View Adaptation      |
| 1   | View Adaptation        |
| 2   | 1/2 View Adaptation, 1/2 No View Adaptation  (picked at random)      |

The following steps replace the steps 3., 4. and 5. from the previous section. The **view_adaptation** container is more generic than **telecarla**, i.e you can start the launch files from both the **view_adaptation** package and the telecarla package from the **view_adaptation** docker container.

1. `docker-compose run view_adaptation`
1. `roslaunch telecarla_scenario_runner telecarla_scenario_evaluation_runner.launch scenario_splitting_mode:=2`
1.

```shell
docker ps
docker exec -it <name-of-view_adaptation-container> /bin/bash
run-scenario
```

* It's possible to start the evaluation from a certain scenario index to avoid restarting the evaluation from the beginning. Stopping te scenario in the middle of its execution is not a clean operation and it will yield to execution problems, so it's sometimes necessary to restart to the scenario evaluation node. So this feature makes it easier to do so without the need of going through the already executed scenarios:
`roslaunch telecarla_scenario_runner telecarla_scenario_evaluation_runner.launch scenario_index:=<index>`

### Additional Requirements

* Custom *single_sensors.json* and *multi_sensors.json* files as well the modified telecarla launch files are also included in the evaluation node to account for the necessary changes that will allow the evaluation of the scenarios. One of these necessary changes is the vehicle id which needs to be changed to **"hero"** for the evaluation of the scenarios to function.
* Only OpenScenarios are usable when using ROS bridge, the other available scenarios work under certain conditions e.g when using the `manual_control.py` of *scenario_runner*.
* If the timeout value doesn't seem to change even with different values when calling the launch file, then the `scenario_runner.py` from *scenario_runner* needs to be modified to allow the timeout to vary. The source code currently has a fixed value of 100000.
* An overview of the main scenarios for the evaluation can be checked [here](scenarios/README.md).

### Known Errors

* If the telecarla client does not run and an error with the following message appears,

```shell
ERROR default gstrtspconnection.c:1046:gst_rtsp_connection_connect_with_response: failed to connect
```

then changing the port on which the telecarla client listens, by default on 8551 for the single view client, can solve it:

```shell
# Server Side
roslaunch telecarla_scenario_runner telecarla_single_server_scenario_evaluation.launch town:=$(rosparam get /town) role_name:="hero" port:=<Chosen Port>
# Local Machine
roslaunch telecarla telecarla_single_client.launch host:=<IP-address-of-the-remote-server> role_name:="hero" port:=<Chosen Port>
```

This error is due to the UDP port being held up by a process that was not properly terminated.

* Run `source devel/setup.bash` in every docker container terminal.
