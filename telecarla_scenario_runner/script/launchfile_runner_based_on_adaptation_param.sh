#!/bin/bash

VAL1=$(rosparam get /adaptation)

if [ "$VAL1" = 0 ]; then
    echo "TeleCarla"
    roslaunch telecarla_scenario_runner telecarla_multi_server_scenario_evaluation.launch town:="$(rosparam get /town)" role_name:="hero"
else
    echo "View Adaptation"
    roslaunch view_adaptation view_adaptation_multi_server_scenario_runner.launch town:="$(rosparam get /town)" role_name:="hero"
fi
