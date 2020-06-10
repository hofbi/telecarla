# TELECARLA Scenario Runner

Customization for the [carla scenario runner](https://github.com/carla-simulator/scenario_runner)

## Setup

1. Download the [Carla Simulator](https://github.com/carla-simulator/carla/releases/latest)
1. Download the [scenario runner](https://github.com/carla-simulator/scenario_runner) and setup the environment

## Run

1. Run the carla simulator
1. Run the scenario evaluator with `python3 script/scenario_evaluation.py`

```shell
usage: scenario_evaluation.py [-h] [-s SCENARIOCLASSES [SCENARIOCLASSES ...]]
                              [-n SCENARIONUMBER] [-o OUTPUTDIR]

Evaluate scenarios using the carla scenario runner

optional arguments:
  -h, --help            show this help message and exit
  -s SCENARIOCLASSES [SCENARIOCLASSES ...],--scenarioClasses SCENARIOCLASSES [SCENARIOCLASSES ...]
                        Scenario classes that should be used for the
                        evaluation (default:
                        ['FollowLeadingVehicleWithObstacle',
                        'StationaryObjectCrossing',
                        'DynamicObjectCrossing',
                        'SignalizedJunctionLeftTurn',
                        'SignalizedJunctionRightTurn',
                        'ManeuverOppositeDirection'])
  -n SCENARIONUMBER, --scenarioNumber SCENARIONUMBER
                        Scenario number that should be used for each scenario
                        of a class (default: 1)
  -o OUTPUTDIR, --outputDir OUTPUTDIR
                        ScenarioRunner output dir (default: ./output)
```

## Scenario end conditions

Some carla scenarios have odd end conditions. A brief overview can be found [here](doc/scenarios.md)
