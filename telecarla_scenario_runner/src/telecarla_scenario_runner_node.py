#!/usr/bin/env python
"""
Automatically runs a group of scenarios in a ROS node.
"""

import glob
import subprocess
import sys
from pathlib import Path
from random import shuffle

import ros_compatibility as roscomp
import rospkg
import rospy
from bs4 import BeautifulSoup as bs


class ScenarioRunner:
    """
    Create a scenario object.
    """

    def __init__(self):
        self._host = rospy.get_param("/telecarla_scenario_runner/host", "localhost")
        self._port = rospy.get_param("/telecarla_scenario_runner/port", 2000)
        self._path = rospy.get_param("/telecarla_scenario_runner/scenario_runner_root")
        self._timeout = rospy.get_param("/telecarla_scenario_runner/timeout", 100000)
        self._splitting_mode = rospy.get_param(
            "/telecarla_scenario_runner/scenario_splitting_mode"
        )
        self._output_dir_with_prio = (
            Path(rospy.get_param("/telecarla_scenario_runner/output_dir"))
            / "view_adaptation"
        )
        self._scenario_index = rospy.get_param(
            "/telecarla_scenario_runner/scenario_index", 0
        )

        self._output_dir_with_prio.mkdir(parents=True, exist_ok=True)

    def get_scenarios(self, runner_path):
        """
        Return all openscenario files to run the evaluation on them.
        """

        list_of_openscenario_files = sorted(
            glob.glob(f"{runner_path}/srunner/examples/*.xosc")
        )

        rospack = rospkg.RosPack()
        scenario_runner_path = rospack.get_path("telecarla_scenario_runner")
        list_of_openscenario_files += sorted(
            glob.glob(f"{scenario_runner_path}/scenarios/*.xosc")
        )

        scenario_adaptation_labels = []
        if int(self._splitting_mode) == 2:
            scenario_adaptation_labels += [1] * int(
                len(list_of_openscenario_files) // 2
            )
            scenario_adaptation_labels += [0] * (
                len(list_of_openscenario_files) - len(scenario_adaptation_labels)
            )
            shuffle(scenario_adaptation_labels)
        elif int(self._splitting_mode) == 1:
            scenario_adaptation_labels += [1] * len(list_of_openscenario_files)
        else:
            scenario_adaptation_labels += [0] * len(list_of_openscenario_files)

        return (
            list_of_openscenario_files[self._scenario_index :],
            scenario_adaptation_labels[self._scenario_index :],
        )


def create_town_param(scenario_file):
    """
    Create a ros parameter as /town for telecarla's use.
    """

    with Path(scenario_file).open() as file:

        lines = file.readlines()
        lines = "".join(lines)
        lines_structured = bs(lines, "lxml")
        town = lines_structured.find("logicfile").get("filepath")

    rospy.set_param("/town", town)


def main(args=None):
    """
    Run the available scenarios that respect OpenScenario's notation.
    """

    roscomp.init("This is for the evaluation of scenarios...", args=args)

    scenario_runner = ScenarioRunner()
    scenarios, scenario_adaptation_labels = scenario_runner.get_scenarios(
        scenario_runner._path
    )
    output_dir = scenario_runner._output_dir

    for index, scenario_file in enumerate(scenarios):

        rospy.set_param("/adaptation", scenario_adaptation_labels[index])
        if scenario_adaptation_labels[index] == 1:
            output_dir = scenario_runner._output_dir_with_prio
        else:
            output_dir = scenario_runner._output_dir

        create_town_param(scenario_file)
        ind = scenario_file.rfind("/")
        scenario_name = scenario_file[ind + 1 : -5]
        print(output_dir)

        print(
            "\n==============================================================================\n"
        )
        print(
            "Running scenario %s (%d/%d)" % (scenario_name, index + 1, len(scenarios))
        )

        print("Start your client in a few seconds, after the world has loaded.")

        cmdline = [
            "python3",
            f"{scenario_runner._path}/scenario_runner.py",
            "--openscenario",
            f"{scenario_file}",
            "--timeout",
            str(scenario_runner._timeout),
            "--host",
            scenario_runner._host,
            "--port",
            str(scenario_runner._port),
            "--waitForEgo",
            "--junit",
            "--outputDir",
            output_dir,
            "--output",
        ]
        subprocess.call(cmdline)

        input("\n-> Stop your current client and press enter to continue...")
        print(
            "\n==============================================================================\n"
        )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(1)
