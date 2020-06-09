#!/usr/bin/env python3
"""
Automatically run a sequence of scenarios
"""

import argparse
import os
import subprocess

SCENARIO_RUNNER = os.path.join(os.environ["ROOT_SCENARIO_RUNNER"], "scenario_runner.py")
DEFAULT_OUTPUT_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "output")
)


def main():
    """main"""
    args = parse_arguments()

    for index, scenario_class in enumerate(args.scenarioClasses):
        scenario = scenario_class + "_" + str(args.scenarioNumber)
        print(
            "\n==============================================================================\n"
        )
        print(
            "Running scenario %s (%d/%d)"
            % (scenario, index + 1, len(args.scenarioClasses))
        )
        print("Start your client in a few seconds, after the world has loaded.")

        subprocess.call(
            [
                "python3",
                SCENARIO_RUNNER,
                "--scenario",
                scenario,
                "--waitForEgo",
                "--junit",
                "--outputDir",
                args.outputDir,
                "--reloadWorld",
            ]
        )

        input("\n-> Stop your current client and press enter to continue...")
        print(
            "\n==============================================================================\n"
        )


def parse_arguments():
    """
    Parse command line arguments
    :return:
    """
    parser = argparse.ArgumentParser(
        description="Evaluate scenarios using the carla scenario runner",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-s",
        "--scenarioClasses",
        type=str,
        nargs="+",
        help="Scenario classes that should be used for the evaluation",
        default=[
            "FollowLeadingVehicleWithObstacle",
            "StationaryObjectCrossing",
            "DynamicObjectCrossing",
            "SignalizedJunctionLeftTurn",
            "SignalizedJunctionRightTurn",
            "ManeuverOppositeDirection",
        ],
    )
    parser.add_argument(
        "-n",
        "--scenarioNumber",
        type=int,
        default=1,
        help="Scenario number that should be used for each scenario of a class",
    )
    parser.add_argument(
        "-o",
        "--outputDir",
        default=DEFAULT_OUTPUT_DIR,
        help="ScenarioRunner output dir",
    )
    return parser.parse_args()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
