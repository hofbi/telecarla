"""
Plot results of one or multiple scenario runner evaluations
"""

import argparse
import os
import re
import statistics
import sys
import xml.etree.cElementTree as ET

import matplotlib.pyplot as plt
import numpy as np


class ScenarioResult:
    """
    Collection of multiple results for the same scenario
    """

    def __init__(self, name):
        self._name = name
        self._collisions = []
        self._durations = []

    def __lt__(self, other):
        return self.name < other.name

    def add_result(self, collision, duration):
        self._collisions.append(collision)
        self._durations.append(duration)

    @property
    def collision_rate(self):
        return np.sum(self.collisions) / len(self.collisions)

    @property
    def mean_duration(self):
        return statistics.mean(self.durations)

    @property
    def std_dev_duration(self):
        return statistics.stdev(self.durations)

    @property
    def collisions(self):
        return self._collisions

    @property
    def durations(self):
        return self._durations

    @property
    def name(self):
        return self._name


def main():
    """main"""
    parser = argparse.ArgumentParser(
        description="Plot the results from the scenario evaluation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--eval_dir",
        type=str,
        default=os.path.join(
            os.path.dirname(os.path.abspath(sys.argv[0])), "..", "output"
        ),
        help="Path to the scenario runner results",
    )
    parser.add_argument(
        "--out_dir",
        type=str,
        default=os.path.join(
            os.path.dirname(os.path.abspath(sys.argv[0])), "..", "output"
        ),
        help="Path to the output directory",
    )
    parser.add_argument(
        "--yerr", action="store_true", help="Show the standard deviation"
    )
    parser.add_argument("--show", action="store_true", help="Show the plot")

    args = parser.parse_args()

    scenario_result_files = get_scenario_result_file_paths(args.eval_dir)
    scenario_results = get_scenario_results(scenario_result_files)
    plot_results(scenario_results, args.yerr)
    plt.savefig(os.path.join(args.out_dir, os.path.basename(args.eval_dir)))

    if args.show:
        plt.show()


def plot_results(scenario_results, show_yerr):
    """
    Plot the results
    :param scenario_results:
    :param show_yerr:
    :return:
    """
    plt.figure("Scenario Runner Results")
    sorted_results = sorted(scenario_results.values())
    durations = [result.mean_duration for result in sorted_results]
    failure_rates = [result.collision_rate for result in sorted_results]
    failures = [a * b for a, b in zip(durations, failure_rates)]
    indices = range(1, len(scenario_results) + 1)

    average = sum(durations) / len(durations)
    if show_yerr:
        duration_errors = [result.std_dev_duration for result in sorted_results]
        plt.bar(
            indices,
            durations,
            yerr=duration_errors,
            align="center",
            label="Mean Duration (∅%.0fs)" % average,
        )
    else:
        plt.bar(
            indices, durations, align="center", label="Mean Duration (∅%.0fs)" % average
        )
    average = sum(failure_rates) / len(failure_rates) * 100
    rects = plt.bar(
        indices,
        failures,
        align="center",
        label="Collision Rate (∅{0:.0f}%)".format(average),
    )

    auto_label(rects, failure_rates)
    plt.xlabel("Scenario")
    plt.ylabel("Mean Scenario Duration [s]")
    plt.legend()


def auto_label(rects, values):
    """
    Add labels to the bar plot
    :param rects:
    :param values:
    :return:
    """
    for index, rect in enumerate(rects):
        plt.annotate(
            "{0:.2f}%".format(100 * float(values[index])),
            xy=(rect.get_x() + rect.get_width() / 2, rect.get_height()),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords="offset points",
            ha="center",
            va="bottom",
        )


def get_scenario_results(scenario_result_files):
    """
    Parse scenario results from files
    :param scenario_result_files:
    :return:
    """
    scenario_results = {}
    for scenario_result_file in scenario_result_files:
        fix_cdata(scenario_result_file)
        root = ET.parse(scenario_result_file).getroot()
        test_suite = root[0]
        test_name = test_suite.attrib["name"]
        if test_name not in scenario_results:
            scenario_results[test_name] = ScenarioResult(test_name)

        scenario_results[test_name].add_result(
            has_collision(test_suite), get_duration(test_suite)
        )

    return scenario_results


def fix_cdata(scenario_result_file):
    """
    Currently the scenario runner produces invalid XML CDATA,
    which is fixed by this function to have a valid XML for the parser
    :param scenario_result_file:
    :return:
    """
    with open(scenario_result_file, "r") as file:
        file_data = file.read()

    file_data = file_data.replace(r"\[CDATA\[", r"[CDATA[")
    file_data = file_data.replace(r"\]\]", "]]")

    with open(scenario_result_file, "w") as file:
        file.write(file_data)


def has_collision(test_suite):
    """
    Check if a run has a colition
    :param test_suite:
    :return:
    """
    return test_suite[0].find("failure") is not None


def get_duration(test_suite):
    """
    Get the duration of a single run
    :param test_suite:
    :return:
    """
    duration_case = test_suite[1]
    if len(list(duration_case)) > 0:
        text = duration_case[0].text
    else:
        text = duration_case.text
    durations = re.findall(r"[-+]?\d*\.\d+|\d+", text)
    return float(durations[0])


def get_scenario_result_file_paths(eval_dir):
    """
    Get all scenario result file paths located in the given directory
    :param eval_dir:
    :return:
    """
    scenario_results = []
    for root, dirs, files in os.walk(eval_dir):
        for scenario_file in files:
            if scenario_file.endswith(".xml"):
                scenario_results.append(
                    os.path.abspath(os.path.join(root, scenario_file))
                )

    return scenario_results


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
