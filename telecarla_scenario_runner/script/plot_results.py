"""
Plot results of one or multiple scenario runner evaluations
"""

import argparse
import re
import statistics
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import numpy as np
import tikzplotlib


@dataclass
class ScenarioResult:
    """
    Collection of multiple results for the same scenario
    """

    name: str = ""
    collisions: List[int] = field(default_factory=lambda: [])
    red_lights: List[int] = field(default_factory=lambda: [])
    failures: List[int] = field(default_factory=lambda: [])
    tests: List[int] = field(default_factory=lambda: [])
    durations: List[float] = field(default_factory=lambda: [])

    def __lt__(self, other):
        return self.name < other.name

    def add_result(self, test_suite):
        self.collisions.append(self.parse_test_failure(test_suite, "CollisionTest"))
        self.red_lights.append(
            self.parse_test_failure(test_suite, "RunningRedLightTest")
        )
        self.failures.append(int(test_suite.attrib["failures"]))
        self.tests.append(int(test_suite.attrib["tests"]))
        self.durations.append(float(test_suite.attrib["time"]))

    @property
    def failure_rate(self) -> float:
        return np.sum(self.failures) / np.sum(self.tests)

    @property
    def collision_rate(self) -> float:
        return np.sum(self.collisions) / len(self.collisions)

    @property
    def red_light_rate(self) -> float:
        return np.sum(self.red_lights) / len(self.red_lights)

    @property
    def mean_duration(self) -> float:
        return statistics.mean(self.durations)

    @property
    def std_dev_duration(self) -> float:
        return statistics.stdev(self.durations)

    @staticmethod
    def parse_test_failure(test_suite, failure_condition: str) -> int:
        for test in test_suite:
            failure = test.find("failure")
            if failure is not None and failure_condition in failure.get("message"):
                return int(re.findall(rf"{failure_condition} = (\d+)", failure.text)[0])
        return 0


def plot_results(scenario_results, show_yerr):
    """Plot the results"""
    plt.figure("Scenario Runner Results")
    # plt.rc("text", usetex=True)
    # plt.rc("font", size=15)
    sorted_results = sorted(scenario_results.values())
    durations = [result.mean_duration for result in sorted_results]
    failure_rates = [result.failure_rate for result in sorted_results]
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
            label="Mean Duration ({average:.0f}s)",
        )
        # top_ylim = 130
    else:
        plt.bar(
            indices, durations, align="center", label=f"Mean Duration ({average:.0f}s)"
        )
        # top_ylim = 100
    average = sum(failure_rates) / len(failure_rates) * 100
    rects = plt.bar(
        indices,
        failures,
        align="center",
        label=f"Failure Rate (∅{average:.0f}%)",
    )

    auto_label(rects, failure_rates)
    plt.xlabel("Scenario")
    plt.ylabel("Mean Scenario Duration [s]")
    # plt.ylim(top=top_ylim)
    plt.legend(loc="upper left")

    print(
        f"Duration {sum(durations) / len(durations):.0f}s ({statistics.stdev(durations):.1f}s)"
    )
    print(
        f"Failure Rate {sum(failure_rates) / len(failure_rates) * 100:.0f}%"
        f"({statistics.stdev(failure_rates) * 100:.1f}%)"
    )
    collision_rates = [result.collision_rate for result in sorted_results]
    print(
        f"Collision Rate {sum(collision_rates) / len(collision_rates) * 100:.0f}%"
        f"({statistics.stdev(collision_rates) * 100:.1f}%)"
    )
    red_light_rates = [result.red_light_rate for result in sorted_results]
    print(
        f"Red Light Rate {sum(red_light_rates) / len(red_light_rates) * 100:.0f}%"
        f"({statistics.stdev(red_light_rates) * 100:.1f}%)"
    )


def auto_label(rects, values):
    """Add labels to the bar plot"""
    for index, rect in enumerate(rects):
        plt.annotate(
            f"{100 * float(values[index]):.2f}%",
            xy=(rect.get_x() + rect.get_width() / 2, rect.get_height()),
            xytext=(0, 3),  # 3 points vertical offset
            textcoords="offset points",
            ha="center",
            va="bottom",
        )


def get_scenario_results(scenario_result_files):
    """Parse scenario results from files"""
    scenario_results = {}
    for scenario_result_file in scenario_result_files:
        fix_cdata(scenario_result_file)
        root = ET.parse(scenario_result_file).getroot()
        test_suite = root[0]
        test_name = test_suite.attrib["name"]
        if test_name not in scenario_results:
            scenario_results[test_name] = ScenarioResult(test_name)

        scenario_results[test_name].add_result(test_suite)

    return scenario_results


def fix_cdata(scenario_result_file: Path):
    """
    Currently the scenario runner produces invalid XML CDATA,
    which is fixed by this function to have a valid XML for the parser
    :param scenario_result_file:
    :return:
    """
    file_data = scenario_result_file.read_text()

    file_data = file_data.replace(r"\[CDATA\[", r"[CDATA[")
    file_data = file_data.replace(r"\]\]", "]]")

    scenario_result_file.write_text(file_data)


def get_scenario_result_file_paths(eval_dir: Path) -> List[Path]:
    """Get all scenario result file paths located in the given directory"""
    return [file.absolute() for file in sorted(eval_dir.rglob("*.xml"))]


def parse_arguments():
    """Parse command line arguments"""

    def dir_path(path_string: str) -> Path:
        """Argparse type check if path is a directory"""
        if Path(path_string).absolute().is_dir():
            return Path(path_string)
        raise NotADirectoryError(path_string)

    parser = argparse.ArgumentParser(
        description="Plot the results from the scenario evaluation",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-e",
        "--eval-dir",
        type=dir_path,
        default=Path(__file__).parent.parent / "output",
        help="Path to the scenario runner results",
    )
    parser.add_argument(
        "-o",
        "--out-dir",
        type=dir_path,
        default=Path(__file__).parent.parent / "output",
        help="Path to the output directory",
    )
    parser.add_argument(
        "--yerr", action="store_true", help="Show the standard deviation"
    )
    parser.add_argument("--show", action="store_true", help="Show the plot")
    return parser.parse_args()


def main():
    """main"""
    args = parse_arguments()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    scenario_result_files = get_scenario_result_file_paths(args.eval_dir)
    scenario_results = get_scenario_results(scenario_result_files)
    plot_results(scenario_results, args.yerr)
    tikzplotlib.save(args.out_dir / args.eval_dir.with_suffix(".tex").name)

    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
