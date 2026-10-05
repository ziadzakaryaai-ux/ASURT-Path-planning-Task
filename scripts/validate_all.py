#!/usr/bin/env python3
"""Headless validation: run the showcase scenarios, check the path constraints,
and save one plot per scenario to output/ (no display needed).

  python scripts/validate_all.py          # top 6 showcase scenarios
  python scripts/validate_all.py --all    # every scenario (1-24)
"""
from __future__ import annotations

import argparse
import math
import os
import sys

_CURRENT_DIR = os.path.dirname(__file__)
_PROJECT_ROOT = os.path.abspath(os.path.join(_CURRENT_DIR, os.pardir))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from src.path_planning import PathPlanning
from src.scenarios import get_scenario_names, make_scenario

OUT_DIR = os.path.join(_PROJECT_ROOT, "output")
MAX_STEP = 0.5
MIN_LEN, MAX_LEN = 5.0, 10.0

# Showcase set: classic gates, hard stray-cone case, boundary-only, Part 2.
TOP_SCENARIOS = ["3", "10", "11", "13", "21", "24"]


def check_path(scenario, car_pose, path):
    problems = []
    if len(path) < 2:
        return ["path has fewer than 2 points"]
    dx0 = path[0][0] - car_pose.x
    dy0 = path[0][1] - car_pose.y
    if math.hypot(dx0, dy0) > 1e-6:
        problems.append("path does not start at the car pose")
    steps = [math.hypot(b[0] - a[0], b[1] - a[1])
             for a, b in zip(path, path[1:])]
    if max(steps) > MAX_STEP + 1e-6:
        problems.append("step > 0.5 m (max %.3f m)" % max(steps))
    length = sum(steps)
    if not (MIN_LEN - 0.05 <= length <= MAX_LEN + 0.05):
        problems.append("length out of range: %.2f m" % length)
    return problems, length


def plot(scenario, cones, car_pose, path):
    os.makedirs(OUT_DIR, exist_ok=True)
    _, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle=":", linewidth=0.5)
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_title(f"Scenario {scenario}")
    for color, label, c in ((0, "Yellow (Right)", "gold"),
                            (1, "Blue (Left)", "royalblue")):
        xs = [c_.x for c_ in cones if c_.color == color]
        ys = [c_.y for c_ in cones if c_.color == color]
        if xs:
            ax.scatter(xs, ys, c=c, edgecolors="black", label=label)
    ax.scatter([car_pose.x], [car_pose.y], c="red", s=60, marker="o",
               label="Car")
    ax.arrow(car_pose.x, car_pose.y, math.cos(car_pose.yaw),
             math.sin(car_pose.yaw), head_width=0.3, head_length=0.4,
             fc="red", ec="red")
    px = [p[0] for p in path]
    py = [p[1] for p in path]
    ax.plot(px, py, "-", color="limegreen", linewidth=2.0,
            label="Planned Path")
    ax.legend(loc="best")
    plt.savefig(os.path.join(OUT_DIR, f"scenario_{scenario}.png"))
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--all", action="store_true",
                        help="validate every scenario instead of the top 6")
    args = parser.parse_args()
    names = get_scenario_names() if args.all else TOP_SCENARIOS

    n_pass = 0
    for name in names:
        cones, car = make_scenario(name)
        path = PathPlanning(car, cones).generatePath()
        problems, length = check_path(name, car, path)
        plot(name, cones, car, path)
        status = "PASS" if not problems else "FAIL"
        if not problems:
            n_pass += 1
        print(f"{name:>3}  {status}  len={length:5.2f} m  "
              f"pts={len(path):3d}  {', '.join(problems)}")
    print(f"\n{n_pass}/{len(names)} scenarios passed")
    print(f"plots saved to {OUT_DIR}")


if __name__ == "__main__":
    main()
