import os
import sys
import csv
import time
import argparse
from pathlib import Path

# allow importing neupan and other modules from repository root
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from neupan import neupan
import irsim
import numpy as np


def run_once(env_file: str, planner_file: str, max_steps: int = 1000, point_vel: bool = False):
    """Run a single scenario and collect statistics.

    Returns a dictionary containing success flag, minimum obstacle distance,
    path length, runtime, number of steps, simulated time, and the recorded
    states/actions for each step.
    """
    env = irsim.make(env_file, save_ani=False, full=False, display=False)
    planner = neupan.init_from_yaml(planner_file)

    states: list[list[float]] = []
    actions: list[list[float]] = []

    start = time.time()
    prev_state = None
    path_length = 0.0
    min_dist = float("inf")
    success = False

    for step in range(max_steps):
        state = env.get_robot_state()
        scan = env.get_lidar_scan()

        if point_vel:
            points, point_velocities = planner.scan_to_point_velocity(state, scan)
        else:
            points = planner.scan_to_point(state, scan)
            point_velocities = None

        action, info = planner(state, points, point_velocities)

        states.append(state.squeeze().tolist())
        actions.append(action.squeeze().tolist())

        if prev_state is not None:
            path_length += float(np.linalg.norm(state[:2] - prev_state[:2]))
        prev_state = state.copy()

        min_dist = min(min_dist, planner.min_distance)

        env.step(action)

        if info["arrive"]:
            success = True
            break
        if info["stop"] or env.done():
            break

    runtime = time.time() - start
    sim_time = (step + 1) * planner.dt

    return {
        "success": success,
        "min_dist": float(min_dist),
        "path_length": float(path_length),
        "runtime": float(runtime),
        "steps": step + 1,
        "sim_time": float(sim_time),
        "states": states,
        "actions": actions,
    }


def save_step_log(name: str, states: list[list[float]], actions: list[list[float]], out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    step_file = out_dir / f"{name}_steps.csv"
    with step_file.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["step", "x", "y", "theta", "v", "w"])
        for i, (s, a) in enumerate(zip(states, actions)):
            writer.writerow([i] + s + a)


def evaluate(env_files, planner_files, names, max_steps, point_vel, out_path):
    results = []
    out_dir = Path(out_path).parent

    for env_file, planner_file, name in zip(env_files, planner_files, names):
        res = run_once(env_file, planner_file, max_steps=max_steps, point_vel=point_vel)
        results.append({"scenario": name, **{k: res[k] for k in ["success", "min_dist", "path_length", "runtime", "steps", "sim_time"]}})
        save_step_log(name, res["states"], res["actions"], out_dir)

    success_rate = sum(r["success"] for r in results) / len(results) if results else 0.0

    with open(out_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["scenario", "success", "min_dist", "path_length", "runtime", "steps", "sim_time"])
        writer.writeheader()
        for r in results:
            writer.writerow(r)
        writer.writerow({"scenario": "success_rate", "success": success_rate})

    return results, success_rate


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate NeuPAN planner on given scenarios")
    parser.add_argument("--envs", nargs="+", help="List of environment YAML files", required=True)
    parser.add_argument("--planners", nargs="+", help="List of planner YAML files. If one file is given, it will be reused for all scenarios.", required=True)
    parser.add_argument("--names", nargs="+", help="Optional scenario names", default=None)
    parser.add_argument("--max_steps", type=int, default=1000, help="Maximum steps per scenario")
    parser.add_argument("--point_vel", action="store_true", help="Use scan_to_point_velocity")
    parser.add_argument("--output", type=str, default="evaluation_summary.csv", help="Output CSV for summary results")

    args = parser.parse_args()

    envs = args.envs
    planners = args.planners
    if len(planners) == 1 and len(envs) > 1:
        planners = planners * len(envs)
    if len(envs) != len(planners):
        raise ValueError("Number of envs and planners must match")

    if args.names is None:
        names = [Path(e).parent.name for e in envs]
    else:
        if len(args.names) != len(envs):
            raise ValueError("Number of names must match scenarios")
        names = args.names

    evaluate(envs, planners, names, args.max_steps, args.point_vel, args.output)
