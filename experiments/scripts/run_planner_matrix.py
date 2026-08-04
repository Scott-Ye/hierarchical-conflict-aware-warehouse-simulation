import argparse
import csv
import glob
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime


BASELINE_GLOBAL_PLANNERS = [
    "LayeredAStar",
    "RRTStar",
    "MARRTStar",
    "INashRRT",
]

ALL_GLOBAL_PLANNERS = BASELINE_GLOBAL_PLANNERS + [
    "LayeredAStarCollisionAware",
    "LayeredAStarReservationAware",
    "LayeredAStarQueueAware",
]

LOCAL_PLANNERS = [
    "DullPlanner",
    "VirtualForcePlanner",
    "RVOPlanner",
    "HRVOPlanner",
    "DDPlanner",
    "FLCPlanner",
]

TASK_MANAGERS = [
    "NaiveTaskManager",
    "CongestionAwareTaskManager",
]


def parse_planner_subset(raw_value, defaults):
    if not raw_value:
        return list(defaults)
    values = [item.strip() for item in raw_value.split(",") if item.strip()]
    invalid = [item for item in values if item not in defaults]
    if invalid:
        raise ValueError("Unknown planners: {}".format(", ".join(invalid)))
    return values


def parse_args():
    parser = argparse.ArgumentParser(description="Run a planner combination matrix for Dorabot Minions.")
    parser.add_argument("--time", type=float, default=1.0, help="Headless simulation time in minutes for each run")
    parser.add_argument("--agent", type=int, default=4, help="Number of agents")
    parser.add_argument("--port-load", type=int, default=2, dest="port_load", help="Number of loading ports")
    parser.add_argument("--port-unload", type=int, default=2, dest="port_unload", help="Number of unloading ports")
    parser.add_argument("--size", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"), default=[20, 20], help="Map width and height")
    parser.add_argument("--resolution", type=int, default=1, help="Grid resolution")
    parser.add_argument("--step", type=int, default=20, help="Simulation steps per second")
    parser.add_argument("--matrix-name", type=str, default="planner_matrix", dest="matrix_name", help="Prefix used for generated summaries")
    parser.add_argument("--timeout", type=int, default=180, help="Per-run timeout in seconds")
    parser.add_argument("--global-planners", type=str, default="", help="Optional comma-separated subset of global planners")
    parser.add_argument("--local-planners", type=str, default="", help="Optional comma-separated subset of local planners")
    parser.add_argument("--task-managers", type=str, default="", help="Optional comma-separated subset of task managers")
    args = parser.parse_args()
    args.global_planner_subset = parse_planner_subset(args.global_planners, ALL_GLOBAL_PLANNERS)
    args.local_planner_subset = parse_planner_subset(args.local_planners, LOCAL_PLANNERS)
    args.task_manager_subset = parse_planner_subset(args.task_managers, TASK_MANAGERS)
    if not args.global_planners:
        args.global_planner_subset = list(BASELINE_GLOBAL_PLANNERS)
    if not args.task_managers:
        args.task_manager_subset = ["NaiveTaskManager"]
    return args


def slugify(value):
    return re.sub(r"[^A-Za-z0-9_-]+", "_", value)


def read_latest_result(results_dir, tag):
    stable_path = os.path.join(results_dir, "{}.json".format(tag))
    if os.path.exists(stable_path):
        with open(stable_path, "r") as handle:
            return json.load(handle)
    pattern = os.path.join(results_dir, "{}_*.json".format(tag))
    matching_files = sorted(glob.glob(pattern), key=os.path.getmtime, reverse=True)
    if not matching_files:
        return None
    with open(matching_files[0], "r") as handle:
        return json.load(handle)


def format_stdout_tail(stdout_text, max_lines=12):
    lines = [line for line in stdout_text.strip().splitlines() if line.strip()]
    return "\n".join(lines[-max_lines:])


def build_command(args, global_planner, local_planner, task_manager, record_tag, results_dir):
    size_width, size_height = args.size
    return [
        sys.executable,
        "simulator.py",
        "-t",
        str(args.time),
        "--default",
        "--agent",
        str(args.agent),
        "--port",
        str(args.port_load),
        str(args.port_unload),
        "--size",
        str(size_width),
        str(size_height),
        "--resolution",
        str(args.resolution),
        "--step",
        str(args.step),
        "--gp",
        global_planner,
        "--lp",
        local_planner,
        "--tm",
        task_manager,
        "--record-tag",
        record_tag,
        "--record-dir",
        results_dir,
    ]


def run_single_combination(src_dir, results_dir, args, global_planner, local_planner, task_manager):
    record_tag = slugify("{}_{}_{}_{}".format(args.matrix_name, global_planner, local_planner, task_manager))
    command = build_command(args, global_planner, local_planner, task_manager, record_tag, results_dir)
    started = time.time()
    try:
        completed = subprocess.run(
            command,
            cwd=src_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=args.timeout,
            check=False,
        )
        runtime = round(time.time() - started, 3)
        summary = read_latest_result(results_dir, record_tag)
        result = {
            "global_planner": global_planner,
            "local_planner": local_planner,
            "task_manager": task_manager,
            "status": "success" if completed.returncode == 0 else "failed",
            "exit_code": completed.returncode,
            "runtime_seconds": runtime,
            "packages_delivered": None,
            "pph": None,
            "total_distance_travelled": None,
            "global_plan_calls": None,
            "replan_events": None,
            "total_collision_events": None,
            "agent_agent_collision_events": None,
            "agent_wall_collision_events": None,
            "summary_file": None,
            "stdout_tail": format_stdout_tail(completed.stdout),
        }
        if summary:
            result["packages_delivered"] = summary.get("packages_delivered")
            result["pph"] = summary.get("pph")
            result["total_distance_travelled"] = summary.get("total_distance_travelled")
            result["global_plan_calls"] = summary.get("global_plan_calls")
            result["replan_events"] = summary.get("replan_events")
            result["total_collision_events"] = summary.get("total_collision_events")
            result["agent_agent_collision_events"] = summary.get("agent_agent_collision_events")
            result["agent_wall_collision_events"] = summary.get("agent_wall_collision_events")
            stable_path = os.path.join(results_dir, "{}.json".format(record_tag))
            if os.path.exists(stable_path):
                result["summary_file"] = os.path.basename(stable_path)
            else:
                matching_files = glob.glob(os.path.join(results_dir, "{}_*.json".format(record_tag)))
                result["summary_file"] = os.path.basename(matching_files[-1]) if matching_files else None
        return result
    except subprocess.TimeoutExpired as exc:
        return {
            "global_planner": global_planner,
            "local_planner": local_planner,
            "task_manager": task_manager,
            "status": "timeout",
            "exit_code": None,
            "runtime_seconds": round(time.time() - started, 3),
            "packages_delivered": None,
            "pph": None,
            "total_distance_travelled": None,
            "global_plan_calls": None,
            "replan_events": None,
            "total_collision_events": None,
            "agent_agent_collision_events": None,
            "agent_wall_collision_events": None,
            "summary_file": None,
            "stdout_tail": format_stdout_tail(exc.stdout or ""),
        }


def write_csv(results_dir, matrix_name, results):
    csv_path = os.path.join(results_dir, "{}_summary.csv".format(matrix_name))
    fieldnames = [
        "global_planner",
        "local_planner",
        "task_manager",
        "status",
        "exit_code",
        "runtime_seconds",
        "packages_delivered",
        "pph",
        "total_distance_travelled",
        "global_plan_calls",
        "replan_events",
        "total_collision_events",
        "agent_agent_collision_events",
        "agent_wall_collision_events",
        "summary_file",
        "stdout_tail",
    ]
    with open(csv_path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)
    return csv_path


def write_markdown(results_dir, matrix_name, args, results):
    markdown_path = os.path.join(results_dir, "{}_summary.md".format(matrix_name))
    successes = [row for row in results if row["status"] == "success"]
    failures = [row for row in results if row["status"] != "success"]
    successes.sort(key=lambda row: ((row["packages_delivered"] or 0), (row["pph"] or 0.0)), reverse=True)

    lines = [
        "# Planner Matrix Summary",
        "",
        "- Matrix name: `{}`".format(matrix_name),
        "- Generated at: `{}`".format(datetime.utcnow().isoformat() + "Z"),
        "- Scenario: `time={} min, agents={}, ports=({},{}) size={}x{}, resolution={}, step={}`".format(
            args.time,
            args.agent,
            args.port_load,
            args.port_unload,
            args.size[0],
            args.size[1],
            args.resolution,
            args.step,
        ),
        "- Total combinations: `{}`".format(len(results)),
        "- Successful runs: `{}`".format(len(successes)),
        "- Failed or timeout runs: `{}`".format(len(failures)),
        "",
        "## Successful Runs",
        "",
        "| Rank | GP | LP | TM | Packages | PPH | Distance | Global Plans | Replans | Collisions | Agent-Agent | Agent-Wall | Runtime(s) |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    for index, row in enumerate(successes, start=1):
        lines.append(
            "| {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} | {} |".format(
                index,
                row["global_planner"],
                row["local_planner"],
                row["task_manager"],
                row["packages_delivered"],
                round(row["pph"] or 0.0, 3),
                row["total_distance_travelled"],
                row["global_plan_calls"],
                row["replan_events"],
                row["total_collision_events"],
                row["agent_agent_collision_events"],
                row["agent_wall_collision_events"],
                row["runtime_seconds"],
            )
        )

    lines.extend([
        "",
        "## Failed Or Timeout Runs",
        "",
        "| GP | LP | TM | Status | Exit Code | Runtime(s) | Notes |",
        "| --- | --- | --- | --- | --- | --- | --- |",
    ])
    for row in failures:
        notes = row["stdout_tail"].replace("\n", " <br> ")
        lines.append(
            "| {} | {} | {} | {} | {} | {} | {} |".format(
                row["global_planner"],
                row["local_planner"],
                row["task_manager"],
                row["status"],
                row["exit_code"],
                row["runtime_seconds"],
                notes,
            )
        )

    with open(markdown_path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    return markdown_path


def main():
    args = parse_args()
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    src_dir = os.path.join(repo_root, "src")
    results_dir = os.path.join(repo_root, "experiments", "results")
    os.makedirs(results_dir, exist_ok=True)

    results = []
    for global_planner in args.global_planner_subset:
        for local_planner in args.local_planner_subset:
            for task_manager in args.task_manager_subset:
                print("Running {} + {} + {}".format(global_planner, local_planner, task_manager))
                result = run_single_combination(src_dir, results_dir, args, global_planner, local_planner, task_manager)
                print("  -> status={}, packages={}, pph={}".format(
                    result["status"],
                    result["packages_delivered"],
                    result["pph"],
                ))
                results.append(result)

    csv_path = write_csv(results_dir, args.matrix_name, results)
    markdown_path = write_markdown(results_dir, args.matrix_name, args, results)
    print("Wrote:")
    print("  {}".format(csv_path))
    print("  {}".format(markdown_path))


if __name__ == "__main__":
    main()
