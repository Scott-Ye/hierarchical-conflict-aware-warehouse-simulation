import csv
import json
import re
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_CSV = REPO_ROOT / "experiments" / "results" / "run_summaries.csv"
FINAL_REPORT_DIR = REPO_ROOT / "experiments" / "results" / "final_report"
TABLES_DIR = FINAL_REPORT_DIR / "tables"
PLOTS_DIR = FINAL_REPORT_DIR / "plots"

STAGE_ORDER = [
    ("LayeredAStar", "baseline"),
    ("LayeredAStarCollisionAware", "v1"),
    ("LayeredAStarReservationAware", "v2"),
    ("LayeredAStarQueueAware", "v3"),
]
SCENARIO_ORDER = ["small", "medium", "high"]
LOCAL_PLANNER_ORDER = ["DullPlanner", "VirtualForcePlanner"]


def to_number(value):
    if value in ("", None):
        return None
    try:
        if "." in str(value):
            return float(value)
        return int(value)
    except ValueError:
        return value


def load_csv_rows(path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        rows = []
        for row in csv.DictReader(handle):
            parsed = {}
            for key, value in row.items():
                parsed[key] = to_number(value)
            rows.append(parsed)
        return rows


def write_csv(path, rows, fieldnames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def planner_to_stage(global_planner):
    for planner_name, stage_name in STAGE_ORDER:
        if planner_name == global_planner:
            return stage_name
    return global_planner


def parse_stage_chain_rows(rows):
    pattern = re.compile(r"^stage_chain_(small|medium|high)_60s_")
    stage_rows = []
    by_key = {}
    for row in rows:
        record_tag = str(row["record_tag"])
        match = pattern.match(record_tag)
        if not match:
            continue
        scenario = match.group(1)
        stage = planner_to_stage(row["global_planner"])
        normalized = {
            "record_tag": record_tag,
            "scenario": scenario,
            "stage": stage,
            "global_planner": row["global_planner"],
            "local_planner": row["local_planner"],
            "task_manager": row["task_manager"],
            "packages_delivered": row["packages_delivered"],
            "pph": row["pph"],
            "total_collision_events": row["total_collision_events"],
            "agent_agent_collision_events": row["agent_agent_collision_events"],
            "replan_events": row["replan_events"],
            "real_runtime_seconds": row["real_runtime_seconds"],
            "status": "available",
        }
        stage_rows.append(normalized)
        by_key[(scenario, stage, row["local_planner"])] = normalized

    expected_rows = []
    for scenario in SCENARIO_ORDER:
        for _, stage in STAGE_ORDER:
            for local_planner in LOCAL_PLANNER_ORDER:
                row = by_key.get((scenario, stage, local_planner))
                if row is not None:
                    expected_rows.append(row)
                    continue
                expected_rows.append(
                    {
                        "record_tag": "",
                        "scenario": scenario,
                        "stage": stage,
                        "global_planner": "",
                        "local_planner": local_planner,
                        "task_manager": "NaiveTaskManager",
                        "packages_delivered": "",
                        "pph": "",
                        "total_collision_events": "",
                        "agent_agent_collision_events": "",
                        "replan_events": "",
                        "real_runtime_seconds": "",
                        "status": "pending",
                    }
                )
    return expected_rows


def build_markdown_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(header, "")) for header in headers) + " |")
    return "\n".join(lines)


def write_stage_chain_assets(rows):
    fieldnames = [
        "scenario",
        "stage",
        "global_planner",
        "local_planner",
        "task_manager",
        "packages_delivered",
        "pph",
        "total_collision_events",
        "agent_agent_collision_events",
        "replan_events",
        "real_runtime_seconds",
        "status",
        "record_tag",
    ]
    write_csv(TABLES_DIR / "stage_chain_all.csv", rows, fieldnames)

    for local_planner in LOCAL_PLANNER_ORDER:
        subset = [row for row in rows if row["local_planner"] == local_planner]
        csv_name = "stage_chain_{}.csv".format(local_planner.replace("Planner", "").lower())
        md_name = "stage_chain_{}.md".format(local_planner.replace("Planner", "").lower())
        write_csv(TABLES_DIR / csv_name, subset, fieldnames)

        table_rows = []
        for row in subset:
            table_rows.append(
                {
                    "Scenario": row["scenario"],
                    "Stage": row["stage"],
                    "Planner": row["global_planner"],
                    "Packages": row["packages_delivered"],
                    "PPH": row["pph"],
                    "Collisions": row["total_collision_events"],
                    "Replans": row["replan_events"],
                    "Runtime(s)": row["real_runtime_seconds"],
                    "Status": row["status"],
                }
            )
        markdown = [
            "# Stage Chain ({})".format(local_planner),
            "",
            build_markdown_table(
                ["Scenario", "Stage", "Planner", "Packages", "PPH", "Collisions", "Replans", "Runtime(s)", "Status"],
                table_rows,
            ),
            "",
        ]
        write_text(TABLES_DIR / md_name, "\n".join(markdown))

    plot_stage_chain(rows, "DullPlanner", PLOTS_DIR / "stage_chain_dull.png")
    plot_stage_chain(rows, "VirtualForcePlanner", PLOTS_DIR / "stage_chain_virtualforce.png")


def plot_stage_chain(rows, local_planner, output_path):
    subset = [row for row in rows if row["local_planner"] == local_planner]
    available = [row for row in subset if row["status"] == "available"]
    if not available:
        return

    scenario_labels = SCENARIO_ORDER
    stage_labels = [stage for _, stage in STAGE_ORDER]
    width = 0.18
    x_positions = list(range(len(scenario_labels)))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))

    for stage_index, stage in enumerate(stage_labels):
        pph_values = []
        collision_values = []
        for scenario in scenario_labels:
            row = next((item for item in subset if item["scenario"] == scenario and item["stage"] == stage), None)
            pph_values.append(float(row["pph"]) if row and row["status"] == "available" and row["pph"] != "" else 0.0)
            collision_values.append(
                float(row["total_collision_events"])
                if row and row["status"] == "available" and row["total_collision_events"] != ""
                else 0.0
            )
        offsets = [value + (stage_index - 1.5) * width for value in x_positions]
        axes[0].bar(offsets, pph_values, width=width, label=stage)
        axes[1].bar(offsets, collision_values, width=width, label=stage)

    for axis, title, ylabel in [
        (axes[0], "{}: throughput".format(local_planner), "PPH"),
        (axes[1], "{}: collisions".format(local_planner), "Collision events"),
    ]:
        axis.set_xticks(x_positions)
        axis.set_xticklabels(scenario_labels)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def parse_task_manager_rows(rows):
    pattern = re.compile(r"^task_manager_(medium|high)_60s_")
    parsed = []
    for row in rows:
        record_tag = str(row["record_tag"])
        match = pattern.match(record_tag)
        if not match:
            continue
        parsed.append(
            {
                "record_tag": record_tag,
                "scenario": match.group(1),
                "global_planner": row["global_planner"],
                "local_planner": row["local_planner"],
                "task_manager": row["task_manager"],
                "packages_delivered": row["packages_delivered"],
                "pph": row["pph"],
                "total_collision_events": row["total_collision_events"],
                "replan_events": row["replan_events"],
                "real_runtime_seconds": row["real_runtime_seconds"],
            }
        )
    return sorted(parsed, key=lambda item: (SCENARIO_ORDER.index(item["scenario"]), item["local_planner"], item["task_manager"]))


def write_task_manager_assets(rows):
    fieldnames = [
        "scenario",
        "global_planner",
        "local_planner",
        "task_manager",
        "packages_delivered",
        "pph",
        "total_collision_events",
        "replan_events",
        "real_runtime_seconds",
        "record_tag",
    ]
    write_csv(TABLES_DIR / "task_manager_comparison.csv", rows, fieldnames)

    markdown_rows = [
        {
            "Scenario": row["scenario"],
            "Planner": row["global_planner"],
            "Local": row["local_planner"],
            "Task Manager": row["task_manager"],
            "Packages": row["packages_delivered"],
            "PPH": row["pph"],
            "Collisions": row["total_collision_events"],
            "Replans": row["replan_events"],
            "Runtime(s)": row["real_runtime_seconds"],
        }
        for row in rows
    ]
    markdown = [
        "# Task Manager Comparison",
        "",
        build_markdown_table(
            ["Scenario", "Planner", "Local", "Task Manager", "Packages", "PPH", "Collisions", "Replans", "Runtime(s)"],
            markdown_rows,
        ),
        "",
    ]
    write_text(TABLES_DIR / "task_manager_comparison.md", "\n".join(markdown))
    plot_task_manager(rows, PLOTS_DIR / "task_manager_comparison.png")


def plot_task_manager(rows, output_path):
    if not rows:
        return
    labels = [
        "{}\n{}\n{}".format(row["scenario"], row["local_planner"].replace("Planner", ""), row["task_manager"].replace("TaskManager", ""))
        for row in rows
    ]
    pph_values = [float(row["pph"] or 0.0) for row in rows]
    collision_values = [float(row["total_collision_events"] or 0.0) for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    axes[0].bar(labels, pph_values, color="#4e79a7")
    axes[0].set_ylabel("PPH")
    axes[0].set_title("Task manager comparison")
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(labels, collision_values, color="#e15759")
    axes[1].set_ylabel("Collision events")
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].tick_params(axis="x", rotation=20)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def parse_planner_matrix_rows(rows):
    parsed = []
    for row in rows:
        record_tag = str(row["record_tag"])
        if not record_tag.startswith("planner_matrix_baseline_60s_"):
            continue
        parsed.append(
            {
                "record_tag": record_tag,
                "global_planner": row["global_planner"],
                "local_planner": row["local_planner"],
                "task_manager": row["task_manager"],
                "packages_delivered": row["packages_delivered"],
                "pph": row["pph"],
                "total_collision_events": row["total_collision_events"],
                "agent_agent_collision_events": row["agent_agent_collision_events"],
                "replan_events": row["replan_events"],
                "real_runtime_seconds": row["real_runtime_seconds"],
            }
        )
    return sorted(parsed, key=lambda item: (-float(item["pph"] or 0.0), float(item["total_collision_events"] or 0.0), item["record_tag"]))


def write_planner_matrix_assets(rows):
    fieldnames = [
        "global_planner",
        "local_planner",
        "task_manager",
        "packages_delivered",
        "pph",
        "total_collision_events",
        "agent_agent_collision_events",
        "replan_events",
        "real_runtime_seconds",
        "record_tag",
    ]
    write_csv(TABLES_DIR / "planner_matrix_baseline.csv", rows, fieldnames)

    markdown_rows = [
        {
            "Global": row["global_planner"],
            "Local": row["local_planner"],
            "Packages": row["packages_delivered"],
            "PPH": row["pph"],
            "Collisions": row["total_collision_events"],
            "AA Collisions": row["agent_agent_collision_events"],
            "Replans": row["replan_events"],
            "Runtime(s)": row["real_runtime_seconds"],
        }
        for row in rows
    ]
    markdown = [
        "# Baseline Planner Matrix",
        "",
        build_markdown_table(
            ["Global", "Local", "Packages", "PPH", "Collisions", "AA Collisions", "Replans", "Runtime(s)"],
            markdown_rows,
        ),
        "",
    ]
    write_text(TABLES_DIR / "planner_matrix_baseline.md", "\n".join(markdown))
    plot_planner_matrix(rows, PLOTS_DIR / "planner_matrix_baseline.png")


def plot_planner_matrix(rows, output_path):
    if not rows:
        return
    labels = ["{}\n{}".format(row["global_planner"], row["local_planner"].replace("Planner", "")) for row in rows]
    pph_values = [float(row["pph"] or 0.0) for row in rows]
    collision_values = [float(row["total_collision_events"] or 0.0) for row in rows]

    fig, ax = plt.subplots(figsize=(10, max(4.5, len(rows) * 0.42)))
    y_positions = list(range(len(rows)))
    bars = ax.barh(y_positions, pph_values, color="#59a14f")
    ax.set_yticks(y_positions)
    ax.set_yticklabels(labels)
    ax.set_xlabel("PPH")
    ax.set_title("Baseline planner matrix")
    ax.grid(axis="x", alpha=0.25)

    for index, bar in enumerate(bars):
        ax.text(
            bar.get_width() + 4,
            bar.get_y() + bar.get_height() / 2.0,
            "coll={}".format(int(collision_values[index])),
            va="center",
            fontsize=8,
        )

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def parse_showcase_rows():
    rows = []
    for path in sorted(FINAL_REPORT_DIR.rglob("*.json")):
        if path.name in {"run_summaries.jsonl"}:
            continue
        if path.name.startswith("run_summaries"):
            continue
        data = json.loads(path.read_text(encoding="utf-8"))
        rows.append(
            {
                "artifact": path.stem,
                "relative_path": str(path.relative_to(REPO_ROOT)).replace("\\", "/"),
                "global_planner": data.get("global_planner"),
                "local_planner": data.get("local_planner"),
                "task_manager": data.get("task_manager"),
                "agent_count": data.get("agent_count"),
                "map_size": "{}x{}".format(data.get("map_width"), data.get("map_height")),
                "packages_delivered": data.get("packages_delivered"),
                "pph": data.get("pph"),
                "total_collision_events": data.get("total_collision_events"),
                "replan_events": data.get("replan_events"),
                "real_runtime_seconds": data.get("real_runtime_seconds"),
            }
        )
    return sorted(rows, key=lambda item: (0 if item["artifact"].startswith("report_") else 1, -float(item["pph"] or 0.0), item["artifact"]))


def write_showcase_assets(rows):
    fieldnames = [
        "artifact",
        "global_planner",
        "local_planner",
        "task_manager",
        "agent_count",
        "map_size",
        "packages_delivered",
        "pph",
        "total_collision_events",
        "replan_events",
        "real_runtime_seconds",
        "relative_path",
    ]
    write_csv(TABLES_DIR / "showcase_runs.csv", rows, fieldnames)

    markdown_rows = [
        {
            "Artifact": row["artifact"],
            "Agents": row["agent_count"],
            "Map": row["map_size"],
            "Packages": row["packages_delivered"],
            "PPH": row["pph"],
            "Collisions": row["total_collision_events"],
            "Replans": row["replan_events"],
            "Runtime(s)": row["real_runtime_seconds"],
        }
        for row in rows
    ]
    markdown = [
        "# Showcase And Smoke Runs",
        "",
        build_markdown_table(
            ["Artifact", "Agents", "Map", "Packages", "PPH", "Collisions", "Replans", "Runtime(s)"],
            markdown_rows,
        ),
        "",
    ]
    write_text(TABLES_DIR / "showcase_runs.md", "\n".join(markdown))
    plot_showcase(rows, PLOTS_DIR / "showcase_runs.png")


def plot_showcase(rows, output_path):
    if not rows:
        return
    labels = [row["artifact"].replace("report_", "").replace("smoke_", "smoke ") for row in rows]
    pph_values = [float(row["pph"] or 0.0) for row in rows]
    collision_values = [float(row["total_collision_events"] or 0.0) for row in rows]

    fig, axes = plt.subplots(2, 1, figsize=(10, 6.5), sharex=True)
    axes[0].bar(labels, pph_values, color="#9c755f")
    axes[0].set_ylabel("PPH")
    axes[0].set_title("Maintenance branch showcase")
    axes[0].grid(axis="y", alpha=0.25)

    axes[1].bar(labels, collision_values, color="#e15759")
    axes[1].set_ylabel("Collision events")
    axes[1].grid(axis="y", alpha=0.25)
    axes[1].tick_params(axis="x", rotation=20)

    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def build_pending_lines(stage_rows):
    pending = [row for row in stage_rows if row["status"] == "pending"]
    grouped = defaultdict(list)
    for row in pending:
        grouped[row["local_planner"]].append("{}-{}".format(row["scenario"], row["stage"]))
    if not pending:
        return ["- stage_chain 主线矩阵已全部落盘。"]
    lines = []
    for local_planner in LOCAL_PLANNER_ORDER:
        items = grouped.get(local_planner, [])
        if items:
            lines.append("- {} pending: {}".format(local_planner, ", ".join(items)))
    return lines


def get_row(stage_rows, scenario, stage, local_planner):
    return next(
        (
            row
            for row in stage_rows
            if row["scenario"] == scenario and row["stage"] == stage and row["local_planner"] == local_planner and row["status"] == "available"
        ),
        None,
    )


def write_summary(stage_rows, task_rows, planner_rows, showcase_rows):
    generated_at = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")

    baseline_small = get_row(stage_rows, "small", "baseline", "DullPlanner")
    v1_small = get_row(stage_rows, "small", "v1", "DullPlanner")
    v2_small = get_row(stage_rows, "small", "v2", "DullPlanner")
    v3_small = get_row(stage_rows, "small", "v3", "DullPlanner")
    baseline_medium = get_row(stage_rows, "medium", "baseline", "DullPlanner")
    v1_medium = get_row(stage_rows, "medium", "v1", "DullPlanner")
    v2_medium = get_row(stage_rows, "medium", "v2", "DullPlanner")
    v3_medium = get_row(stage_rows, "medium", "v3", "DullPlanner")
    baseline_high = get_row(stage_rows, "high", "baseline", "DullPlanner")
    v1_high = get_row(stage_rows, "high", "v1", "DullPlanner")
    v2_high = get_row(stage_rows, "high", "v2", "DullPlanner")
    v3_high = get_row(stage_rows, "high", "v3", "DullPlanner")

    task_medium_dull = [row for row in task_rows if row["scenario"] == "medium" and row["local_planner"] == "DullPlanner"]
    showcase_reports = [row for row in showcase_rows if row["artifact"].startswith("report_")]
    best_matrix = planner_rows[:3]

    lines = [
        "# Final Report Summary",
        "",
        "- Generated at: `{}`".format(generated_at),
        "- Source: `experiments/results/run_summaries.csv` and `experiments/results/final_report/**/*.json`",
        "",
        "## Main Findings",
        "",
    ]

    if baseline_small and v1_small:
        lines.append(
            "- `small + DullPlanner`: baseline delivered `{}` packages with `{}` collisions and `{}` replans; `v1` improved to `{}` packages with `{}` collisions, but at `{}` replans.".format(
                int(baseline_small["packages_delivered"]),
                int(baseline_small["total_collision_events"]),
                int(baseline_small["replan_events"]),
                int(v1_small["packages_delivered"]),
                int(v1_small["total_collision_events"]),
                int(v1_small["replan_events"]),
            )
        )
    if v2_small and v3_small:
        lines.append(
            "- `small + DullPlanner`: `v2` and `v3` are not net wins in the current tuning; `v2` dropped to `{}` packages and `v3` to `{}` package while replans stayed very high (`{}` / `{}`).".format(
                int(v2_small["packages_delivered"]),
                int(v3_small["packages_delivered"]),
                int(v2_small["replan_events"]),
                int(v3_small["replan_events"]),
            )
        )
    if baseline_medium and v1_medium and v2_medium:
        lines.append(
            "- `medium + DullPlanner`: baseline had `{}` collisions at `{}` PPH; `v1` removed collisions while keeping `{}` PPH; `v2` further lifted throughput to `{}` PPH with only `{}` collisions.".format(
                int(baseline_medium["total_collision_events"]),
                int(baseline_medium["pph"]),
                int(v1_medium["pph"]),
                int(v2_medium["pph"]),
                int(v2_medium["total_collision_events"]),
            )
        )
    if baseline_high and v1_high:
        lines.append(
            "- `high + DullPlanner`: baseline reached `{}` collisions and `{}` replans; `v1` cut that to `{}` collisions and `{}` replans while throughput rose from `{}` to `{}` PPH.".format(
                int(baseline_high["total_collision_events"]),
                int(baseline_high["replan_events"]),
                int(v1_high["total_collision_events"]),
                int(v1_high["replan_events"]),
                int(baseline_high["pph"]),
                int(v1_high["pph"]),
            )
        )
    if v2_high and v3_high and v1_high:
        lines.append(
            "- `high + DullPlanner`: `v2` regressed badly (`{}` PPH, `{}` collisions), while `v3` recovered to `{}` PPH and lowered collisions to `{}`, outperforming `v1` on safety at the same throughput.".format(
                int(v2_high["pph"]),
                int(v2_high["total_collision_events"]),
                int(v3_high["pph"]),
                int(v3_high["total_collision_events"]),
            )
        )
    if v3_medium and v2_medium:
        lines.append(
            "- `medium + DullPlanner`: `v3` kept collisions at `{}` but throughput fell to `{}` PPH, so the queue-aware rules are helping safety more than throughput in the current tuning.".format(
                int(v3_medium["total_collision_events"]),
                int(v3_medium["pph"]),
            )
        )
    if task_medium_dull:
        naive = next((row for row in task_medium_dull if row["task_manager"] == "NaiveTaskManager"), None)
        congestion = next((row for row in task_medium_dull if row["task_manager"] == "CongestionAwareTaskManager"), None)
        if naive and congestion:
            lines.append(
                "- `medium task_manager + DullPlanner`: `CongestionAwareTaskManager` did not beat the naive baseline in this rerun (`{}` vs `{}` PPH, `{}` vs `{}` collisions).".format(
                    int(congestion["pph"]),
                    int(naive["pph"]),
                    int(congestion["total_collision_events"]),
                    int(naive["total_collision_events"]),
                )
            )
    if showcase_reports:
        showcase_reports = sorted(showcase_reports, key=lambda item: -float(item["pph"] or 0.0))
        best = showcase_reports[0]
        lines.append(
            "- `BaselineTrafficAware` showcase currently peaks at `{}` PPH in `{}` with `{}` collisions; the default 10-agent post run is recorded separately for direct demonstration.".format(
                int(best["pph"]),
                best["artifact"],
                int(best["total_collision_events"]),
            )
        )

    lines.extend(
        [
            "",
            "## Pending Stage-Chain Runs",
            "",
            *build_pending_lines(stage_rows),
            "",
            "## Best Baseline Matrix Entries",
            "",
        ]
    )
    for row in best_matrix:
        lines.append(
            "- `{}` + `{}`: `{}` PPH, `{}` collisions, `{}` replans.".format(
                row["global_planner"],
                row["local_planner"],
                int(row["pph"]),
                int(row["total_collision_events"]),
                int(row["replan_events"]),
            )
        )

    lines.extend(
        [
            "",
            "## Assets",
            "",
            "- `tables/stage_chain_dull.csv` / `.md`",
            "- `tables/stage_chain_virtualforce.csv` / `.md`",
            "- `tables/task_manager_comparison.csv` / `.md`",
            "- `tables/planner_matrix_baseline.csv` / `.md`",
            "- `tables/showcase_runs.csv` / `.md`",
            "- `plots/stage_chain_dull.png`",
            "- `plots/stage_chain_virtualforce.png`",
            "- `plots/task_manager_comparison.png`",
            "- `plots/planner_matrix_baseline.png`",
            "- `plots/showcase_runs.png`",
            "",
        ]
    )
    write_text(FINAL_REPORT_DIR / "summary.md", "\n".join(lines))


def main():
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    rows = load_csv_rows(RESULTS_CSV)
    stage_rows = parse_stage_chain_rows(rows)
    task_rows = parse_task_manager_rows(rows)
    planner_rows = parse_planner_matrix_rows(rows)
    showcase_rows = parse_showcase_rows()

    write_stage_chain_assets(stage_rows)
    write_task_manager_assets(task_rows)
    write_planner_matrix_assets(planner_rows)
    write_showcase_assets(showcase_rows)
    write_summary(stage_rows, task_rows, planner_rows, showcase_rows)

    print("Generated report assets in {}".format(FINAL_REPORT_DIR))


if __name__ == "__main__":
    main()
