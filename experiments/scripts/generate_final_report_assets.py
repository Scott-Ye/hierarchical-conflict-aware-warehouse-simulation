import csv
import re
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_CSV = REPO_ROOT / "experiments" / "results" / "run_summaries.csv"
FINAL_REPORT_DIR = REPO_ROOT / "experiments" / "results" / "final_report"
TABLES_DIR = FINAL_REPORT_DIR / "tables"
PLOTS_DIR = FINAL_REPORT_DIR / "plots"

SCENARIO_ORDER = ["small", "medium", "high"]
MAINLINE_STAGE_ORDER = [
    ("LayeredAStar", "Baseline"),
    ("LayeredAStarCollisionAware", "CollisionAware"),
    ("LayeredAStarReservationAware", "ReservationAware"),
    ("LayeredAStarQueueAware", "QueueAware"),
]
BTA_STAGE = ("LayeredAStarBaselineTrafficAware", "BaselineTrafficAware")


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
            rows.append({key: to_number(value) for key, value in row.items()})
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


def build_markdown_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join(["---"] * len(headers)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(header, "")) for header in headers) + " |")
    return "\n".join(lines)


def collect_mainline_rows(rows):
    pattern = re.compile(r"^stage_chain_(small|medium|high)_60s_")
    stage_lookup = dict(MAINLINE_STAGE_ORDER)
    parsed = []
    by_key = {}

    for row in rows:
        record_tag = str(row.get("record_tag", ""))
        match = pattern.match(record_tag)
        if not match:
            continue
        planner_name = row.get("global_planner")
        if planner_name not in stage_lookup:
            continue
        if row.get("local_planner") != "DullPlanner":
            continue

        normalized = {
            "scenario": match.group(1),
            "stage": stage_lookup[planner_name],
            "global_planner": planner_name,
            "local_planner": row.get("local_planner"),
            "task_manager": row.get("task_manager"),
            "packages_delivered": row.get("packages_delivered"),
            "pph": row.get("pph"),
            "total_collision_events": row.get("total_collision_events"),
            "agent_agent_collision_events": row.get("agent_agent_collision_events"),
            "replan_events": row.get("replan_events"),
            "real_runtime_seconds": row.get("real_runtime_seconds"),
            "record_tag": record_tag,
            "status": "available",
        }
        parsed.append(normalized)
        by_key[(normalized["scenario"], normalized["stage"])] = normalized

    expected = []
    for scenario in SCENARIO_ORDER:
        for _, stage in MAINLINE_STAGE_ORDER:
            row = by_key.get((scenario, stage))
            if row is None:
                expected.append(
                    {
                        "scenario": scenario,
                        "stage": stage,
                        "global_planner": "",
                        "local_planner": "DullPlanner",
                        "task_manager": "NaiveTaskManager",
                        "packages_delivered": "",
                        "pph": "",
                        "total_collision_events": "",
                        "agent_agent_collision_events": "",
                        "replan_events": "",
                        "real_runtime_seconds": "",
                        "record_tag": "",
                        "status": "pending",
                    }
                )
            else:
                expected.append(row)
    return expected


def collect_bta_rows(rows):
    pattern = re.compile(r"^stage_chain_bta_(small|medium|high)_60s_")
    parsed = []
    by_scenario = {}

    for row in rows:
        record_tag = str(row.get("record_tag", ""))
        match = pattern.match(record_tag)
        if not match:
            continue
        if row.get("global_planner") != BTA_STAGE[0]:
            continue
        normalized = {
            "scenario": match.group(1),
            "stage": BTA_STAGE[1],
            "global_planner": row.get("global_planner"),
            "local_planner": row.get("local_planner"),
            "task_manager": row.get("task_manager"),
            "packages_delivered": row.get("packages_delivered"),
            "pph": row.get("pph"),
            "total_collision_events": row.get("total_collision_events"),
            "agent_agent_collision_events": row.get("agent_agent_collision_events"),
            "replan_events": row.get("replan_events"),
            "real_runtime_seconds": row.get("real_runtime_seconds"),
            "record_tag": record_tag,
            "status": "available",
        }
        parsed.append(normalized)
        by_scenario[normalized["scenario"]] = normalized

    expected = []
    for scenario in SCENARIO_ORDER:
        row = by_scenario.get(scenario)
        if row is None:
            expected.append(
                {
                    "scenario": scenario,
                    "stage": BTA_STAGE[1],
                    "global_planner": BTA_STAGE[0],
                    "local_planner": "DullPlanner",
                    "task_manager": "NaiveTaskManager",
                    "packages_delivered": "",
                    "pph": "",
                    "total_collision_events": "",
                    "agent_agent_collision_events": "",
                    "replan_events": "",
                    "real_runtime_seconds": "",
                    "record_tag": "",
                    "status": "pending",
                }
            )
        else:
            expected.append(row)
    return expected


def collect_planner_matrix_rows(rows):
    parsed = []
    for row in rows:
        record_tag = str(row.get("record_tag", ""))
        if not record_tag.startswith("planner_matrix_baseline_60s_"):
            continue
        if row.get("task_manager") != "NaiveTaskManager":
            continue
        parsed.append(
            {
                "global_planner": row.get("global_planner"),
                "local_planner": row.get("local_planner"),
                "task_manager": row.get("task_manager"),
                "packages_delivered": row.get("packages_delivered"),
                "pph": row.get("pph"),
                "total_collision_events": row.get("total_collision_events"),
                "agent_agent_collision_events": row.get("agent_agent_collision_events"),
                "replan_events": row.get("replan_events"),
                "real_runtime_seconds": row.get("real_runtime_seconds"),
                "record_tag": record_tag,
            }
        )
    return sorted(parsed, key=lambda item: (-float(item["pph"] or 0.0), float(item["total_collision_events"] or 0.0), item["record_tag"]))


def get_row(rows, scenario, stage):
    return next((row for row in rows if row["scenario"] == scenario and row["stage"] == stage and row["status"] == "available"), None)


def write_stage_chain_assets(mainline_rows):
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
    write_csv(TABLES_DIR / "stage_chain_dull.csv", mainline_rows, fieldnames)

    markdown_rows = [
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
        for row in mainline_rows
    ]
    write_text(
        TABLES_DIR / "stage_chain_dull.md",
        "\n".join(
            [
                "# Mainline Stage Chain (DullPlanner)",
                "",
                build_markdown_table(
                    ["Scenario", "Stage", "Planner", "Packages", "PPH", "Collisions", "Replans", "Runtime(s)", "Status"],
                    markdown_rows,
                ),
                "",
            ]
        ),
    )
    plot_stage_metrics(mainline_rows, PLOTS_DIR / "stage_chain_dull.png")


def write_bta_assets(mainline_rows, bta_rows):
    combined_rows = []
    for scenario in SCENARIO_ORDER:
        combined_rows.extend([row for row in mainline_rows if row["scenario"] == scenario])
        combined_rows.extend([row for row in bta_rows if row["scenario"] == scenario])

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
    write_csv(TABLES_DIR / "stage_chain_with_bta.csv", combined_rows, fieldnames)

    markdown_rows = [
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
        for row in combined_rows
    ]
    write_text(
        TABLES_DIR / "stage_chain_with_bta.md",
        "\n".join(
            [
                "# Mainline Stage Chain With BaselineTrafficAware",
                "",
                build_markdown_table(
                    ["Scenario", "Stage", "Planner", "Packages", "PPH", "Collisions", "Replans", "Runtime(s)", "Status"],
                    markdown_rows,
                ),
                "",
            ]
        ),
    )
    plot_stage_metrics(combined_rows, PLOTS_DIR / "stage_chain_with_bta.png")
    plot_mainline_with_bta(combined_rows, PLOTS_DIR / "mainline_with_bta.png")


def plot_stage_metrics(rows, output_path):
    available = [row for row in rows if row["status"] == "available"]
    if not available:
        return

    stage_labels = []
    for row in rows:
        if row["stage"] not in stage_labels:
            stage_labels.append(row["stage"])
    width = max(0.12, 0.8 / max(len(stage_labels), 1))
    x_positions = list(range(len(SCENARIO_ORDER)))

    fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.8))
    center_shift = (len(stage_labels) - 1) / 2.0

    for stage_index, stage in enumerate(stage_labels):
        pph_values = []
        collision_values = []
        for scenario in SCENARIO_ORDER:
            row = next((item for item in rows if item["scenario"] == scenario and item["stage"] == stage), None)
            pph_values.append(float(row["pph"]) if row and row["status"] == "available" and row["pph"] != "" else 0.0)
            collision_values.append(
                float(row["total_collision_events"])
                if row and row["status"] == "available" and row["total_collision_events"] != ""
                else 0.0
            )
        offsets = [value + (stage_index - center_shift) * width for value in x_positions]
        axes[0].bar(offsets, pph_values, width=width, label=stage)
        axes[1].bar(offsets, collision_values, width=width, label=stage)

    for axis, title, ylabel in [
        (axes[0], "Throughput comparison", "PPH"),
        (axes[1], "Collision comparison", "Collision events"),
    ]:
        axis.set_xticks(x_positions)
        axis.set_xticklabels(SCENARIO_ORDER)
        axis.set_title(title)
        axis.set_ylabel(ylabel)
        axis.grid(axis="y", alpha=0.25)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=max(1, len(stage_labels)))
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_mainline_with_bta(rows, output_path):
    available = [row for row in rows if row["status"] == "available"]
    if not available:
        return

    ordered_rows = []
    stage_order = [stage for _, stage in MAINLINE_STAGE_ORDER] + [BTA_STAGE[1]]
    for scenario in SCENARIO_ORDER:
        for stage in stage_order:
            row = next((item for item in rows if item["scenario"] == scenario and item["stage"] == stage and item["status"] == "available"), None)
            if row is not None:
                ordered_rows.append(row)

    labels = ["{}\n{}".format(row["scenario"], row["stage"]) for row in ordered_rows]
    pph_values = [float(row["pph"] or 0.0) for row in ordered_rows]
    collision_values = [float(row["total_collision_events"] or 0.0) for row in ordered_rows]

    fig, ax = plt.subplots(figsize=(10.5, max(5.0, len(ordered_rows) * 0.45)))
    bars = ax.barh(range(len(ordered_rows)), pph_values, color="#4e79a7")
    ax.set_yticks(range(len(ordered_rows)))
    ax.set_yticklabels(labels)
    ax.set_xlabel("PPH")
    ax.set_title("Mainline planners and BaselineTrafficAware")
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
    write_text(
        TABLES_DIR / "planner_matrix_baseline.md",
        "\n".join(
            [
                "# Baseline Planner Matrix",
                "",
                build_markdown_table(
                    ["Global", "Local", "Packages", "PPH", "Collisions", "AA Collisions", "Replans", "Runtime(s)"],
                    markdown_rows,
                ),
                "",
            ]
        ),
    )
    plot_planner_matrix(rows, PLOTS_DIR / "planner_matrix_baseline.png")


def plot_planner_matrix(rows, output_path):
    if not rows:
        return

    labels = ["{}\n{}".format(row["global_planner"], row["local_planner"].replace("Planner", "")) for row in rows]
    pph_values = [float(row["pph"] or 0.0) for row in rows]
    collision_values = [float(row["total_collision_events"] or 0.0) for row in rows]

    fig, ax = plt.subplots(figsize=(10, max(4.5, len(rows) * 0.42)))
    bars = ax.barh(range(len(rows)), pph_values, color="#59a14f")
    ax.set_yticks(range(len(rows)))
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


def build_pending_lines(mainline_rows, bta_rows):
    pending = [row for row in mainline_rows + bta_rows if row["status"] == "pending"]
    if not pending:
        return ["- All planned mainline comparison runs are present."]
    return [
        "- Missing: {} {}".format(row["scenario"], row["stage"])
        for row in pending
    ]


def write_summary(mainline_rows, bta_rows, planner_rows):
    generated_at = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%SZ")

    baseline_small = get_row(mainline_rows, "small", "Baseline")
    v1_small = get_row(mainline_rows, "small", "CollisionAware")
    v2_small = get_row(mainline_rows, "small", "ReservationAware")
    v3_small = get_row(mainline_rows, "small", "QueueAware")
    baseline_medium = get_row(mainline_rows, "medium", "Baseline")
    v1_medium = get_row(mainline_rows, "medium", "CollisionAware")
    v2_medium = get_row(mainline_rows, "medium", "ReservationAware")
    v3_medium = get_row(mainline_rows, "medium", "QueueAware")
    baseline_high = get_row(mainline_rows, "high", "Baseline")
    v1_high = get_row(mainline_rows, "high", "CollisionAware")
    v2_high = get_row(mainline_rows, "high", "ReservationAware")
    v3_high = get_row(mainline_rows, "high", "QueueAware")
    bta_small = get_row(bta_rows, "small", BTA_STAGE[1])
    bta_medium = get_row(bta_rows, "medium", BTA_STAGE[1])
    bta_high = get_row(bta_rows, "high", BTA_STAGE[1])

    lines = [
        "# Final Report Summary",
        "",
        "- Generated at: `{}`".format(generated_at),
        "- Source: `experiments/results/run_summaries.csv`",
        "",
        "## Main Findings",
        "",
    ]

    if baseline_small and v1_small:
        lines.append(
            "- `small`: `CollisionAware` improves on the baseline from `{}` to `{}` packages while removing collisions (`{}` to `{}`).".format(
                int(baseline_small["packages_delivered"]),
                int(v1_small["packages_delivered"]),
                int(baseline_small["total_collision_events"]),
                int(v1_small["total_collision_events"]),
            )
        )
    if v2_small and v3_small:
        lines.append(
            "- `small`: `ReservationAware` and `QueueAware` are too conservative in the current tuning (`{}` / `{}` PPH).".format(
                int(v2_small["pph"]),
                int(v3_small["pph"]),
            )
        )
    if baseline_medium and v1_medium and v2_medium:
        lines.append(
            "- `medium`: `CollisionAware` removes the baseline collisions while keeping `{}` PPH, and `ReservationAware` further increases throughput to `{}` PPH with only `{}` collisions.".format(
                int(v1_medium["pph"]),
                int(v2_medium["pph"]),
                int(v2_medium["total_collision_events"]),
            )
        )
    if baseline_high and v1_high and v3_high and v2_high:
        lines.append(
            "- `high`: `QueueAware` reaches `{}` PPH with `{}` collisions, improving on both the baseline (`{}` collisions) and `ReservationAware` (`{}` PPH).".format(
                int(v3_high["pph"]),
                int(v3_high["total_collision_events"]),
                int(baseline_high["total_collision_events"]),
                int(v2_high["pph"]),
            )
        )
    if bta_small and bta_medium and bta_high:
        lines.append(
            "- `BaselineTrafficAware` stays collision-free across all three scenarios, with throughput `{}`, `{}`, and `{}` PPH for `small`, `medium`, and `high`, respectively.".format(
                int(bta_small["pph"]),
                int(bta_medium["pph"]),
                int(bta_high["pph"]),
            )
        )
    if bta_medium and v2_medium and v1_medium:
        lines.append(
            "- In the `medium` scenario, `BaselineTrafficAware` matches the collision-free behavior of `CollisionAware` at `{}` PPH, but remains below the peak throughput of `ReservationAware` (`{}` PPH).".format(
                int(bta_medium["pph"]),
                int(v2_medium["pph"]),
            )
        )
    if bta_high and v1_high and v3_high:
        lines.append(
            "- In the `high` scenario, `BaselineTrafficAware` provides the safest behavior (`{}` collisions) at a lower throughput than `CollisionAware` and `QueueAware` (`{}` versus `{}` / `{}` PPH).".format(
                int(bta_high["total_collision_events"]),
                int(bta_high["pph"]),
                int(v1_high["pph"]),
                int(v3_high["pph"]),
            )
        )

    lines.extend(
        [
            "",
            "## Pending Runs",
            "",
            *build_pending_lines(mainline_rows, bta_rows),
            "",
            "## Best Baseline Matrix Entries",
            "",
        ]
    )
    for row in planner_rows[:3]:
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
            "- `tables/stage_chain_with_bta.csv` / `.md`",
            "- `tables/planner_matrix_baseline.csv` / `.md`",
            "- `plots/stage_chain_dull.png`",
            "- `plots/stage_chain_with_bta.png`",
            "- `plots/mainline_with_bta.png`",
            "- `plots/planner_matrix_baseline.png`",
            "",
        ]
    )
    write_text(FINAL_REPORT_DIR / "summary.md", "\n".join(lines))


def main():
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    PLOTS_DIR.mkdir(parents=True, exist_ok=True)

    rows = load_csv_rows(RESULTS_CSV)
    mainline_rows = collect_mainline_rows(rows)
    bta_rows = collect_bta_rows(rows)
    planner_rows = collect_planner_matrix_rows(rows)

    write_stage_chain_assets(mainline_rows)
    write_bta_assets(mainline_rows, bta_rows)
    write_planner_matrix_assets(planner_rows)
    write_summary(mainline_rows, bta_rows, planner_rows)

    print("Generated report assets in {}".format(FINAL_REPORT_DIR))


if __name__ == "__main__":
    main()
