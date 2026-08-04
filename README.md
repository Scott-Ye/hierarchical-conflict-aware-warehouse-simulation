# Hierarchical Conflict-Aware Planning for Multi-Agent Warehouse Simulation

This repository contains the simulator, planner implementations, experiment scripts, and recorded results for the warehouse planning study.

It is organized so that a new reader can:

1. understand the simulator and the five global planners studied in the report;
2. run the simulator in GUI mode;
3. reproduce the quantitative results from the final experiments;
4. regenerate the summary tables and plots from the recorded experiment logs.

## Included Content

- `src/`: Python 3 warehouse simulator and all planner implementations
- `experiments/scripts/`: batch experiment and result-asset generation scripts
- `experiments/results/`: recorded results used by the final analysis
- `docs/REPRODUCTION_GUIDE.md`: step-by-step reproduction guide

## Tested Environment

- Windows + PowerShell
- Python 3.10

## Python Dependencies

Install the required packages with:

```powershell
python -m pip install -r requirements.txt
```

## Quick Start

Run the maintained traffic-aware planner in GUI mode:

```powershell
cd src
python simulator.py --default --agent 6 --port 3 3 --size 24 24 --resolution 1 --step 20 --gp LayeredAStarBaselineTrafficAware --lp DullPlanner --tm NaiveTaskManager -d
```

This is the fastest way to verify that the simulator, planner registration, and GUI rendering all work correctly.

## Reproducing the Report Results

The full instructions are provided in [`docs/REPRODUCTION_GUIDE.md`](./docs/REPRODUCTION_GUIDE.md).

If you want to rerun the main report results directly, use:

```powershell
powershell -ExecutionPolicy Bypass -File .\experiments\scripts\reproduce_report_results.ps1
```

This reruns:

- the baseline-selection matrix used to justify `LayeredAStar` as the baseline;
- the three main scenario groups (`small`, `medium`, `high`) for
  `Baseline`, `CollisionAware`, `ReservationAware`, `QueueAware`;
- the three `BaselineTrafficAware` runs used in the final comparison;
- the final summary-table and plot generation step.

## Where the Main Results Are Stored

- Raw combined logs:
  - `experiments/results/run_summaries.csv`
  - `experiments/results/run_summaries.jsonl`
- Final derived assets:
  - `experiments/results/final_report/summary.md`
  - `experiments/results/final_report/tables/`
  - `experiments/results/final_report/plots/`

## Repository Layout

```text
dorabot_minions_github_ready/
├── README.md
├── requirements.txt
├── docs/
│   └── REPRODUCTION_GUIDE.md
├── experiments/
│   ├── scripts/
│   └── results/
└── src/
```

## Reading Order

1. `README.md`
2. `docs/REPRODUCTION_GUIDE.md`
3. `experiments/results/final_report/summary.md`
4. `experiments/results/final_report/tables/`
5. `src/global_planners/user/`
