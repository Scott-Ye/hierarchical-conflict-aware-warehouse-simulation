# Reproduction Guide

This guide explains how to reproduce the quantitative results stored in this repository.

## 1. Install Dependencies

From the repository root:

```powershell
python -m pip install -r requirements.txt
```

## 2. Verify the Simulator in GUI Mode

```powershell
cd src
python simulator.py --default --agent 6 --port 3 3 --size 24 24 --resolution 1 --step 20 --gp LayeredAStarBaselineTrafficAware --lp DullPlanner --tm NaiveTaskManager -d
```

Expected outcome:

- the GUI opens successfully;
- the warehouse ports are visible;
- agents can receive and execute loading/unloading tasks.

## 3. Reproduce the Main Report Results

Return to the repository root and run:

```powershell
powershell -ExecutionPolicy Bypass -File .\experiments\scripts\reproduce_report_results.ps1
```

This script performs four tasks:

1. reruns the baseline-selection matrix on the original planner family;
2. reruns the `small`, `medium`, and `high` main-comparison experiments for
   `LayeredAStar`, `CollisionAware`, `ReservationAware`, and `QueueAware`;
3. reruns the three `BaselineTrafficAware` experiments used in the final comparison;
4. regenerates summary tables and plots from `experiments/results/run_summaries.csv`.

## 4. Output Files to Check

### Baseline selection

- `experiments/results/planner_matrix_baseline_60s_summary.csv`
- `experiments/results/planner_matrix_baseline_60s_summary.md`

### Main scenario summaries

- `experiments/results/stage_chain_small_60s_summary.csv`
- `experiments/results/stage_chain_medium_60s_summary.csv`
- `experiments/results/stage_chain_high_60s_summary.csv`

### Final derived assets

- `experiments/results/final_report/summary.md`
- `experiments/results/final_report/tables/stage_chain_dull.csv`
- `experiments/results/final_report/tables/planner_matrix_baseline.csv`
- `experiments/results/final_report/plots/mainline_with_bta.png`
- `experiments/results/final_report/plots/planner_matrix_baseline.png`
- `experiments/results/final_report/plots/stage_chain_dull.png`
- `experiments/results/final_report/plots/stage_chain_with_bta.png`

## 5. Raw Result Files Used by the Final Analysis

The per-run JSON logs for the main report tables are stored under:

- `experiments/results/stage_chain_small_60s_*.json`
- `experiments/results/stage_chain_medium_60s_*.json`
- `experiments/results/stage_chain_high_60s_*.json`

These files contain the exact recorded metrics for:

- completed packages;
- PPH;
- collision events;
- replanning events.

## 6. Notes

- The main comparison in the paper follows the single-variable principle:
  the local planner and task manager are fixed while only the global planner changes.
- `BaselineTrafficAware` is evaluated on equal footing with the mainline planners.
- Real execution time depends on the machine. The simulation horizon of the formal experiments is fixed at 60 simulated seconds for each run.
