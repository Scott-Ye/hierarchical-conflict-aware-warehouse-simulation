# Final Report Summary

- Generated at: `2026-08-04 14:38:40Z`
- Source: `experiments/results/run_summaries.csv` and `experiments/results/final_report/**/*.json`

## Main Findings

- `small + DullPlanner`: baseline delivered `4` packages with `8` collisions and `100` replans; `v1` improved to `5` packages with `0` collisions, but at `203` replans.
- `small + DullPlanner`: `v2` and `v3` are not net wins in the current tuning; `v2` dropped to `0` packages and `v3` to `1` package while replans stayed very high (`1124` / `965`).
- `medium + DullPlanner`: baseline had `235` collisions at `360` PPH; `v1` removed collisions while keeping `360` PPH; `v2` further lifted throughput to `420` PPH with only `6` collisions.
- `high + DullPlanner`: baseline reached `949` collisions and `2082` replans; `v1` cut that to `27` collisions and `470` replans while throughput rose from `300` to `420` PPH.
- `high + DullPlanner`: `v2` regressed badly (`180` PPH, `1068` collisions), while `v3` recovered to `420` PPH and lowered collisions to `18`, outperforming `v1` on safety at the same throughput.
- `medium + DullPlanner`: `v3` kept collisions at `0` but throughput fell to `300` PPH, so the queue-aware rules are helping safety more than throughput in the current tuning.
- `medium task_manager + DullPlanner`: `CongestionAwareTaskManager` did not beat the naive baseline in this rerun (`300` vs `360` PPH, `7` vs `0` collisions).
- `BaselineTrafficAware` showcase currently peaks at `600` PPH in `report_default_10a_post` with `0` collisions; the default 10-agent post run is recorded separately for direct demonstration.

## Pending Stage-Chain Runs

- stage_chain 主线矩阵已全部落盘。

## Best Baseline Matrix Entries

- `MARRTStar` + `DullPlanner`: `240` PPH, `0` collisions, `0` replans.
- `MARRTStar` + `RVOPlanner`: `240` PPH, `0` collisions, `0` replans.
- `MARRTStar` + `VirtualForcePlanner`: `240` PPH, `0` collisions, `0` replans.

## Assets

- `tables/stage_chain_dull.csv` / `.md`
- `tables/stage_chain_virtualforce.csv` / `.md`
- `tables/task_manager_comparison.csv` / `.md`
- `tables/planner_matrix_baseline.csv` / `.md`
- `tables/showcase_runs.csv` / `.md`
- `plots/stage_chain_dull.png`
- `plots/stage_chain_virtualforce.png`
- `plots/task_manager_comparison.png`
- `plots/planner_matrix_baseline.png`
- `plots/showcase_runs.png`
