# Final Report Summary

- Generated at: `2026-08-04 21:38:21Z`
- Source: `experiments/results/run_summaries.csv`

## Main Findings

- `small`: `CollisionAware` improves on the baseline from `4` to `5` packages while removing collisions (`8` to `0`).
- `small`: `ReservationAware` and `QueueAware` are too conservative in the current tuning (`0` / `60` PPH).
- `medium`: `CollisionAware` removes the baseline collisions while keeping `360` PPH, and `ReservationAware` further increases throughput to `420` PPH with only `6` collisions.
- `high`: `QueueAware` reaches `420` PPH with `18` collisions, improving on both the baseline (`949` collisions) and `ReservationAware` (`180` PPH).
- `BaselineTrafficAware` stays collision-free across all three scenarios, with throughput `60`, `360`, and `360` PPH for `small`, `medium`, and `high`, respectively.
- In the `medium` scenario, `BaselineTrafficAware` matches the collision-free behavior of `CollisionAware` at `360` PPH, but remains below the peak throughput of `ReservationAware` (`420` PPH).
- In the `high` scenario, `BaselineTrafficAware` provides the safest behavior (`0` collisions) at a lower throughput than `CollisionAware` and `QueueAware` (`360` versus `420` / `420` PPH).

## Pending Runs

- All planned mainline comparison runs are present.

## Best Baseline Matrix Entries

- `MARRTStar` + `DullPlanner`: `240` PPH, `0` collisions, `0` replans.
- `MARRTStar` + `RVOPlanner`: `240` PPH, `0` collisions, `0` replans.
- `MARRTStar` + `VirtualForcePlanner`: `240` PPH, `0` collisions, `0` replans.

## Assets

- `tables/stage_chain_dull.csv` / `.md`
- `tables/stage_chain_with_bta.csv` / `.md`
- `tables/planner_matrix_baseline.csv` / `.md`
- `plots/stage_chain_dull.png`
- `plots/stage_chain_with_bta.png`
- `plots/mainline_with_bta.png`
- `plots/planner_matrix_baseline.png`
