# Planner Matrix Summary

- Matrix name: `stage_chain_small_60s`
- Generated at: `2026-08-04T14:21:13.269577Z`
- Scenario: `time=1.0 min, agents=2, ports=(1,1) size=12x12, resolution=1, step=10`
- Total combinations: `8`
- Successful runs: `8`
- Failed or timeout runs: `0`

## Successful Runs

| Rank | GP | LP | TM | Packages | PPH | Distance | Global Plans | Replans | Collisions | Agent-Agent | Agent-Wall | Runtime(s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | LayeredAStarCollisionAware | DullPlanner | NaiveTaskManager | 5 | 300.0 | 144.723 | 229 | 203 | 0 | 0 | 0 | 28.145 |
| 2 | LayeredAStar | DullPlanner | NaiveTaskManager | 4 | 240.0 | 147.689 | 123 | 100 | 8 | 8 | 0 | 13.313 |
| 3 | LayeredAStarQueueAware | DullPlanner | NaiveTaskManager | 1 | 60.0 | 33.555 | 973 | 965 | 1 | 1 | 0 | 40.751 |
| 4 | LayeredAStar | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 9.625 | 571 | 16 | 0 | 0 | 0 | 20.541 |
| 5 | LayeredAStarCollisionAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 6.967 | 1139 | 1135 | 0 | 0 | 0 | 37.129 |
| 6 | LayeredAStarReservationAware | DullPlanner | NaiveTaskManager | 0 | 0.0 | 9.81 | 1130 | 1124 | 0 | 0 | 0 | 50.961 |
| 7 | LayeredAStarReservationAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 6.517 | 1160 | 1157 | 0 | 0 | 0 | 35.426 |
| 8 | LayeredAStarQueueAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 6.967 | 1137 | 1133 | 0 | 0 | 0 | 42.435 |

## Failed Or Timeout Runs

| GP | LP | TM | Status | Exit Code | Runtime(s) | Notes |
| --- | --- | --- | --- | --- | --- | --- |
