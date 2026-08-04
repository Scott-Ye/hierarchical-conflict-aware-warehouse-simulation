# Planner Matrix Summary

- Matrix name: `stage_chain_medium_60s`
- Generated at: `2026-08-04T14:28:09.233533Z`
- Scenario: `time=1.0 min, agents=4, ports=(2,2) size=20x20, resolution=1, step=20`
- Total combinations: `8`
- Successful runs: `8`
- Failed or timeout runs: `0`

## Successful Runs

| Rank | GP | LP | TM | Packages | PPH | Distance | Global Plans | Replans | Collisions | Agent-Agent | Agent-Wall | Runtime(s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | LayeredAStarReservationAware | DullPlanner | NaiveTaskManager | 7 | 420.0 | 314.811 | 277 | 242 | 6 | 6 | 0 | 89.527 |
| 2 | LayeredAStar | DullPlanner | NaiveTaskManager | 6 | 360.0 | 282.977 | 649 | 616 | 235 | 235 | 0 | 57.486 |
| 3 | LayeredAStarCollisionAware | DullPlanner | NaiveTaskManager | 6 | 360.0 | 314.084 | 440 | 404 | 0 | 0 | 0 | 95.517 |
| 4 | LayeredAStarQueueAware | DullPlanner | NaiveTaskManager | 5 | 300.0 | 289.682 | 753 | 721 | 0 | 0 | 0 | 75.874 |
| 5 | LayeredAStar | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 41.074 | 2031 | 1097 | 1 | 1 | 0 | 93.274 |
| 6 | LayeredAStarCollisionAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 39.184 | 3141 | 3133 | 0 | 0 | 0 | 93.67 |
| 7 | LayeredAStarReservationAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 39.184 | 3141 | 3133 | 0 | 0 | 0 | 94.953 |
| 8 | LayeredAStarQueueAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 39.184 | 3142 | 3134 | 0 | 0 | 0 | 82.78 |

## Failed Or Timeout Runs

| GP | LP | TM | Status | Exit Code | Runtime(s) | Notes |
| --- | --- | --- | --- | --- | --- | --- |
