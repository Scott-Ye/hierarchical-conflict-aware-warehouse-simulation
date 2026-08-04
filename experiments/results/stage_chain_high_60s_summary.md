# Planner Matrix Summary

- Matrix name: `stage_chain_high_60s`
- Generated at: `2026-08-04T14:33:09.132754Z`
- Scenario: `time=1.0 min, agents=6, ports=(3,3) size=24x24, resolution=1, step=20`
- Total combinations: `8`
- Successful runs: `8`
- Failed or timeout runs: `0`

## Successful Runs

| Rank | GP | LP | TM | Packages | PPH | Distance | Global Plans | Replans | Collisions | Agent-Agent | Agent-Wall | Runtime(s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | LayeredAStarCollisionAware | DullPlanner | NaiveTaskManager | 7 | 420.0 | 480.029 | 513 | 470 | 27 | 27 | 0 | 128.429 |
| 2 | LayeredAStarQueueAware | DullPlanner | NaiveTaskManager | 7 | 420.0 | 478.877 | 569 | 523 | 18 | 18 | 0 | 79.151 |
| 3 | LayeredAStar | DullPlanner | NaiveTaskManager | 5 | 300.0 | 360.995 | 2117 | 2082 | 949 | 949 | 0 | 109.565 |
| 4 | LayeredAStarReservationAware | DullPlanner | NaiveTaskManager | 3 | 180.0 | 233.569 | 3959 | 3932 | 1068 | 1068 | 0 | 165.641 |
| 5 | LayeredAStar | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 101.197 | 2106 | 136 | 42 | 42 | 0 | 178.35 |
| 6 | LayeredAStarCollisionAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 147.227 | 2077 | 2061 | 0 | 0 | 0 | 141.861 |
| 7 | LayeredAStarReservationAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 130.496 | 2138 | 2122 | 0 | 0 | 0 | 88.022 |
| 8 | LayeredAStarQueueAware | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 178.151 | 2886 | 2561 | 0 | 0 | 0 | 90.662 |

## Failed Or Timeout Runs

| GP | LP | TM | Status | Exit Code | Runtime(s) | Notes |
| --- | --- | --- | --- | --- | --- | --- |
