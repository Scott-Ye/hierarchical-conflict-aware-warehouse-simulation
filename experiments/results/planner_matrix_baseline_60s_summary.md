# Planner Matrix Summary

- Matrix name: `planner_matrix_baseline_60s`
- Generated at: `2026-08-04T14:28:23.380669Z`
- Scenario: `time=1.0 min, agents=2, ports=(1,1) size=12x12, resolution=1, step=10`
- Total combinations: `24`
- Successful runs: `20`
- Failed or timeout runs: `4`

## Successful Runs

| Rank | GP | LP | TM | Packages | PPH | Distance | Global Plans | Replans | Collisions | Agent-Agent | Agent-Wall | Runtime(s) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | LayeredAStar | DullPlanner | NaiveTaskManager | 4 | 240.0 | 147.689 | 123 | 100 | 8 | 8 | 0 | 13.012 |
| 2 | RRTStar | DullPlanner | NaiveTaskManager | 4 | 240.0 | 142.365 | 30 | 0 | 17 | 17 | 0 | 28.894 |
| 3 | RRTStar | VirtualForcePlanner | NaiveTaskManager | 4 | 240.0 | 139.969 | 29 | 0 | 0 | 0 | 0 | 33.105 |
| 4 | RRTStar | RVOPlanner | NaiveTaskManager | 4 | 240.0 | 134.301 | 64 | 0 | 19 | 19 | 0 | 35.099 |
| 5 | MARRTStar | DullPlanner | NaiveTaskManager | 4 | 240.0 | 153.545 | 21 | 0 | 0 | 0 | 0 | 44.032 |
| 6 | MARRTStar | VirtualForcePlanner | NaiveTaskManager | 4 | 240.0 | 148.11 | 21 | 0 | 0 | 0 | 0 | 42.292 |
| 7 | MARRTStar | RVOPlanner | NaiveTaskManager | 4 | 240.0 | 149.411 | 22 | 0 | 0 | 0 | 0 | 39.503 |
| 8 | RRTStar | FLCPlanner | NaiveTaskManager | 3 | 180.0 | 111.063 | 20 | 0 | 0 | 0 | 0 | 27.615 |
| 9 | MARRTStar | FLCPlanner | NaiveTaskManager | 3 | 180.0 | 105.637 | 19 | 0 | 0 | 0 | 0 | 41.145 |
| 10 | LayeredAStar | DDPlanner | NaiveTaskManager | 1 | 60.0 | 41.537 | 47 | 39 | 15 | 15 | 0 | 32.461 |
| 11 | RRTStar | DDPlanner | NaiveTaskManager | 1 | 60.0 | 41.537 | 8 | 0 | 15 | 15 | 0 | 27.161 |
| 12 | MARRTStar | DDPlanner | NaiveTaskManager | 1 | 60.0 | 46.298 | 8 | 0 | 18 | 18 | 0 | 30.888 |
| 13 | INashRRT | DDPlanner | NaiveTaskManager | 1 | 60.0 | 47.623 | 8 | 0 | 17 | 17 | 0 | 36.539 |
| 14 | LayeredAStar | VirtualForcePlanner | NaiveTaskManager | 0 | 0.0 | 9.625 | 571 | 16 | 0 | 0 | 0 | 20.516 |
| 15 | LayeredAStar | RVOPlanner | NaiveTaskManager | 0 | 0.0 | 8.578 | 4 | 0 | 0 | 0 | 0 | 27.904 |
| 16 | LayeredAStar | HRVOPlanner | NaiveTaskManager | 0 | 0.0 | 79.145 | 4 | 0 | 72 | 7 | 65 | 26.055 |
| 17 | LayeredAStar | FLCPlanner | NaiveTaskManager | 0 | 0.0 | 8.067 | 567 | 21 | 0 | 0 | 0 | 36.185 |
| 18 | RRTStar | HRVOPlanner | NaiveTaskManager | 0 | 0.0 | 89.944 | 4 | 0 | 146 | 0 | 146 | 27.092 |
| 19 | MARRTStar | HRVOPlanner | NaiveTaskManager | 0 | 0.0 | 112.305 | 4 | 0 | 30 | 0 | 30 | 27.125 |
| 20 | INashRRT | HRVOPlanner | NaiveTaskManager | 0 | 0.0 | 104.502 | 4 | 0 | 112 | 0 | 112 | 21.026 |

## Failed Or Timeout Runs

| GP | LP | TM | Status | Exit Code | Runtime(s) | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| INashRRT | DullPlanner | NaiveTaskManager | failed | 1 | 31.814 |     temp_sequence_of_poses = self.global_planner.compute_path( <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\multiagent_planner_local_entry.py", line 13, in compute_path <br>     return self.agent.server.request_multiagent_global_planner_compute_path(self.agent) <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\server.py", line 66, in request_multiagent_global_planner_compute_path <br>     solution_paths_dict = ma_planner.compute_path() or {} <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 158, in compute_path <br>     agents_best_edge_path_dict[active_agent_id] = self.best_response(active_agent_id, <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 227, in best_response <br>     current_path_cost, current_diverge_edge_path = frontier.get_pair() <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\sample_global_planner.py", line 29, in get_pair <br>     return heapq.heappop(self.elements) <br> TypeError: '<' not supported between instances of 'iNashStateEdge' and 'iNashStateEdge' |
| INashRRT | VirtualForcePlanner | NaiveTaskManager | failed | 1 | 20.097 |     temp_sequence_of_poses = self.global_planner.compute_path( <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\multiagent_planner_local_entry.py", line 13, in compute_path <br>     return self.agent.server.request_multiagent_global_planner_compute_path(self.agent) <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\server.py", line 66, in request_multiagent_global_planner_compute_path <br>     solution_paths_dict = ma_planner.compute_path() or {} <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 158, in compute_path <br>     agents_best_edge_path_dict[active_agent_id] = self.best_response(active_agent_id, <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 227, in best_response <br>     current_path_cost, current_diverge_edge_path = frontier.get_pair() <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\sample_global_planner.py", line 29, in get_pair <br>     return heapq.heappop(self.elements) <br> TypeError: '<' not supported between instances of 'iNashStateEdge' and 'iNashStateEdge' |
| INashRRT | RVOPlanner | NaiveTaskManager | failed | 1 | 20.192 |     temp_sequence_of_poses = self.global_planner.compute_path( <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\multiagent_planner_local_entry.py", line 13, in compute_path <br>     return self.agent.server.request_multiagent_global_planner_compute_path(self.agent) <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\server.py", line 66, in request_multiagent_global_planner_compute_path <br>     solution_paths_dict = ma_planner.compute_path() or {} <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 158, in compute_path <br>     agents_best_edge_path_dict[active_agent_id] = self.best_response(active_agent_id, <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 227, in best_response <br>     current_path_cost, current_diverge_edge_path = frontier.get_pair() <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\sample_global_planner.py", line 29, in get_pair <br>     return heapq.heappop(self.elements) <br> TypeError: '<' not supported between instances of 'iNashStateEdge' and 'iNashStateEdge' |
| INashRRT | FLCPlanner | NaiveTaskManager | failed | 1 | 10.104 |     temp_sequence_of_poses = self.global_planner.compute_path( <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\multiagent_planner_local_entry.py", line 13, in compute_path <br>     return self.agent.server.request_multiagent_global_planner_compute_path(self.agent) <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\server.py", line 66, in request_multiagent_global_planner_compute_path <br>     solution_paths_dict = ma_planner.compute_path() or {} <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 158, in compute_path <br>     agents_best_edge_path_dict[active_agent_id] = self.best_response(active_agent_id, <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\multiagent_global_planners\inash_planner.py", line 227, in best_response <br>     current_path_cost, current_diverge_edge_path = frontier.get_pair() <br>   File "C:\Users\INT\Desktop\Summer IP\dorabot_minions-master\src\global_planners\sample_global_planner.py", line 29, in get_pair <br>     return heapq.heappop(self.elements) <br> TypeError: '<' not supported between instances of 'iNashStateEdge' and 'iNashStateEdge' |
