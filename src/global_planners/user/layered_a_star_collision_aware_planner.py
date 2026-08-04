from math import pi, sqrt
import json
import urllib.request

from geometry import Point, Vector, compute_direction, edge_edge_shortest_square_distance
from global_planners.layered_astar_planner import LayeredAStar
from representation.float_to_grid import agent_to_gridmap
from task_managers.task_manager import TaskType


class LayeredAStarCollisionAware(LayeredAStar):
    """First-stage improvement over baseline LayeredAStar."""
    _DEBUG_ENDPOINT_DISABLED = object()
    _debug_endpoint_cache = {}

    SENSOR_RANGE = 6.0
    SENSOR_ANGLE = pi
    BASE_INFLATION = 14.0
    REPLAN_DISTANCE = 1.5
    TRACKING_DISTANCE = REPLAN_DISTANCE
    LOOKAHEAD_POINTS = 3
    YIELD_DISTANCE = 1.2
    YIELD_SEGMENT_DISTANCE = 0.85
    CORRIDOR_DISTANCE = 1.5
    STOP_DISTANCE = 1.3
    MOVING_BLOCKER_STOP_DISTANCE = 1.65
    STOPPING_BUFFER_RADIUS = 1.35
    STOPPING_BLOCKER_NODE_INFLATION = 42.0
    STOPPING_BLOCKER_EDGE_INFLATION = 24.0
    FORCED_BYPASS_NODE_INFLATION = 68.0
    FORCED_BYPASS_EDGE_INFLATION = 40.0

    def __init__(self, agent):
        super(LayeredAStarCollisionAware, self).__init__(agent)
        # Compatibility state for later stages that still inherit from v1.
        self.active_yield_blockers = {}
        self._forced_bypass_blocker_id = None

    def get_dynamic_layer(self, gridmap, sensor_observation, inflation=None):
        dynamic_layer = {}
        observed_agents = sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        )
        for agent in observed_agents:
            observed_state = getattr(agent, "userData", agent)
            origin = agent_to_gridmap(agent.position, gridmap)
            # v1 的第一层改进不是改 A* 主体，而是把“别人现在占着的位置”
            # 提前变成高代价区域。这样 baseline 那种完全把别的 agent 当空气、
            # 直到快撞上才被动停住的情况，会先被代价层推着绕开。
            self._inflate_transition_ring(dynamic_layer, gridmap, origin, self.BASE_INFLATION)
            first_ring = list(gridmap.neighbors(origin))
            for first in first_ring:
                self._inflate_transition_ring(dynamic_layer, gridmap, first, self.BASE_INFLATION * 0.75)
                for second in gridmap.neighbors(first):
                    if second == origin:
                        continue
                    self._inflate_transition_ring(
                        dynamic_layer, gridmap, second, self.BASE_INFLATION * 0.35
                    )

            if getattr(observed_state, "id", None) == self._forced_bypass_blocker_id:
                # 当前 replan 已经确定“我该通过、对方该绕/停”，
                # 这里把 blocker 临时钉成一圈局部障碍，强制这次路径改走旁边。
                self._apply_blocker_barrier(
                    dynamic_layer,
                    gridmap,
                    observed_state.position,
                    node_inflation=self.FORCED_BYPASS_NODE_INFLATION,
                    edge_inflation=self.FORCED_BYPASS_EDGE_INFLATION,
                )
                continue

            if self._is_absolute_stopping_blocker(
                self.agent.position,
                getattr(self.agent, "sequence_of_poses", []),
                observed_state,
            ):
                # 如果对方已经是 STOPPING，而且正好停在我未来几步路径上，
                # v1 会把它当成“绝对不能穿过的静态障碍”。
                # 这是为了兜住“replan 失败后 stop 的车又被后车追上”这类近场碰撞。
                self._apply_blocker_barrier(
                    dynamic_layer,
                    gridmap,
                    observed_state.position,
                    node_inflation=self.STOPPING_BLOCKER_NODE_INFLATION,
                    edge_inflation=self.STOPPING_BLOCKER_EDGE_INFLATION,
                )
            elif self._should_force_bypass_stopping_peer(
                self.agent.position,
                getattr(self.agent, "sequence_of_poses", []),
                observed_state,
            ):
                # 对方已经明确在给当前 agent 让路时，也要把它回写成局部绕行障碍；
                # 否则 replan 很容易沿原走廊继续顶进已停住的对手。
                self._apply_blocker_barrier(
                    dynamic_layer,
                    gridmap,
                    observed_state.position,
                    node_inflation=self.STOPPING_BLOCKER_NODE_INFLATION,
                    edge_inflation=self.STOPPING_BLOCKER_EDGE_INFLATION,
                )
        return dynamic_layer

    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold=1):
        if not sequence_of_poses:
            return False

        visible_waypoints = []
        remaining_distance = current_position.distance(sequence_of_poses[-1])
        detected_obstacles = sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        )

        # baseline 的 observe_path 基本只看静态图是否需要重算。
        # v1 在这里加的，是“近场交通变复杂时先重新看一眼”：
        # 只要局部已经很挤，就先触发一次全局重规划，而不是等真正贴脸才处理。
        if len(detected_obstacles) > 4:
            return True

        for pose in sequence_of_poses:
            if (
                current_position.distance(pose) < self.SENSOR_RANGE
                and sequence_of_poses[-1].distance(pose) < remaining_distance
            ):
                visible_waypoints.append(pose)

        for agent in detected_obstacles:
            observed_state = getattr(agent, "userData", agent)
            position = observed_state.position
            if (
                current_position.distance(position) < self.STOP_DISTANCE
                and self._blocker_on_current_corridor(current_position, sequence_of_poses, position)
            ):
                # “已经贴到当前走廊里的人”直接算高危，先重规划。
                # 这一步对应你定义的 v1 思路里的“先看”。
                return True
            for waypoint in visible_waypoints:
                if waypoint.distance(position) < self.REPLAN_DISTANCE:
                    # 如果前方可见路径点会穿到别人的当前位置附近，也视为即将冲突。
                    return True
        return False

    def compute_avoidance_response(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return None

        conflict_candidates = []
        debug_candidates = []
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None or blocker_id == getattr(self.agent, "id", None):
                continue
            has_conflict = self._conflicts_with_blocker_state(position, sequence_of_poses, observed_state, observed_agent)
            if self._should_debug_midmap_pair_window():
                debug_candidates.append(
                    {
                        "blocker_id": blocker_id,
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "distance": round(position.distance(observed_state.position), 3),
                        "stopping_for_agent_id": getattr(observed_state, "stopping_for_agent_id", None),
                        "conflicts": has_conflict,
                    }
                )
            if has_conflict:
                conflict_candidates.append((position.distance(observed_state.position), blocker_id))

        if self._should_debug_midmap_pair_window():
            self._debug_midmap_pair_event(
                "G",
                "layered_a_star_collision_aware_planner.py:compute_avoidance_response",
                "[DEBUG] focused mid-map pair evaluated collision-aware avoidance candidates",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "step": self._current_simulator_step(),
                    "agent_state": own_state_name,
                    "position": [round(position.x, 3), round(position.y, 3)] if position is not None else None,
                    "path_preview": self._point_preview(sequence_of_poses),
                    "candidates": debug_candidates,
                },
            )

        if not conflict_candidates:
            return None
        blocker_distance, blocker_id = min(conflict_candidates, key=lambda item: item[0])
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        blocker_base_state = self._base_state_name(blocker_state) if blocker_state is not None else None
        allow_stop_fallback = True
        if blocker_state is not None and getattr(self.agent, "id", None) in {1, 2} and blocker_id in {1, 2}:
            # #region debug-point E:pairwise-priority
            self._emit_optional_debug_event(
                "collision_pairwise_priority",
                r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-slowdown-cascade.env",
                "http://127.0.0.1:7778/event",
                "global-slowdown-cascade",
                "E",
                "layered_a_star_collision_aware_planner.py:compute_avoidance_response",
                "[DEBUG] pairwise predicted-collision priority snapshot",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "own_state": own_state_name,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_base_state": blocker_base_state,
                    "own_priority": list(self._priority_tuple(self.agent)) if hasattr(self, "_priority_tuple") else None,
                    "blocker_priority": list(self._priority_tuple(blocker_state)) if hasattr(self, "_priority_tuple") else None,
                    "should_yield_to_blocker": self._should_yield_to(blocker_state),
                    "distance": round(blocker_distance, 3),
                },
            )
            # #endregion

        if (
            blocker_state is not None
            and own_state_name in {"CRUISE", "PREQUEUE"}
            and blocker_base_state in {"CRUISE", "PREQUEUE"}
        ):
            # 对向普通巡航/预排队会车时，只允许低优先级一侧进入 stop fallback。
            allow_stop_fallback = self._should_yield_to(blocker_state)
            if not allow_stop_fallback and blocker_distance <= self.MOVING_BLOCKER_STOP_DISTANCE:
                bypass_command = self._direct_bypass_command(
                    position,
                    sequence_of_poses,
                    blocker_state,
                )
                if bypass_command is not None:
                    return {
                        "command": bypass_command,
                        "reason": "predicted_collision_bypass",
                        "blocker_id": blocker_id,
                    }

        if own_state_name == "QUEUING" and blocker_distance <= self.STOP_DISTANCE:
            # v1 只保留最小限度的队列保护：
            # 如果排队状态下已经和前车贴得很近，就不要再硬挤，直接停住。
            return {
                "command": (0.0, 0.0),
                "reason": "queue_collision_guard",
                "blocker_id": blocker_id,
            }

        # 这是 v1 的核心升级：
        # 1. 先从感知范围内挑出最可能撞上的 blocker；
        # 2. 优先要求 agent 重新规划；
        # 3. 如果重规划后路径依然冲突，再由 naive_agent 走 stop fallback。
        return {
            "replan": True,
            "fallback_command": (0.0, 0.0),
            "reason": "predicted_collision",
            "blocker_id": blocker_id,
            "stop_distance": self._stop_distance_for(blocker_id, sensor_observation),
            "allow_stop_fallback": allow_stop_fallback,
        }

    def path_conflicts_with_blocker(self, position, sequence_of_poses, sensor_observation, blocker_id):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is None:
            return False
        blocker_state = getattr(blocker, "userData", blocker)
        return self._conflicts_with_blocker_state(position, sequence_of_poses, blocker_state, blocker)

    def _conflicts_with_blocker_state(self, position, sequence_of_poses, blocker_state, blocker_agent=None):
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return False

        blocker_distance = position.distance(blocker_position)
        if self._is_absolute_stopping_blocker(position, sequence_of_poses, blocker_state):
            return True
        corridor_conflict = self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position)
        if not corridor_conflict and blocker_distance > self.STOP_DISTANCE:
            return False

        # 这里不是简单按欧氏距离判冲突，而是把“我未来几步”和“对方未来几步”
        # 拉出来做一次短时预测。只要短时轨迹会重叠，就认为需要 replan。
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        observed_future = self._future_points(
            blocker_position,
            getattr(blocker_state, "sequence_of_poses", []),
            getattr(blocker_state, "linear_velocity", None)
            or getattr(blocker_agent, "linearVelocity", None),
        )
        if self._future_paths_conflict(own_future, observed_future):
            return True
        return blocker_distance <= self.STOP_DISTANCE and corridor_conflict

    def _inflate_transition_ring(self, dynamic_layer, gridmap, node, inflation):
        for neighbor in gridmap.neighbors(node):
            self.add_inflation(dynamic_layer, node, neighbor, inflation)
            self.add_inflation(dynamic_layer, neighbor, node, inflation)

    def _find_observed_agent_by_id(self, sensor_observation, blocker_id):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) == blocker_id:
                return observed_agent
        return None

    def _current_simulator_step(self):
        port = getattr(getattr(self.agent, "task", None), "port", None)
        port_class = getattr(port, "__class__", None)
        return getattr(port_class, "simulator_step", None)

    def _should_debug_midmap_pair_window(self):
        step = self._current_simulator_step()
        return getattr(self.agent, "id", None) in {1, 2} and step is not None and 165 <= step <= 180

    def _debug_midmap_pair_event(self, hypothesis_id, location, message, data):
        self._emit_optional_debug_event(
            "collision_midmap_pair",
            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env",
            "http://127.0.0.1:7778/event",
            "v4-port-collision",
            hypothesis_id,
            location,
            message,
            data,
        )

    def _distance_to_blocker(self, position, sensor_observation, blocker_id):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is None:
            return float("inf")
        blocker_state = getattr(blocker, "userData", blocker)
        return position.distance(blocker_state.position)

    def _blocker_on_current_corridor(self, position, sequence_of_poses, blocker_position):
        if position.distance(blocker_position) <= self.CORRIDOR_DISTANCE:
            return True
        for pose in list(sequence_of_poses)[:2]:
            if pose.distance(blocker_position) <= self.CORRIDOR_DISTANCE:
                return True
        return False

    def _apply_blocker_barrier(self, dynamic_layer, gridmap, blocker_position, node_inflation=None, edge_inflation=None):
        node_inflation = self.STOPPING_BLOCKER_NODE_INFLATION if node_inflation is None else node_inflation
        edge_inflation = self.STOPPING_BLOCKER_EDGE_INFLATION if edge_inflation is None else edge_inflation
        blocker_origin = agent_to_gridmap(blocker_position, gridmap)
        self._inflate_transition_ring(dynamic_layer, gridmap, blocker_origin, node_inflation)
        for first in gridmap.neighbors(blocker_origin):
            self._inflate_transition_ring(dynamic_layer, gridmap, first, edge_inflation)
            for second in gridmap.neighbors(first):
                if second == blocker_origin:
                    continue
                self._inflate_transition_ring(dynamic_layer, gridmap, second, edge_inflation * 0.45)

    def _stop_distance_for(self, blocker_id, sensor_observation):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is None:
            return self.STOP_DISTANCE
        blocker_state = getattr(blocker, "userData", blocker)
        blocker_base_state = self._base_state_name(blocker_state)
        if blocker_base_state in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return self.MOVING_BLOCKER_STOP_DISTANCE
        return self.STOP_DISTANCE

    def _observed_state_name(self, agent_state):
        if getattr(agent_state, "stopping_active", False):
            return "STOPPING"
        return getattr(getattr(agent_state, "state", None), "name", None)

    def _base_state_name(self, agent_state):
        if getattr(agent_state, "stopping_active", False):
            return getattr(getattr(agent_state, "stopping_base_state", None), "name", None)
        return getattr(getattr(agent_state, "state", None), "name", None)

    def _future_points(self, position, sequence_of_poses, velocity):
        points = [Point(position.x, position.y)]
        for pose in list(sequence_of_poses)[: self.LOOKAHEAD_POINTS]:
            points.append(Point(pose.x, pose.y))

        if len(points) > 1:
            return points

        vx = float(getattr(velocity, "x", 0.0) if velocity is not None else 0.0)
        vy = float(getattr(velocity, "y", 0.0) if velocity is not None else 0.0)
        if velocity is not None and not hasattr(velocity, "x"):
            try:
                vx = float(velocity[0])
                vy = float(velocity[1])
            except Exception:
                vx, vy = 0.0, 0.0
        speed = sqrt(vx * vx + vy * vy)
        if speed < 1e-3:
            return points

        # 如果暂时拿不到 sequence_of_poses，就退化成按当前速度方向投影几步，
        # 让 v1 至少还能做一个粗粒度的“会不会撞”预测。
        step_distance = max(0.8, min(1.4, speed))
        dx = vx / speed
        dy = vy / speed
        for step_idx in range(1, self.LOOKAHEAD_POINTS + 1):
            points.append(
                Point(
                    position.x + dx * step_distance * step_idx,
                    position.y + dy * step_distance * step_idx,
                )
            )
        return points

    def _future_paths_conflict(self, own_future, observed_future):
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            for other_index in range(len(observed_future) - 1):
                other_start = observed_future[other_index]
                other_end = observed_future[other_index + 1]
                square_distance, _ = edge_edge_shortest_square_distance(
                    own_start, own_end, other_start, other_end
                )
                if square_distance <= self.YIELD_SEGMENT_DISTANCE ** 2:
                    return True

        for own_point in own_future[:3]:
            for other_point in observed_future[:3]:
                if own_point.distance(other_point) <= self.YIELD_DISTANCE:
                    return True
        return False

    def _future_path_hits_stopping_agent(self, own_future, stopping_position):
        stop_point = Point(stopping_position.x, stopping_position.y)
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            square_distance, _ = edge_edge_shortest_square_distance(
                own_start, own_end, stop_point, stop_point
            )
            if square_distance <= self.STOPPING_BUFFER_RADIUS ** 2:
                return True
        for own_point in own_future[:3]:
            if own_point.distance(stop_point) <= self.STOPPING_BUFFER_RADIUS:
                return True
        return False

    def _is_absolute_stopping_blocker(self, position, sequence_of_poses, observed_state):
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if getattr(observed_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None):
            # 对方已经明确在给当前 agent 让路时，不要再把它当成“必须双停”的硬 blocker。
            return False
        blocker_position = getattr(observed_state, "position", None)
        if blocker_position is None:
            return False
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        return self._future_path_hits_stopping_agent(own_future, blocker_position)

    def _should_force_bypass_stopping_peer(self, position, sequence_of_poses, observed_state):
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if getattr(observed_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None):
            return False
        if self._base_state_name(self.agent) not in {"CRUISE", "PREQUEUE"}:
            return False
        blocker_position = getattr(observed_state, "position", None)
        if blocker_position is None:
            return False
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        return self._future_path_hits_stopping_agent(own_future, blocker_position)

    def begin_forced_bypass(self, blocker_id):
        self._forced_bypass_blocker_id = blocker_id

    def end_forced_bypass(self):
        self._forced_bypass_blocker_id = None

    # Compatibility helpers for later stages. These stay available for v3/v4
    # inheritance, but they are not part of v1's core decision path anymore.
    def _should_yield_to(self, observed_state):
        if self._observed_state_name(observed_state) == "STOPPING":
            return getattr(observed_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None)
        observed_base_state = self._base_state_name(observed_state)
        own_state_name = self._base_state_name(self.agent)
        if own_state_name in {"CRUISE", "PREQUEUE"} and observed_base_state in {"CRUISE", "PREQUEUE"}:
            return self._tie_break_yield_to(observed_state)
        return self._priority_tuple(observed_state) > self._priority_tuple(self.agent)

    def _should_keep_yield_memory(self, position, sequence_of_poses, observed_state):
        blocker_distance = position.distance(observed_state.position)
        if self._is_absolute_stopping_blocker(position, sequence_of_poses, observed_state):
            return True
        if blocker_distance <= self.REPLAN_DISTANCE:
            return True
        return blocker_distance <= self.MOVING_BLOCKER_STOP_DISTANCE and self._blocker_on_current_corridor(
            position, sequence_of_poses, observed_state.position
        )

    def _tie_break_yield_to(self, observed_state):
        own_priority = self._priority_tuple(self.agent)
        observed_priority = self._priority_tuple(observed_state)
        if observed_priority != own_priority:
            return observed_priority > own_priority
        return getattr(observed_state, "id", 0) < getattr(self.agent, "id", 0)

    def _direct_bypass_command(self, position, sequence_of_poses, blocker_state):
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return None
        goal_pose = None
        for pose in list(sequence_of_poses)[:2]:
            if pose is not None:
                goal_pose = pose
                break
        if goal_pose is None:
            goal_pose = getattr(self.agent, "destination_location", None)
        if goal_pose is None:
            return None

        goal_dir = compute_direction(position, goal_pose)
        radial_dir = compute_direction(blocker_position, position)
        tangent_a = Vector(-radial_dir.y, radial_dir.x)
        tangent_b = Vector(radial_dir.y, -radial_dir.x)
        tangent_dir = tangent_a if tangent_a.dot(goal_dir) >= tangent_b.dot(goal_dir) else tangent_b
        blended = goal_dir + tangent_dir.scale(1.35) + radial_dir.scale(0.45)
        try:
            return blended.normalize().scale(self.agent.cruise_speed).to_tuple()
        except Exception:
            return None

    def _priority_tuple(self, agent_state):
        return (
            self._priority_score(agent_state),
            self._queue_progress_score(agent_state),
            -getattr(agent_state, "id", 0),
        )

    def _priority_score(self, agent_state):
        state_name = self._observed_state_name(agent_state)
        if state_name == "STOPPING":
            return 7
        if self._is_exiting_port(agent_state):
            return 6
        if state_name in {"LOADING", "UNLOADING"}:
            return 5
        if state_name == "QUEUING":
            return 4
        if state_name == "PREQUEUE":
            return 3
        if state_name == "CRUISE":
            return 2
        return 1

    def _queue_progress_score(self, agent_state):
        if self._base_state_name(agent_state) != "QUEUING":
            return 0
        port = self._target_port_for_state(agent_state)
        if port is None:
            return 0
        queue = getattr(port, "queue", None)
        if queue is None:
            return 0
        try:
            slot = port.get_slot(agent_state)
        except Exception:
            return 0
        for index, slot_point in enumerate(getattr(queue, "slots", [])):
            if slot_point.distance(slot) <= 1e-3:
                return len(queue.slots) - index
        return 0

    def _is_exiting_port(self, agent_state):
        state_name = self._base_state_name(agent_state)
        if state_name != "CRUISE":
            return False
        source_port = self._source_port_for_state(agent_state)
        position = getattr(agent_state, "position", None)
        if source_port is None or position is None:
            return False
        anchor = getattr(source_port, "operation_zone", None) or getattr(source_port, "location", None)
        return anchor is not None and anchor.distance(position) <= 3.0

    def _target_port_for_state(self, agent_state):
        return getattr(getattr(agent_state, "task", None), "port", None)

    def _source_port_for_state(self, agent_state):
        task_type = getattr(getattr(agent_state, "task", None), "type", None)
        if task_type == TaskType.GO_TO_UNLOADING_PORT:
            return self._nearest_port_by_type(agent_state, "loading")
        if task_type == TaskType.GO_TO_LOADING_PORT:
            return self._nearest_port_by_type(agent_state, "unloading")
        return None

    def _nearest_port_by_type(self, agent_state, port_type):
        position = getattr(agent_state, "position", None)
        if position is None:
            return None
        candidate_ports = [
            port for port in self._all_ports() if getattr(port, "port_type", None) == port_type
        ]
        if not candidate_ports:
            return None
        nearest = min(
            candidate_ports,
            key=lambda port: position.distance(getattr(port, "operation_zone", None) or port.location),
        )
        anchor = getattr(nearest, "operation_zone", None) or nearest.location
        if anchor.distance(position) <= 3.0:
            return nearest
        return None

    def _all_ports(self):
        server = getattr(self.agent, "server", None)
        if server is None:
            return []
        return list(getattr(server, "loading_ports", [])) + list(getattr(server, "unloading_ports", []))

    def _remember_blocker(self, blocker_id, blocker_position, blocker_distance):
        if blocker_id is None:
            return
        payload = {"distance": blocker_distance}
        if blocker_position is not None:
            payload["last_position"] = Point(blocker_position.x, blocker_position.y)
        self.active_yield_blockers[blocker_id] = payload

    def _point_preview(self, points):
        preview = []
        for point in list(points)[:4]:
            preview.append((round(point.x, 2), round(point.y, 2)))
        return preview

    def _resolve_debug_endpoint(self, cache_key, env_path, fallback_url, fallback_session_id):
        cached = self._debug_endpoint_cache.get(cache_key, None)
        if cached is self._DEBUG_ENDPOINT_DISABLED:
            return None
        if cached is not None:
            return cached
        try:
            debug_url = fallback_url
            session_id = fallback_session_id
            with open(env_path, "r", encoding="utf-8") as env_file:
                env_content = env_file.read()
            for line in env_content.splitlines():
                if line.startswith("DEBUG_SERVER_URL="):
                    debug_url = line.split("=", 1)[1]
                elif line.startswith("DEBUG_SESSION_ID="):
                    session_id = line.split("=", 1)[1]
            endpoint = (debug_url, session_id)
        except Exception:
            endpoint = self._DEBUG_ENDPOINT_DISABLED
        self._debug_endpoint_cache[cache_key] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_DISABLED:
            return None
        return endpoint

    def _resolve_optional_debug_endpoint(self, cache_key, env_path, fallback_url, fallback_session_id):
        cached = self._debug_endpoint_cache.get(cache_key, None)
        if cached is self._DEBUG_ENDPOINT_DISABLED:
            return None
        if cached is not None:
            return cached
        try:
            debug_url = fallback_url
            session_id = fallback_session_id
            debug_enabled = False
            with open(env_path, "r", encoding="utf-8") as env_file:
                env_content = env_file.read()
            for line in env_content.splitlines():
                if line.startswith("DEBUG_SERVER_URL="):
                    debug_url = line.split("=", 1)[1]
                elif line.startswith("DEBUG_SESSION_ID="):
                    session_id = line.split("=", 1)[1]
                elif line.startswith("DEBUG_HTTP_ENABLED="):
                    debug_enabled = line.split("=", 1)[1].strip().lower() in {"1", "true", "yes", "on"}
            endpoint = (debug_url, session_id) if debug_enabled else self._DEBUG_ENDPOINT_DISABLED
        except Exception:
            endpoint = self._DEBUG_ENDPOINT_DISABLED
        self._debug_endpoint_cache[cache_key] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_DISABLED:
            return None
        return endpoint

    def _emit_optional_debug_event(self, cache_key, env_path, fallback_url, fallback_session_id, hypothesis_id, location, message, data):
        endpoint = self._resolve_optional_debug_endpoint(cache_key, env_path, fallback_url, fallback_session_id)
        if endpoint is None:
            return
        debug_url, session_id = endpoint
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    debug_url,
                    data=json.dumps(
                        {
                            "sessionId": session_id,
                            "runId": "pre-fix",
                            "hypothesisId": hypothesis_id,
                            "location": location,
                            "msg": message,
                            "data": data,
                            "ts": 0,
                        }
                    ).encode(),
                    headers={"Content-Type": "application/json"},
                ),
                timeout=0.2,
            ).read()
        except Exception:
            self._debug_endpoint_cache[cache_key] = self._DEBUG_ENDPOINT_DISABLED

    def _debug_event(self, hypothesis_id, message, data):
        return
