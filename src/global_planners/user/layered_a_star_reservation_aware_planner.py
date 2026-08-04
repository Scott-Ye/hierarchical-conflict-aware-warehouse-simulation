from math import pi, sqrt

from geometry import Point
from global_planners.user.layered_a_star_collision_aware_planner import LayeredAStarCollisionAware
from representation.float_to_grid import agent_to_gridmap
from task_managers.task_manager import TaskType


class LayeredAStarReservationAware(LayeredAStarCollisionAware):
    """Collision-aware LayeredAStar with a lightweight rolling reservation window."""

    SENSOR_RANGE = 7.0
    SENSOR_ANGLE = pi
    WINDOW_DISTANCE = 8.0
    REPLAN_DISTANCE = 1.15
    CROWD_REPLAN_THRESHOLD = 3

    RESERVATION_WAYPOINTS = 4
    RESERVATION_DECAY = 0.82
    RESERVATION_NODE_INFLATION = 4.5
    RESERVATION_EDGE_INFLATION = 5.0
    DETOUR_NODE_SCALE = 1.8
    DETOUR_EDGE_SCALE = 2.1
    PROJECTED_CONFLICT_THRESHOLD = 2
    RESERVATION_CONFLICT_WINDOW = 4
    PREDICTIVE_TRACKING_DISTANCE = 5.4
    DETOUR_COMMIT_STEPS = 6
    QUEUE_BLOCKER_TRACKING_DISTANCE = 3.4
    QUEUE_BLOCKER_RELEASE_DISTANCE = 4.2

    def __init__(self, agent):
        super(LayeredAStarReservationAware, self).__init__(agent)
        self.predictive_detours = {}

    def get_dynamic_layer(self, gridmap, sensor_observation, inflation=None):
        dynamic_layer = super(LayeredAStarReservationAware, self).get_dynamic_layer(
            gridmap, sensor_observation, inflation
        )

        observed_agents = sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE)
        for observed_agent in observed_agents:
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if not self._supports_predictive_blocker(observed_state):
                continue
            if not self._should_yield_to(observed_state):
                continue
            # v3 不再依赖 v2 的热点记忆，而是直接收集其他机器人“接下来几步可能经过的节点”。
            # 这些节点会被当成轻量 reservation window，用来提前规避近未来冲突。
            reservation_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
            self._apply_reservation_window(dynamic_layer, gridmap, reservation_nodes)

        # v3 的另一个关键点是“detour commit”：
        # 一旦已经决定绕某个 blocker，不要下一帧立刻把这个决定忘掉，
        # 而是短时间内持续保留这段 reservation，减少左右横跳和双停。
        self._refresh_predictive_detours(gridmap, sensor_observation)
        for commit in self.predictive_detours.values():
            reservation_nodes = commit.get("reservation_nodes", [])
            if reservation_nodes:
                self._apply_reservation_window(
                    dynamic_layer,
                    gridmap,
                    reservation_nodes,
                    node_scale=self.DETOUR_NODE_SCALE,
                    edge_scale=self.DETOUR_EDGE_SCALE,
                )
        return dynamic_layer

    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold=1):
        if super(LayeredAStarReservationAware, self).observe_path(
            gridmap, current_position, sensor_observation, sequence_of_poses, threshold
        ):
            return True

        if not sequence_of_poses:
            return False

        visible_waypoints = []
        remaining_distance = current_position.distance(sequence_of_poses[-1])
        for pose in sequence_of_poses:
            if current_position.distance(pose) < self.WINDOW_DISTANCE and sequence_of_poses[-1].distance(pose) < remaining_distance:
                visible_waypoints.append(pose)

        own_nodes = self._collect_own_reservation_nodes(gridmap, current_position, sequence_of_poses)
        predictive_candidates = []
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            observed_base_state = self._base_state_name(observed_state)
            if not self._supports_predictive_blocker(observed_state):
                self._debug_event(
                    "A",
                    "[DEBUG] v3 observed agent skipped before predictive detour",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "candidate_id": getattr(observed_state, "id", None),
                        "candidate_base_state": observed_base_state,
                        "candidate_state": self._observed_state_name(observed_state),
                        "reason": "unsupported_state",
                    },
                )
                continue
            if not self._should_yield_to(observed_state):
                self._debug_event(
                    "C",
                    "[DEBUG] v3 observed agent skipped by predictive priority gate",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "candidate_id": getattr(observed_state, "id", None),
                        "candidate_base_state": observed_base_state,
                        "candidate_state": self._observed_state_name(observed_state),
                    },
                )
                continue
            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None:
                continue
            blocker_distance = current_position.distance(observed_state.position)
            if blocker_distance <= self.TRACKING_DISTANCE:
                continue
            if blocker_distance > self.PREDICTIVE_TRACKING_DISTANCE:
                continue
            reservation_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
            if not reservation_nodes:
                continue
            conflict_profile = self._reservation_conflict_profile(own_nodes, reservation_nodes)
            if conflict_profile["reverse_edges"] >= 1:
                # reverse edge 表示两车很可能在同一条边上对向相遇。
                # 这是 v3 最重点要比 v1 多解决的一类冲突：双方都在动，但马上要“对冲”。
                predictive_candidates.append((blocker_id, observed_state.position, blocker_distance, reservation_nodes))
                continue

            projected_conflicts = 0
            for waypoint in visible_waypoints:
                waypoint_node = agent_to_gridmap(waypoint, gridmap)
                if waypoint_node in reservation_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1]:
                    projected_conflicts += 1
            if projected_conflicts >= self.PROJECTED_CONFLICT_THRESHOLD and conflict_profile["shared_nodes"] >= 2:
                # 如果双方未来短窗口里会重复踏进同一批节点，
                # 虽然还没到贴脸距离，也先把它列入 predictive detour 候选。
                predictive_candidates.append((blocker_id, observed_state.position, blocker_distance, reservation_nodes))

        if not predictive_candidates:
            return False

        predictive_candidates.sort(key=lambda item: item[2])
        blocker_id, blocker_position, blocker_distance, reservation_nodes = predictive_candidates[0]
        existing_commit = self.predictive_detours.get(blocker_id)
        # 只对最近、最可能冲突的对象提交 detour，
        # 避免同时对很多 blocker 让路导致路径震荡。
        self._remember_predictive_detour(blocker_id, blocker_position, blocker_distance, reservation_nodes)
        self._debug_event(
            "B",
            "[DEBUG] v3 predictive detour committed",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": blocker_id,
                "blocker_distance": blocker_distance,
                "existing_commit": existing_commit is not None,
                "reservation_nodes": list(reservation_nodes),
            },
        )
        return existing_commit is None
        return False

    def compute_avoidance_response(self, position, sensor_observation, sequence_of_poses):
        queue_blocker = self._queue_blocker_for_self(position, sensor_observation, sequence_of_poses)
        if queue_blocker is not None:
            blocker_id, blocker_distance = queue_blocker
            # v3 开始把“排队队尾占住走廊”的情形单独拎出来：
            # 这类冲突靠 v1 的临近 stop 不够，需要强制绕队列，而不是继续顶上去。
            self._remember_blocker(
                blocker_id,
                getattr(self._find_observed_agent_by_id(sensor_observation, blocker_id), "position", None)
                or self.active_yield_blockers.get(blocker_id, {}).get("last_position"),
                blocker_distance,
            )
            self._debug_event(
                "A",
                "[DEBUG] v3 forcing replan around queue blocker",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": blocker_distance,
                },
            )
            return {
                "replan": True,
                "fallback_command": (0.0, 0.0),
                "reason": "queue_detour",
                "blocker_id": blocker_id,
                "stop_distance": self.STOP_DISTANCE,
            }
        stopping_blocker = self._stopping_blocker_for_self(position, sensor_observation, sequence_of_poses)
        if stopping_blocker is not None:
            blocker_id, blocker_distance = stopping_blocker
            # 如果前方 stop 的 agent 本来就是在给我让路，v3 会要求我绕过去，
            # 而不是和它一起互相等，尽量减少“双停僵住”的时间。
            self._debug_event(
                "B",
                "[DEBUG] v3 forcing detour around stopping peer",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": blocker_distance,
                },
            )
            return {
                "replan": True,
                "fallback_command": (0.0, 0.0),
                "reason": "around_stopping_peer",
                "blocker_id": blocker_id,
                "stop_distance": blocker_distance,
                "allow_stop_fallback": False,
            }
        return super(LayeredAStarReservationAware, self).compute_avoidance_response(
            position, sensor_observation, sequence_of_poses
        )

    def path_conflicts_with_blocker(self, position, sequence_of_poses, sensor_observation, blocker_id):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is not None:
            blocker_state = getattr(blocker, "userData", blocker)
            if self._is_queue_blocker(blocker_state):
                blocker_distance = position.distance(blocker_state.position)
                if (
                    self._base_state_name(self.agent) in {"CRUISE", "PREQUEUE"}
                    and blocker_distance <= self.QUEUE_BLOCKER_TRACKING_DISTANCE
                    and self._blocker_on_current_corridor(position, sequence_of_poses, blocker_state.position)
                ):
                    return True
        return super(LayeredAStarReservationAware, self).path_conflicts_with_blocker(
            position, sequence_of_poses, sensor_observation, blocker_id
        )

    def _collect_reservation_nodes(self, observed_agent, gridmap):
        observed_state = getattr(observed_agent, "userData", observed_agent)
        nodes = []

        origin = agent_to_gridmap(observed_state.position, gridmap)
        nodes.append(origin)

        # 优先使用对方已有的 sequence_of_poses；
        # 如果拿不到，就退化为按当前速度方向投影未来若干步。
        path_nodes = self._path_nodes_from_waypoints(observed_state, gridmap, observed_state.position)
        if path_nodes:
            nodes.extend(path_nodes)
        else:
            nodes.extend(self._path_nodes_from_velocity(observed_agent, gridmap, observed_state.position))

        deduped = []
        for node in nodes:
            if not deduped or node != deduped[-1]:
                deduped.append(node)
        return deduped[: self.RESERVATION_WAYPOINTS + 1]

    def _collect_own_reservation_nodes(self, gridmap, current_position, sequence_of_poses):
        nodes = [agent_to_gridmap(current_position, gridmap)]
        for pose in list(sequence_of_poses)[: self.RESERVATION_WAYPOINTS]:
            if current_position.distance(pose) > self.WINDOW_DISTANCE:
                continue
            nodes.append(agent_to_gridmap(pose, gridmap))

        deduped = []
        for node in nodes:
            if not deduped or node != deduped[-1]:
                deduped.append(node)
        return deduped[: self.RESERVATION_WAYPOINTS + 1]

    def _path_nodes_from_waypoints(self, observed_state, gridmap, current_position):
        sequence_of_poses = list(getattr(observed_state, "sequence_of_poses", []) or [])
        if not sequence_of_poses:
            return []

        nodes = []
        for pose in sequence_of_poses:
            if current_position.distance(pose) > self.WINDOW_DISTANCE:
                continue
            nodes.append(agent_to_gridmap(pose, gridmap))
            if len(nodes) >= self.RESERVATION_WAYPOINTS:
                break
        return nodes

    def _path_nodes_from_velocity(self, observed_agent, gridmap, current_position):
        velocity = getattr(observed_agent, "linearVelocity", None)
        if velocity is None:
            return []

        vx = float(getattr(velocity, "x", 0.0))
        vy = float(getattr(velocity, "y", 0.0))
        speed = sqrt(vx * vx + vy * vy)
        if speed < 1e-3:
            return []

        step_distance = max(0.8, min(1.6, speed))
        dx = vx / speed
        dy = vy / speed

        nodes = []
        for step_idx in range(1, self.RESERVATION_WAYPOINTS + 1):
            projected_point = Point(
                current_position.x + dx * step_distance * step_idx,
                current_position.y + dy * step_distance * step_idx,
            )
            nodes.append(agent_to_gridmap(projected_point, gridmap))
        return nodes

    def _apply_reservation_window(self, dynamic_layer, gridmap, reservation_nodes, node_scale=1.0, edge_scale=1.0):
        for step_idx, node in enumerate(reservation_nodes):
            decay = self.RESERVATION_DECAY ** step_idx
            node_inflation = self.RESERVATION_NODE_INFLATION * node_scale * decay
            edge_inflation = self.RESERVATION_EDGE_INFLATION * edge_scale * decay
            # 离当前越远的预约节点影响越小，因此按 step 做衰减。
            # 这里的思想和静态“封路”不同：不是彻底禁止走，而是把别人的未来轨迹
            # 做成一个逐步衰减的软约束，让 A* 更倾向于提前错开。
            self._reserve_node_entries(dynamic_layer, gridmap, node, node_inflation)
            if step_idx > 0:
                previous = reservation_nodes[step_idx - 1]
                self._reserve_transition(dynamic_layer, previous, node, edge_inflation)

    def _reserve_node_entries(self, dynamic_layer, gridmap, node, inflation):
        for neighbor in gridmap.neighbors(node):
            self.add_inflation(dynamic_layer, neighbor, node, inflation)

    def _reserve_transition(self, dynamic_layer, start, end, inflation):
        self.add_inflation(dynamic_layer, start, end, inflation)
        self.add_inflation(dynamic_layer, end, start, inflation)

    def _reservation_paths_conflict(self, own_nodes, observed_nodes):
        conflict_profile = self._reservation_conflict_profile(own_nodes, observed_nodes)
        return conflict_profile["shared_nodes"] > 0 or conflict_profile["reverse_edges"] > 0

    def _reservation_conflict_profile(self, own_nodes, observed_nodes):
        own_slice = own_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1]
        observed_slice = observed_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1]
        shared_nodes = len(set(own_slice).intersection(observed_slice))

        observed_edges = set(zip(observed_slice, observed_slice[1:]))
        reverse_edges = 0
        for own_start, own_end in zip(own_slice, own_slice[1:]):
            if (own_end, own_start) in observed_edges:
                reverse_edges += 1
        return {
            "shared_nodes": shared_nodes,
            "reverse_edges": reverse_edges,
        }

    def _refresh_predictive_detours(self, gridmap, sensor_observation):
        active_ids = set()
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None:
                continue
            commit = self.predictive_detours.get(blocker_id)
            if commit is None:
                continue
            if not self._supports_predictive_blocker(observed_state):
                continue
            if not self._should_yield_to(observed_state):
                continue
            reservation_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
            self._remember_predictive_detour(
                blocker_id,
                observed_state.position,
                self.agent.position.distance(observed_state.position),
                reservation_nodes,
            )
            active_ids.add(blocker_id)

        for blocker_id in list(self.predictive_detours.keys()):
            if blocker_id in active_ids:
                continue
            # blocker 暂时不在感知范围里时，不立刻清除 detour，
            # 而是给一个 TTL，避免“刚绕一下又回去”的抖动。
            commit = self.predictive_detours.get(blocker_id, {})
            commit["ttl"] = commit.get("ttl", 0) - 1
            if commit.get("ttl", 0) <= 0:
                self.predictive_detours.pop(blocker_id, None)
            else:
                self.predictive_detours[blocker_id] = commit

    def _remember_predictive_detour(self, blocker_id, blocker_position, blocker_distance, reservation_nodes):
        self.predictive_detours[blocker_id] = {
            "distance": blocker_distance,
            "last_position": Point(blocker_position.x, blocker_position.y),
            "reservation_nodes": list(reservation_nodes[: self.RESERVATION_WAYPOINTS + 1]),
            "ttl": self.DETOUR_COMMIT_STEPS,
        }

    def _stopping_blocker_for_self(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE"}:
            return None

        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        candidate = None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._observed_state_name(observed_state) != "STOPPING":
                continue
            if getattr(observed_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None):
                continue
            blocker_position = getattr(observed_state, "position", None)
            if blocker_position is None:
                continue
            if not self._future_path_hits_stopping_agent(own_future, blocker_position):
                continue
            blocker_id = getattr(observed_state, "id", None)
            blocker_distance = position.distance(blocker_position)
            if blocker_id is None:
                continue
            if candidate is None or blocker_distance < candidate[1]:
                candidate = (blocker_id, blocker_distance)
        if candidate is None:
            self._debug_event(
                "A",
                "[DEBUG] v3 found no stopping blocker for current agent",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "own_state": own_state_name,
                },
            )
        return candidate

    def _supports_predictive_blocker(self, observed_state):
        observed_base_state = self._base_state_name(observed_state)
        if observed_base_state in {"CRUISE", "PREQUEUE"}:
            return True
        return self._is_queue_blocker(observed_state)

    def _should_yield_to(self, observed_state):
        if super(LayeredAStarReservationAware, self)._should_yield_to(observed_state):
            return True
        return self._is_queue_blocker(observed_state)

    def _should_keep_yield_memory(self, position, sequence_of_poses, observed_state):
        if self._is_queue_blocker(observed_state):
            blocker_distance = position.distance(observed_state.position)
            if blocker_distance <= self.QUEUE_BLOCKER_RELEASE_DISTANCE and \
                self._blocker_on_current_corridor(position, sequence_of_poses, observed_state.position):
                return True
        return super(LayeredAStarReservationAware, self)._should_keep_yield_memory(
            position,
            sequence_of_poses,
            observed_state,
        )

    def _is_queue_blocker(self, observed_state):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE"}:
            return False
        if self._base_state_name(observed_state) != "QUEUING":
            return False
        task_type = getattr(getattr(observed_state, "task", None), "type", None)
        return task_type in {TaskType.GO_TO_LOADING_PORT, TaskType.GO_TO_UNLOADING_PORT}

    def _queue_blocker_for_self(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE"}:
            return None

        candidate = None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if not self._is_queue_blocker(observed_state):
                continue
            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None:
                continue
            blocker_distance = position.distance(observed_state.position)
            if blocker_distance > self.QUEUE_BLOCKER_TRACKING_DISTANCE:
                continue
            if not self._blocker_on_current_corridor(position, sequence_of_poses, observed_state.position):
                continue
            # 只把“真的堵在我当前走廊上的最近队列 blocker”挑出来，
            # 这样 v3 解决的是当前剩余最疼的一类问题，而不是把所有 queue 都视为全局障碍。
            if candidate is None or blocker_distance < candidate[1]:
                candidate = (blocker_id, blocker_distance)
        return candidate
