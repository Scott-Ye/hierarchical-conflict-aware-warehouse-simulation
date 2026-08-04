from math import pi
import json
import urllib.request

from global_planners.sample_global_planner import PriorityQueue
from global_planners.user.layered_a_star_reservation_aware_planner import LayeredAStarReservationAware
from representation.float_to_grid import agent_to_gridmap

DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env"
DEBUG_FALLBACK_URL = "http://127.0.0.1:7778/event"
DEBUG_SESSION_ID = "v4-port-collision"


class LayeredAStarQueueAware(LayeredAStarReservationAware):
    """Reservation-aware LayeredAStar that explicitly avoids active queue corridors."""

    SENSOR_RANGE = 6.8
    SENSOR_ANGLE = pi
    WINDOW_DISTANCE = 6.5
    REPLAN_DISTANCE = 1.2
    HEAT_DECAY = 0.84
    HEAT_CUTOFF = 0.28
    HEAT_INCREMENT = 4.5
    CROWD_REPLAN_THRESHOLD = 3

    QUEUE_NODE_INFLATION = 4.0
    QUEUE_EDGE_INFLATION = 3.0
    QUEUE_NEIGHBOR_INFLATION = 1.5
    QUEUE_ACTIVE_SCALE = 1.0
    NON_TARGET_QUEUE_SCALE = 0.75
    TARGET_QUEUE_SCALE = 0.2
    QUEUE_MEMORY_SCALE = 0.2
    OBSERVED_QUEUE_REPLAN_THRESHOLD = 3
    PORT_PRIORITY_DISTANCE = 3.0
    PORT_PRIORITY_STOP_DISTANCE = 1.25
    PORT_OWNER_PRIORITY_SCALE = 1.2
    FRONT_ZONE_DEPTH = 2
    FRONT_ZONE_ACTIVE_SCALE = 1.45
    FRONT_ZONE_TARGET_SCALE = 0.25
    TAIL_QUEUE_SCALE = 0.16
    TARGET_TAIL_QUEUE_SCALE = 0.08
    EXIT_QUEUE_AVOID_DISTANCE = 3.2
    EXIT_QUEUE_STOP_DISTANCE = 1.35
    EXIT_FRONT_ZONE_STOP_DISTANCE = 2.15
    QUEUE_FOLLOWER_HOLD_DISTANCE = 1.7
    EXIT_SOURCE_QUEUE_SCALE = 1.35
    EXIT_STOPPING_QUEUE_STOP_DISTANCE = 1.8
    EXIT_STOPPING_QUEUE_NODE_INFLATION = 46.0
    EXIT_STOPPING_QUEUE_EDGE_INFLATION = 28.0
    SOURCE_QUEUE_MOUTH_NODE_INFLATION = 72.0
    SOURCE_QUEUE_MOUTH_EDGE_INFLATION = 44.0
    EXIT_ZONE_RADIUS = 3.0
    QUEUE_SLOT_EPSILON = 1e-3
    FRONT_OWNER_NODE_INFLATION = 52.0
    FRONT_OWNER_EDGE_INFLATION = 30.0
    NON_TARGET_PORT_RELEVANCE_DISTANCE = 3.4
    NON_TARGET_PORT_PATH_RELEVANCE_DISTANCE = 2.6
    def get_dynamic_layer(self, gridmap, sensor_observation, inflation=None):
        dynamic_layer = super(LayeredAStarQueueAware, self).get_dynamic_layer(gridmap, sensor_observation, inflation)
        self._forbidden_transitions = set()

        target_port = self._target_port()
        source_port = self._source_port_for_state(self.agent)
        current_node = agent_to_gridmap(self.agent.position, gridmap)
        observed_agents = sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE)
        for port in self._all_ports():
            _is_relevant_port = self._port_is_relevant_to_agent(
                port,
                getattr(self.agent, "position", None),
                getattr(self.agent, "sequence_of_poses", None),
                target_port,
                source_port,
            )
            if not _is_relevant_port:
                continue
            queue_nodes = self._queue_nodes_for_port(port, gridmap)
            if not queue_nodes:
                continue

            queue_load = self._queue_load(port)
            if queue_load <= 0:
                continue

            front_nodes = self._front_zone_nodes(queue_nodes)
            tail_nodes = self._tail_queue_nodes(queue_nodes)
            front_owner_present = self._port_has_priority_owner(sensor_observation, port)
            active_operation_owner_present = self._port_has_active_operation_owner(sensor_observation, port)
            if port != target_port and port != source_port:
                anchor = self._port_anchor(port)
                anchor_distance = getattr(self.agent, "position", None).distance(anchor) if getattr(self.agent, "position", None) is not None and anchor is not None else None
                path_near_port = self._path_passes_near_port(getattr(self.agent, "sequence_of_poses", None), port)
                # #region debug-point A:non-target-port-influence
                self._emit_optional_debug_event(
                    "queue_non_target_port_influence",
                    r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-slowdown-cascade.env",
                    "http://127.0.0.1:7778/event",
                    "global-slowdown-cascade",
                    "A",
                    "layered_a_star_queue_aware_planner.py:get_dynamic_layer",
                    "[DEBUG] non-target port still influences dynamic layer",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "agent_state": self._base_state_name(self.agent),
                        "port_id": getattr(port, "id", None),
                        "target_port_id": getattr(target_port, "id", None),
                        "source_port_id": getattr(source_port, "id", None),
                        "queue_load": queue_load,
                        "anchor_distance": round(anchor_distance, 3) if anchor_distance is not None else None,
                        "path_near_port": path_near_port,
                        "front_owner_present": front_owner_present,
                        "active_operation_owner_present": active_operation_owner_present,
                        "port_type": getattr(port, "port_type", None),
                        "target_port_type": getattr(target_port, "port_type", None),
                        "source_port_type": getattr(source_port, "port_type", None),
                        "same_target_semantic": getattr(port, "id", None) == getattr(target_port, "id", None) and getattr(port, "port_type", None) == getattr(target_port, "port_type", None),
                        "same_source_semantic": getattr(port, "id", None) == getattr(source_port, "id", None) and getattr(port, "port_type", None) == getattr(source_port, "port_type", None),
                    },
                )
                # #endregion

            # v4 重构后采用“两层规则”：
            # 1) 后排 queue slots 允许多人排队，只保留轻量结构代价；
            # 2) port mouth + operation zone 前区单 owner 通行，外部 agent 需要让开。
            front_scale = self.NON_TARGET_QUEUE_SCALE
            if port == target_port:
                front_scale = self.FRONT_ZONE_TARGET_SCALE
            front_scale *= self.QUEUE_ACTIVE_SCALE + min(0.35, 0.06 * queue_load)
            if source_port is not None and port == source_port and self._is_exiting_port(self.agent):
                front_scale = max(front_scale, self.NON_TARGET_QUEUE_SCALE * self.EXIT_SOURCE_QUEUE_SCALE)
            if front_owner_present:
                front_scale = max(
                    front_scale,
                    self.NON_TARGET_QUEUE_SCALE * self.PORT_OWNER_PRIORITY_SCALE * self.FRONT_ZONE_ACTIVE_SCALE,
                )

            tail_scale = self.TAIL_QUEUE_SCALE
            if port == target_port:
                tail_scale = self.TARGET_TAIL_QUEUE_SCALE
            tail_scale *= self.QUEUE_ACTIVE_SCALE + min(0.2, 0.03 * queue_load)

            self._apply_queue_front_zone(dynamic_layer, gridmap, front_nodes, front_scale)
            self._apply_queue_tail_zone(dynamic_layer, gridmap, tail_nodes, tail_scale)
            if front_owner_present and not (source_port is not None and port == source_port and self._is_exiting_port(self.agent)):
                self._apply_front_owner_barrier(dynamic_layer, gridmap, sensor_observation, port)
            if active_operation_owner_present:
                protected_nodes = self._active_operation_owner_footprint(
                    sensor_observation,
                    port,
                    gridmap,
                    queue_nodes,
                )
                # active LOADING/UNLOADING owner 需要一层比 front-zone 更贴近本体的硬禁入区。
                # 即使当前已经因为网格 rounding 踩进保护格，也应该优先把后续搜索导向“撤出保护区”，
                # 而不是完全跳过禁入规则。
                self._mark_protected_node_entries_forbidden(gridmap, protected_nodes)
            if (
                source_port is not None
                and port == source_port
                and self._is_exiting_port(self.agent)
                and front_owner_present
            ):
                self._apply_source_queue_mouth_barrier(dynamic_layer, gridmap, front_nodes)
                self._mark_source_queue_mouth_forbidden_transitions(gridmap, front_nodes)
            if (
                target_port is not None
                and port == target_port
                and not self._is_exiting_port(self.agent)
                and self._exiting_owner_for_port(sensor_observation, port) is not None
                and current_node not in self._front_zone_node_footprint(port, gridmap, queue_nodes)
            ):
                # #region debug-point A:front-zone-entry-block
                self._emit_optional_debug_event(
                    "queue_front_zone_entry_block",
                    DEBUG_ENV_PATH,
                    DEBUG_FALLBACK_URL,
                    DEBUG_SESSION_ID,
                    "A",
                    "layered_a_star_queue_aware_planner.py:get_dynamic_layer",
                    "[DEBUG] v4 blocks front-zone entry",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "target_port_id": getattr(target_port, "id", None),
                        "blocked_port_id": getattr(port, "id", None),
                        "current_node": list(current_node),
                        "front_nodes": [list(n) for n in front_nodes],
                        "owner_candidate": self._exiting_owner_for_port(sensor_observation, port),
                    },
                )
                # #endregion
                # v4 需要把“前区单 owner”落实成真正的入口约束：
                # 当同 port 还有出港 owner 没清掉前区时，后续进港/排队 agent
                # 不应该再切进 mouth / first slot，否则很容易把双方都逼成 STOPPING。
                self._mark_front_zone_entry_forbidden_transitions(gridmap, front_nodes, self._front_zone_node_footprint(port, gridmap, queue_nodes))

        if source_port is not None and self._is_exiting_port(self.agent):
            for observed_agent in observed_agents:
                observed_state = getattr(observed_agent, "userData", observed_agent)
                if not (
                    self._is_same_port_stopping_queue_owner(observed_state, source_port)
                    or self._is_same_port_stopping_front_blocker(observed_state, source_port)
                ):
                    continue
                if self._can_ignore_yielding_blocker(
                    observed_state,
                    source_port,
                    getattr(self.agent, "position", None),
                    getattr(self.agent, "sequence_of_poses", None),
                ):
                    # 这是本轮双停的根因点：
                    # front-zone queue agent 已经因为给当前 exiting agent 让路而停下后，
                    # 不能再被 exiting agent 继续当成“硬障碍”封死整条 corridor。
                    continue
                blocker_position = getattr(observed_state, "position", None)
                if blocker_position is None:
                    continue
                self._apply_blocker_barrier(
                    dynamic_layer,
                    gridmap,
                    blocker_position,
                    node_inflation=self.EXIT_STOPPING_QUEUE_NODE_INFLATION,
                    edge_inflation=self.EXIT_STOPPING_QUEUE_EDGE_INFLATION,
                )
        return dynamic_layer

    def layered_a_star_search(self, graph, start, goal, dynamic_layer):
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {start: None}
        cost_so_far = {start: 0}

        forbidden_transitions = getattr(self, "_forbidden_transitions", set())
        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break
            try:
                for next in graph.neighbors(current):
                    if (current, next) in forbidden_transitions:
                        continue
                    try:
                        new_cost = cost_so_far[current] + graph.cost(current, next) + dynamic_layer[current][next]
                    except Exception:
                        new_cost = cost_so_far[current] + graph.cost(current, next)
                    if next not in cost_so_far or new_cost < cost_so_far[next]:
                        cost_so_far[next] = new_cost
                        priority = new_cost + self.heuristic(goal, next)
                        frontier.put(next, priority)
                        came_from[next] = current
            except Exception:
                return None
        return came_from

    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold=1):
        if super(LayeredAStarQueueAware, self).observe_path(
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

        target_port = self._target_port()
        source_port = self._source_port_for_state(self.agent)
        current_node = agent_to_gridmap(current_position, gridmap)
        queue_conflicts = 0
        for port in self._all_ports():
            if port == target_port:
                continue
            _is_relevant_port = self._port_is_relevant_to_agent(
                port,
                current_position,
                sequence_of_poses,
                target_port,
                source_port,
            )
            if not _is_relevant_port:
                continue
            if self._queue_load(port) <= 0:
                continue

            queue_nodes = self._queue_nodes_for_port(port, gridmap)
            if not queue_nodes:
                continue
            front_detection_nodes = set(queue_nodes[:2]) | self._front_zone_approach_nodes(port, gridmap, queue_nodes)

            if current_node in front_detection_nodes:
                anchor = self._port_anchor(port)
                # #region debug-point B:non-target-frontzone-replan
                self._emit_optional_debug_event(
                    "queue_non_target_frontzone_replan",
                    r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-slowdown-cascade.env",
                    "http://127.0.0.1:7778/event",
                    "global-slowdown-cascade",
                    "B",
                    "layered_a_star_queue_aware_planner.py:observe_path",
                    "[DEBUG] non-target port triggered front-zone replan",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "agent_state": self._base_state_name(self.agent),
                        "port_id": getattr(port, "id", None),
                        "target_port_id": getattr(target_port, "id", None),
                        "source_port_id": getattr(source_port, "id", None),
                        "current_node": list(current_node),
                        "anchor_distance": round(current_position.distance(anchor), 3) if anchor is not None else None,
                        "path_near_port": self._path_passes_near_port(sequence_of_poses, port),
                        "port_type": getattr(port, "port_type", None),
                        "target_port_type": getattr(target_port, "port_type", None),
                        "source_port_type": getattr(source_port, "port_type", None),
                        "same_target_semantic": getattr(port, "id", None) == getattr(target_port, "id", None) and getattr(port, "port_type", None) == getattr(target_port, "port_type", None),
                        "same_source_semantic": getattr(port, "id", None) == getattr(source_port, "id", None) and getattr(port, "port_type", None) == getattr(source_port, "port_type", None),
                    },
                )
                # #endregion
                # #region debug-point D:replan-already-in-conflict
                self._emit_optional_debug_event(
                    "queue_replan_already_in_conflict",
                    DEBUG_ENV_PATH,
                    DEBUG_FALLBACK_URL,
                    DEBUG_SESSION_ID,
                    "D",
                    "layered_a_star_queue_aware_planner.py:observe_path",
                    "[DEBUG] v4 replan triggered after entering non-target front zone",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "port_id": getattr(port, "id", None),
                        "current_node": list(current_node),
                        "queue_nodes": [list(n) for n in queue_nodes[:2]],
                        "approach_nodes": [list(n) for n in sorted(front_detection_nodes - set(queue_nodes[:2]))],
                    },
                )
                # #endregion
                # 如果自己已经切进了非目标 port 的排队入口，就立刻重规划。
                return True

            for waypoint in visible_waypoints:
                waypoint_node = agent_to_gridmap(waypoint, gridmap)
                if waypoint_node in set(queue_nodes) or waypoint_node in front_detection_nodes:
                    queue_conflicts += 1
                    if queue_conflicts >= self.OBSERVED_QUEUE_REPLAN_THRESHOLD:
                        anchor = self._port_anchor(port)
                        # #region debug-point B:non-target-visible-replan
                        self._emit_optional_debug_event(
                            "queue_non_target_visible_replan",
                            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-slowdown-cascade.env",
                            "http://127.0.0.1:7778/event",
                            "global-slowdown-cascade",
                            "B",
                            "layered_a_star_queue_aware_planner.py:observe_path",
                            "[DEBUG] non-target visible queue conflict triggered replan",
                            {
                                "agent_id": getattr(self.agent, "id", None),
                                "agent_state": self._base_state_name(self.agent),
                                "port_id": getattr(port, "id", None),
                                "target_port_id": getattr(target_port, "id", None),
                                "source_port_id": getattr(source_port, "id", None),
                                "waypoint_node": list(waypoint_node),
                                "queue_conflicts": queue_conflicts,
                                "anchor_distance": round(current_position.distance(anchor), 3) if anchor is not None else None,
                                "path_near_port": self._path_passes_near_port(sequence_of_poses, port),
                                "port_type": getattr(port, "port_type", None),
                                "target_port_type": getattr(target_port, "port_type", None),
                                "source_port_type": getattr(source_port, "port_type", None),
                                "same_target_semantic": getattr(port, "id", None) == getattr(target_port, "id", None) and getattr(port, "port_type", None) == getattr(target_port, "port_type", None),
                                "same_source_semantic": getattr(port, "id", None) == getattr(source_port, "id", None) and getattr(port, "port_type", None) == getattr(source_port, "port_type", None),
                            },
                        )
                        # #endregion
                        # #region debug-point D:replan-visible-conflict
                        self._emit_optional_debug_event(
                            "queue_replan_visible_conflict",
                            DEBUG_ENV_PATH,
                            DEBUG_FALLBACK_URL,
                            DEBUG_SESSION_ID,
                            "D",
                            "layered_a_star_queue_aware_planner.py:observe_path",
                            "[DEBUG] v4 replan triggered by visible queue conflict",
                            {
                                "agent_id": getattr(self.agent, "id", None),
                                "port_id": getattr(port, "id", None),
                                "waypoint_node": list(waypoint_node),
                                "queue_conflicts": queue_conflicts,
                            },
                        )
                        # #endregion
                        return True
        return False

    def compute_avoidance_response(self, position, sensor_observation, sequence_of_poses):
        # 不再通过 v4 主动下发停车命令来处理前区/队列让行，
        # 以满足“后续任何操作都不影响 agent 行进速度”的约束。
        persistent_port_hold_response = self._persistent_port_corridor_hold_response(
            position,
            sensor_observation,
            sequence_of_poses,
        )
        if persistent_port_hold_response is not None:
            return persistent_port_hold_response
        exiting_response = self._exit_port_queue_response(position, sensor_observation, sequence_of_poses)
        if exiting_response is not None:
            return exiting_response
        port_priority_response = self._port_priority_response(position, sensor_observation, sequence_of_poses)
        if port_priority_response is not None:
            return port_priority_response
        base_response = super(LayeredAStarQueueAware, self).compute_avoidance_response(
            position,
            sensor_observation,
            sequence_of_poses,
        )
        if base_response is None:
            return None

        if base_response.get("reason") == "around_stopping_peer":
            blocker = self._find_observed_agent_by_id(sensor_observation, base_response.get("blocker_id"))
            blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
            own_base_state = self._base_state_name(self.agent)
            blocker_base_state = self._base_state_name(blocker_state) if blocker_state is not None else None
            # v4 的中场会车里，如果“绕开已停让行车”的重规划失败且冲突仍在，
            # 继续把控制权交给 local planner 会沿原方向顶进对方。
            # 这里只对低优先级的巡航侧开放 stop fallback，高优先级一侧继续通过，
            # 避免把“单边让行”重新放大成“双边互停”。
            if (
                own_base_state in {"CRUISE", "PREQUEUE"}
                and blocker_base_state in {"CRUISE", "PREQUEUE"}
                and getattr(blocker_state, "id", None) is not None
                and getattr(blocker_state, "id", None) < getattr(self.agent, "id", None)
            ):
                base_response["allow_stop_fallback"] = True
                base_response["suppress_stop_when_path_clear"] = True
        return base_response

    def _persistent_port_corridor_hold_response(self, position, sensor_observation, sequence_of_poses):
        if not getattr(self.agent, "stopping_active", False):
            return None
        if getattr(self.agent, "stopping_reason", None) != "port_corridor_yield":
            return None

        blocker_id = getattr(self.agent, "stopping_for_agent_id", None)
        if blocker_id is None:
            return None

        blocker = self._find_observed_agent_by_id_any_angle(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_state is None or blocker_position is None:
            return None

        blocker_distance = position.distance(blocker_position)
        if blocker_distance > self.PORT_PRIORITY_STOP_DISTANCE + 0.35:
            return None
        if not self._blocker_still_occupies_port_corridor(position, sequence_of_poses, blocker_state):
            return None

        return {
            "replan": True,
            "fallback_command": (0.0, 0.0),
            "reason": "port_corridor_yield",
            "blocker_id": blocker_id,
            "stop_distance": self.PORT_PRIORITY_STOP_DISTANCE + 0.35,
        }

    def should_extend_bottom_port_corridor_hold(self, position, sensor_observation, sequence_of_poses, blocker_id):
        if getattr(self.agent, "stopping_reason", None) != "port_corridor_yield":
            return False
        if self._base_state_name(self.agent) != "QUEUING":
            return False

        target_port = self._target_port()
        gridmap = getattr(self.agent, "static_environment", None)
        if target_port is None or gridmap is None:
            return False
        if self._port_border_offset(target_port, gridmap) != (0, -1):
            return False
        if not self._is_near_port_front_zone(position, target_port):
            return False

        blocker = self._find_observed_agent_by_id_any_angle(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        if blocker_state is None:
            return False
        if not self._is_exiting_port(blocker_state):
            return False
        if self._source_port_for_state(blocker_state) != target_port:
            return False
        if not self._occupies_front_zone_footprint(blocker_state, target_port):
            return False

        return self._conflicts_with_blocker_state(
            position,
            sequence_of_poses,
            blocker_state,
            blocker,
        )

    def _conflicts_with_blocker_state(self, position, sequence_of_poses, blocker_state, blocker_agent=None):
        if self._is_exiting_port(self.agent):
            source_port = self._source_port_for_state(self.agent)
            if self._is_same_port_yielding_to_self(blocker_state, source_port):
                return False
            if self._is_same_port_queue_yielder(blocker_state, source_port):
                # v4 的结构化规则里，同 port queueing 对 exiting owner 永远让路。
                # 因此 exiting agent 不应再把 queue head / follower 当成“必须停住”的碰撞 blocker。
                return False
        return super(LayeredAStarQueueAware, self)._conflicts_with_blocker_state(
            position,
            sequence_of_poses,
            blocker_state,
            blocker_agent,
        )

    def _should_yield_to(self, observed_state):
        if self._port_owner_should_hold(observed_state):
            return False
        if self._must_yield_to_port_owner(observed_state):
            return True
        return super(LayeredAStarQueueAware, self)._should_yield_to(observed_state)

    def _is_queue_blocker(self, observed_state):
        if self._is_exiting_port(self.agent):
            return False
        if self._port_owner_should_hold(observed_state):
            return False
        return super(LayeredAStarQueueAware, self)._is_queue_blocker(observed_state)

    def _all_ports(self):
        server = getattr(self.agent, "server", None)
        if server is None:
            return []
        return list(getattr(server, "loading_ports", [])) + list(getattr(server, "unloading_ports", []))

    def _find_observed_agent_by_id_any_angle(self, sensor_observation, blocker_id):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, 2 * pi
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) == blocker_id:
                return observed_agent
        return None

    def _target_port(self):
        return getattr(getattr(self.agent, "task", None), "port", None)

    def _port_anchor(self, port):
        if port is None:
            return None
        return getattr(port, "operation_zone", None) or getattr(port, "location", None)

    def _path_passes_near_port(self, sequence_of_poses, port):
        anchor = self._port_anchor(port)
        if anchor is None or not sequence_of_poses:
            return False
        for pose in list(sequence_of_poses)[:6]:
            if pose.distance(anchor) <= self.NON_TARGET_PORT_PATH_RELEVANCE_DISTANCE:
                return True
        return False

    def _port_is_relevant_to_agent(self, port, position, sequence_of_poses, target_port=None, source_port=None):
        if port is None:
            return False
        if port == target_port or port == source_port:
            return True
        anchor = self._port_anchor(port)
        if anchor is None or position is None:
            return False
        if position.distance(anchor) <= self.NON_TARGET_PORT_RELEVANCE_DISTANCE:
            return True
        return self._path_passes_near_port(sequence_of_poses, port)

    def _queue_load(self, port):
        queue = getattr(port, "queue", None)
        if queue is None:
            return 0
        return queue.num_agents()

    def _queue_nodes_for_port(self, port, gridmap):
        queue = getattr(port, "queue", None)
        if queue is None:
            return []

        nodes = []
        # queue slots + operation_zone 共同定义了 queue corridor 的核心节点。
        for slot in getattr(queue, "slots", []):
            nodes.append(agent_to_gridmap(slot, gridmap))

        operation_zone = getattr(port, "operation_zone", None)
        if operation_zone is not None:
            nodes.insert(0, agent_to_gridmap(operation_zone, gridmap))

        deduped = []
        for node in nodes:
            if not deduped or node != deduped[-1]:
                deduped.append(node)
        return deduped

    def _port_border_offset(self, port, gridmap):
        if port is None or gridmap is None:
            return None
        location = getattr(port, "location", None)
        if location is None:
            return None

        max_x = gridmap.num_x_grids - 1
        max_y = gridmap.num_y_grids - 1
        if location.y >= max_y - 1:
            return (0, -1)
        if location.y <= 1:
            return (0, 1)
        if location.x <= 1:
            return (1, 0)
        if location.x >= max_x - 1:
            return (-1, 0)
        return None

    def _front_zone_approach_nodes(self, port, gridmap=None, queue_nodes=None):
        if port is None:
            return set()
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return set()
        offset = self._port_border_offset(port, gridmap)
        if offset is None:
            return set()

        queue_nodes = queue_nodes or self._queue_nodes_for_port(port, gridmap)
        approach_nodes = set()
        for node in self._front_zone_nodes(queue_nodes):
            candidate = (node[0] + offset[0], node[1] + offset[1])
            if gridmap.in_bounds(candidate) and gridmap.passable(candidate):
                approach_nodes.add(candidate)
        return approach_nodes

    def _apply_queue_front_zone(self, dynamic_layer, gridmap, queue_nodes, scale):
        for step_idx, node in enumerate(queue_nodes):
            decay = max(0.45, 1.0 - 0.12 * step_idx)
            node_inflation = self.QUEUE_NODE_INFLATION * scale * decay
            edge_inflation = self.QUEUE_EDGE_INFLATION * scale * decay
            neighbor_inflation = self.QUEUE_NEIGHBOR_INFLATION * scale * decay

            self._reserve_node_entries(dynamic_layer, gridmap, node, node_inflation)
            for neighbor in gridmap.neighbors(node):
                self.add_inflation(dynamic_layer, node, neighbor, neighbor_inflation)

            if step_idx > 0:
                previous = queue_nodes[step_idx - 1]
                self._reserve_transition(dynamic_layer, previous, node, edge_inflation)

    def _apply_queue_tail_zone(self, dynamic_layer, gridmap, queue_nodes, scale):
        for step_idx, node in enumerate(queue_nodes):
            decay = max(0.3, 1.0 - 0.08 * step_idx)
            node_inflation = self.QUEUE_NODE_INFLATION * scale * decay
            edge_inflation = self.QUEUE_EDGE_INFLATION * scale * decay

            # 后排 slots 允许多人排队，只保留轻量代价，避免把整条 queue 都当成单 owner 区。
            self._reserve_node_entries(dynamic_layer, gridmap, node, node_inflation * 0.55)
            if step_idx > 0:
                previous = queue_nodes[step_idx - 1]
                self._reserve_transition(dynamic_layer, previous, node, edge_inflation * 0.55)

    def _reserve_node_entries(self, dynamic_layer, gridmap, node, inflation):
        for neighbor in gridmap.neighbors(node):
            self.add_inflation(dynamic_layer, neighbor, node, inflation)

    def _reserve_transition(self, dynamic_layer, start, end, inflation):
        self.add_inflation(dynamic_layer, start, end, inflation)
        self.add_inflation(dynamic_layer, end, start, inflation)

    def _apply_source_queue_mouth_barrier(self, dynamic_layer, gridmap, queue_nodes):
        mouth_nodes = list(queue_nodes[:2])
        for step_idx, node in enumerate(mouth_nodes):
            node_inflation = self.SOURCE_QUEUE_MOUTH_NODE_INFLATION * (1.0 - 0.18 * step_idx)
            edge_inflation = self.SOURCE_QUEUE_MOUTH_EDGE_INFLATION * (1.0 - 0.18 * step_idx)
            self._reserve_node_entries(dynamic_layer, gridmap, node, node_inflation)
            for neighbor in gridmap.neighbors(node):
                self.add_inflation(dynamic_layer, node, neighbor, edge_inflation)
            if step_idx > 0:
                previous = mouth_nodes[step_idx - 1]
                self._reserve_transition(dynamic_layer, previous, node, edge_inflation)

    def _apply_front_owner_barrier(self, dynamic_layer, gridmap, sensor_observation, port):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._front_zone_port_for_state(observed_state) != port:
                continue
            if getattr(observed_state, "id", None) == getattr(self.agent, "id", None):
                continue
            blocker_position = getattr(observed_state, "position", None)
            if blocker_position is None:
                continue
            self._apply_blocker_barrier(
                dynamic_layer,
                gridmap,
                blocker_position,
                node_inflation=self.FRONT_OWNER_NODE_INFLATION,
                edge_inflation=self.FRONT_OWNER_EDGE_INFLATION,
            )

    def _mark_source_queue_mouth_forbidden_transitions(self, gridmap, queue_nodes):
        if len(queue_nodes) < 2:
            return
        operation_node = queue_nodes[0]
        first_slot_node = queue_nodes[1]

        # 对离开 port 的 agent 来说，source port 的 queue mouth 不再是“高代价区”，
        # 而是搜索阶段就直接跳过的禁行过渡。
        self._forbidden_transitions.add((operation_node, first_slot_node))
        self._forbidden_transitions.add((first_slot_node, operation_node))

        for neighbor in gridmap.neighbors(first_slot_node):
            self._forbidden_transitions.add((neighbor, first_slot_node))
            self._forbidden_transitions.add((first_slot_node, neighbor))

    def _mark_front_zone_entry_forbidden_transitions(self, gridmap, front_nodes, footprint_nodes=None):
        if not front_nodes:
            return
        protected_nodes = set(footprint_nodes or front_nodes)
        self._mark_protected_node_entries_forbidden(gridmap, protected_nodes)

    def _mark_protected_node_entries_forbidden(self, gridmap, protected_nodes):
        for node in protected_nodes:
            for neighbor in gridmap.neighbors(node):
                if neighbor in protected_nodes:
                    continue
                self._forbidden_transitions.add((neighbor, node))

    def _port_priority_response(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return None

        candidate = None
        target_port = self._target_port()
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if not self._must_yield_to_port_owner(observed_state):
                continue
            if own_state_name == "QUEUING" and not self._is_near_port_front_zone(getattr(observed_state, "position", None), target_port):
                continue

            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None:
                continue

            blocker_distance = position.distance(observed_state.position)
            if blocker_distance > self.PORT_PRIORITY_DISTANCE:
                continue
            if not self._blocker_on_current_corridor(position, sequence_of_poses, observed_state.position):
                continue

            if candidate is None or blocker_distance < candidate[1]:
                candidate = (blocker_id, blocker_distance)

        if candidate is None:
            return None

        blocker_id, blocker_distance = candidate
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        # #region debug-point C:port-priority-selected
        self._emit_optional_debug_event(
            "queue_port_priority_selected",
            DEBUG_ENV_PATH,
            DEBUG_FALLBACK_URL,
            DEBUG_SESSION_ID,
            "C",
            "layered_a_star_queue_aware_planner.py:_port_priority_response",
            "[DEBUG] v4 selected port-priority blocker",
            {
                "agent_id": getattr(self.agent, "id", None),
                "agent_state": self._base_state_name(self.agent),
                "blocker_id": blocker_id,
                "blocker_state": self._observed_state_name(blocker_state) if blocker_state is not None else None,
                "blocker_distance": round(blocker_distance, 3),
                "position": [round(position.x, 2), round(position.y, 2)],
                "blocker_position": [round(getattr(getattr(blocker_state, 'position', None), 'x', 0.0), 2), round(getattr(getattr(blocker_state, 'position', None), 'y', 0.0), 2)] if blocker_state is not None else None,
                "path_preview": self._point_preview(sequence_of_poses),
                "target_port_id": getattr(self._target_port(), "id", None),
            },
        )
        # #endregion
        self._debug_event(
            "A",
            "[DEBUG] v4 yielding to port corridor owner",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": blocker_id,
                "blocker_distance": blocker_distance,
            },
        )
        return {
            "replan": True,
            "fallback_command": (0.0, 0.0),
            "reason": "port_corridor_yield",
            "blocker_id": blocker_id,
            "stop_distance": self.PORT_PRIORITY_STOP_DISTANCE,
        }

    def _exit_port_queue_response(self, position, sensor_observation, sequence_of_poses):
        if not self._is_exiting_port(self.agent):
            return None

        source_port = self._source_port_for_state(self.agent)
        if source_port is None:
            return None

        candidate = None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            is_stopping_front_blocker = self._is_same_port_stopping_front_blocker(observed_state, source_port)
            if self._can_ignore_yielding_blocker(
                observed_state,
                source_port,
                position,
                sequence_of_poses,
            ):
                # 对已经显式“停下来给我让路”的 same-port agent，
                # exiting owner 不应再继续把它当成需要 replan / stop 的 blocker。
                # #region debug-point B:skip-yielding-self
                self._emit_optional_debug_event(
                    "queue_skip_yielding_self",
                    DEBUG_ENV_PATH,
                    DEBUG_FALLBACK_URL,
                    DEBUG_SESSION_ID,
                    "B",
                    "layered_a_star_queue_aware_planner.py:_exit_port_queue_response",
                    "[DEBUG] v4 exit owner skips same-port yielding blocker",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "observed_id": getattr(observed_state, "id", None),
                        "source_port_id": getattr(source_port, "id", None),
                        "observed_state": self._observed_state_name(observed_state),
                        "base_state": self._base_state_name(observed_state),
                    },
                )
                # #endregion
                continue
            if self._is_same_port_queue_yielder(observed_state, source_port):
                # #region debug-point B:skip-queue-yielder
                self._emit_optional_debug_event(
                    "queue_skip_queue_yielder",
                    DEBUG_ENV_PATH,
                    DEBUG_FALLBACK_URL,
                    DEBUG_SESSION_ID,
                    "B",
                    "layered_a_star_queue_aware_planner.py:_exit_port_queue_response",
                    "[DEBUG] v4 exit owner skips same-port queueing blocker",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "observed_id": getattr(observed_state, "id", None),
                        "source_port_id": getattr(source_port, "id", None),
                        "observed_position": [round(getattr(getattr(observed_state, 'position', None), 'x', 0.0), 2), round(getattr(getattr(observed_state, 'position', None), 'y', 0.0), 2)],
                        "front_zone_port_id": getattr(self._front_zone_port_for_state(observed_state), "id", None),
                    },
                )
                # #endregion
                continue
            if not is_stopping_front_blocker:
                if self._base_state_name(observed_state) not in {"QUEUING", "LOADING", "UNLOADING"}:
                    continue
                if not self._is_same_port_exit_blocker(observed_state, source_port):
                    continue
            else:
                if not self._is_near_port_front_zone(getattr(observed_state, "position", None), source_port):
                    continue

            if is_stopping_front_blocker and self._target_port_for_state(observed_state) != source_port:
                continue

            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None:
                continue

            blocker_distance = position.distance(observed_state.position)
            if blocker_distance > self.EXIT_QUEUE_AVOID_DISTANCE:
                continue
            if not (
                self._blocker_on_current_corridor(position, sequence_of_poses, observed_state.position)
                or self._is_near_port_front_zone(position, source_port)
                or self._is_near_port_front_zone(observed_state.position, source_port)
            ):
                continue

            if candidate is None or blocker_distance < candidate[1]:
                candidate = (blocker_id, blocker_distance)

        if candidate is None:
            return None

        blocker_id, blocker_distance = candidate
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        stop_distance = self.EXIT_QUEUE_STOP_DISTANCE
        if blocker_state is not None and self._observed_state_name(blocker_state) == "STOPPING":
            stop_distance = self.EXIT_STOPPING_QUEUE_STOP_DISTANCE
        elif blocker_state is not None and self._is_near_port_front_zone(getattr(blocker_state, "position", None), source_port):
            stop_distance = max(stop_distance, self.EXIT_FRONT_ZONE_STOP_DISTANCE)
        self._debug_event(
            "A",
            "[DEBUG] v4 exiting agent reroutes around same-port queue owner",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": blocker_id,
                "blocker_distance": blocker_distance,
                "blocker_state": self._observed_state_name(blocker_state) if blocker_state is not None else None,
                "is_stopping_front_blocker": blocker_state is not None and self._is_same_port_stopping_front_blocker(blocker_state, source_port),
                "source_port_id": getattr(source_port, "id", None),
            },
        )
        return {
            "replan": True,
            "fallback_command": (0.0, 0.0),
            "reason": "exit_queue_owner_avoid",
            "blocker_id": blocker_id,
            "stop_distance": stop_distance,
        }

    def path_conflicts_with_blocker(self, position, sequence_of_poses, sensor_observation, blocker_id):
        if self._is_exiting_port(self.agent):
            source_port = self._source_port_for_state(self.agent)
            blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
            blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
            if blocker_state is not None and self._can_ignore_yielding_blocker(
                blocker_state,
                source_port,
                position,
                sequence_of_poses,
            ):
                # #region debug-point B:path-conflict-suppressed
                self._emit_optional_debug_event(
                    "queue_path_conflict_suppressed",
                    DEBUG_ENV_PATH,
                    DEBUG_FALLBACK_URL,
                    DEBUG_SESSION_ID,
                    "B",
                    "layered_a_star_queue_aware_planner.py:path_conflicts_with_blocker",
                    "[DEBUG] v4 suppresses path conflict for yielding blocker",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "source_port_id": getattr(source_port, "id", None),
                    },
                )
                # #endregion
                return False
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
        if blocker_state is not None and self._is_retreating_from_source_port_owner(
            position,
            sequence_of_poses,
            blocker_state,
        ):
            next_waypoint = list(sequence_of_poses)[:1]
            blocker_position = getattr(blocker_state, "position", None)
            # #region debug-point C:source-retreat-conflict-suppressed
            self._emit_optional_debug_event(
                "queue_source_retreat_conflict_suppressed",
                DEBUG_ENV_PATH,
                DEBUG_FALLBACK_URL,
                DEBUG_SESSION_ID,
                "C",
                "layered_a_star_queue_aware_planner.py:path_conflicts_with_blocker",
                "[DEBUG] v4 suppresses source-port owner conflict for exit retreat",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "agent_state": self._base_state_name(self.agent),
                    "blocker_state": self._observed_state_name(blocker_state),
                    "current_distance": round(position.distance(blocker_position), 3) if blocker_position is not None else None,
                    "next_distance": round(next_waypoint[0].distance(blocker_position), 3) if blocker_position is not None and next_waypoint else None,
                    "path_preview": self._point_preview(sequence_of_poses),
                },
            )
            # #endregion
            return False
        if blocker_state is not None and self._is_retreating_from_active_operation_owner(
            position,
            sequence_of_poses,
            blocker_state,
        ):
            next_waypoint = list(sequence_of_poses)[:1]
            blocker_position = getattr(blocker_state, "position", None)
            # #region debug-point C:retreat-conflict-suppressed
            self._emit_optional_debug_event(
                "queue_retreat_conflict_suppressed",
                DEBUG_ENV_PATH,
                DEBUG_FALLBACK_URL,
                DEBUG_SESSION_ID,
                "C",
                "layered_a_star_queue_aware_planner.py:path_conflicts_with_blocker",
                "[DEBUG] v4 suppresses active-owner conflict for retreat path",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "agent_state": self._base_state_name(self.agent),
                    "blocker_state": self._observed_state_name(blocker_state),
                    "current_distance": round(position.distance(blocker_position), 3) if blocker_position is not None else None,
                    "next_distance": round(next_waypoint[0].distance(blocker_position), 3) if blocker_position is not None and next_waypoint else None,
                    "path_preview": self._point_preview(sequence_of_poses),
                },
            )
            # #endregion
            return False
        if blocker_state is not None and self._is_active_operation_owner(blocker_state, self._target_port()):
            next_waypoint = list(sequence_of_poses)[:1]
            blocker_position = getattr(blocker_state, "position", None)
            # #region debug-point C:retreat-conflict-kept
            self._emit_optional_debug_event(
                "queue_retreat_conflict_kept",
                DEBUG_ENV_PATH,
                DEBUG_FALLBACK_URL,
                DEBUG_SESSION_ID,
                "C",
                "layered_a_star_queue_aware_planner.py:path_conflicts_with_blocker",
                "[DEBUG] v4 keeps active-owner conflict after replan",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "agent_state": self._base_state_name(self.agent),
                    "blocker_state": self._observed_state_name(blocker_state),
                    "current_distance": round(position.distance(blocker_position), 3) if blocker_position is not None else None,
                    "next_distance": round(next_waypoint[0].distance(blocker_position), 3) if blocker_position is not None and next_waypoint else None,
                    "path_preview": self._point_preview(sequence_of_poses),
                },
            )
            # #endregion
        return super(LayeredAStarQueueAware, self).path_conflicts_with_blocker(
            position,
            sequence_of_poses,
            sensor_observation,
            blocker_id,
        )

    def _front_zone_queue_hold_response(self, position, sensor_observation):
        return None

    def _queue_follower_hold_response(self, position, sensor_observation):
        return None

    def _must_yield_to_port_owner(self, observed_state):
        target_port = self._target_port()
        if self._is_stopping_target_port_front_blocker(observed_state, target_port):
            return True
        own_role = self._port_role_rank(self.agent)
        observed_role = self._port_role_rank(observed_state)
        return observed_role > own_role

    def _port_owner_should_hold(self, observed_state):
        own_role = self._port_role_rank(self.agent)
        if own_role < 2:
            return False
        observed_role = self._port_role_rank(observed_state)
        return observed_role < own_role

    def _port_role_rank(self, agent_state):
        if self._is_exiting_port(agent_state):
            return 4
        state_name = self._base_state_name(agent_state)
        if state_name in {"LOADING", "UNLOADING"}:
            return 3
        if state_name == "QUEUING" and self._front_zone_port_for_state(agent_state) is not None:
            return 2
        if self._is_same_port_stopping_front_blocker(agent_state, self._target_port_for_state(agent_state)):
            return 2
        return 0

    def _port_has_priority_owner(self, sensor_observation, port):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._is_same_port_stopping_front_blocker(observed_state, port):
                return True
            if self._port_role_rank(observed_state) < 2:
                continue
            if self._front_zone_port_for_state(observed_state) == port:
                return True
        return False

    def _port_has_active_operation_owner(self, sensor_observation, port):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._is_active_operation_owner(observed_state, port):
                return True
        return False

    def _port_for_priority_owner(self, agent_state):
        return self._front_zone_port_for_state(agent_state)

    def _is_same_port_stopping_queue_owner(self, observed_state, source_port):
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if self._base_state_name(observed_state) not in {"QUEUING", "LOADING", "UNLOADING"}:
            return False
        return self._front_zone_port_for_state(observed_state) == source_port

    def _is_active_operation_owner(self, observed_state, port):
        if port is None:
            return False
        if self._base_state_name(observed_state) not in {"LOADING", "UNLOADING"}:
            return False
        return self._target_port_for_state(observed_state) == port

    def _is_same_port_stopping_front_blocker(self, observed_state, port):
        if port is None:
            return False
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if self._target_port_for_state(observed_state) != port:
            return False
        base_state = self._base_state_name(observed_state)
        if base_state not in {"LOADING", "UNLOADING"} and not self._is_exiting_port(observed_state):
            return False
        return self._occupies_front_zone_footprint(observed_state, port)

    def _is_stopping_target_port_front_blocker(self, observed_state, port):
        if port is None:
            return False
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if self._is_same_port_stopping_front_blocker(observed_state, port):
            return True
        if self._target_port_for_state(observed_state) != port:
            return False
        # CRUISE/PREQUEUE agent 也可能在 mouth 里因为让行而被 stop fallback 停住；
        # 即使它正是“为我停车”，只要身体还压在 front-zone/mouth 里，也不能提前放行。
        return self._occupies_front_zone_footprint(observed_state, port)

    def _is_same_port_yielding_to_self(self, observed_state, port):
        if port is None:
            return False
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if getattr(observed_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None):
            return False
        if (
            getattr(observed_state, "stopping_reason", None) == "port_corridor_yield"
            and self._target_port_for_state(observed_state) == port
            and not self._occupies_front_zone_footprint(observed_state, port)
        ):
            return True
        if not (
            self._is_same_port_stopping_queue_owner(observed_state, port)
            or self._is_same_port_stopping_front_blocker(observed_state, port)
        ):
            return False
        return True

    def _front_zone_nodes(self, queue_nodes):
        return list(queue_nodes[: self.FRONT_ZONE_DEPTH])

    def _tail_queue_nodes(self, queue_nodes):
        return list(queue_nodes[self.FRONT_ZONE_DEPTH :])

    def _front_zone_port_for_state(self, agent_state):
        if self._is_exiting_port(agent_state):
            return self._source_port_for_state(agent_state)

        state_name = self._base_state_name(agent_state)
        if state_name == "QUEUING":
            target_port = self._target_port_for_state(agent_state)
            if target_port is not None and self._occupies_front_zone_footprint(agent_state, target_port):
                return target_port
        if state_name in {"LOADING", "UNLOADING"}:
            return self._target_port_for_state(agent_state)
        return None

    def _exiting_owner_for_port(self, sensor_observation, port):
        if port is None:
            return None

        candidate = None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if not self._is_exiting_port(observed_state):
                continue
            if self._source_port_for_state(observed_state) != port:
                continue
            if not self._exit_owner_blocks_front_zone(observed_state, port):
                continue

            blocker_id = getattr(observed_state, "id", None)
            blocker_position = getattr(observed_state, "position", None)
            if blocker_id is None or blocker_position is None:
                continue

            blocker_distance = self.agent.position.distance(blocker_position)
            if candidate is None or blocker_distance < candidate[1]:
                candidate = (blocker_id, blocker_distance)
        return candidate

    def _queue_slot_index_for_state(self, agent_state, port):
        queue = getattr(port, "queue", None)
        if queue is None:
            return None
        try:
            slot = port.get_slot(agent_state)
        except Exception:
            return None
        for index, slot_point in enumerate(getattr(queue, "slots", [])):
            if slot_point.distance(slot) <= self.QUEUE_SLOT_EPSILON:
                return index
        return None

    def _current_slot_for_state(self, agent_state, port=None):
        port = port or self._target_port_for_state(agent_state)
        if port is None:
            return None
        try:
            return port.get_slot(agent_state)
        except Exception:
            return None

    def _is_same_port_exit_blocker(self, observed_state, source_port):
        if source_port is None:
            return False

        state_name = self._base_state_name(observed_state)
        if state_name in {"LOADING", "UNLOADING"}:
            return self._target_port_for_state(observed_state) == source_port
        if state_name != "QUEUING":
            return False

        target_port = self._target_port_for_state(observed_state)
        if target_port != source_port:
            return False
        return self._front_zone_port_for_state(observed_state) == source_port

    def _is_near_port_front_zone(self, position, port):
        if position is None or port is None:
            return False

        operation_zone = getattr(port, "operation_zone", None)
        if operation_zone is not None and operation_zone.distance(position) <= self.EXIT_ZONE_RADIUS:
            return True

        queue = getattr(port, "queue", None)
        if queue is None:
            return False
        for slot_point in list(getattr(queue, "slots", []))[: self.FRONT_ZONE_DEPTH]:
            if slot_point.distance(position) <= self.EXIT_ZONE_RADIUS:
                return True
        return False

    def _is_near_queue_corridor(self, position, port):
        if position is None or port is None:
            return False

        operation_zone = getattr(port, "operation_zone", None)
        if operation_zone is not None and operation_zone.distance(position) <= 1.1:
            return True

        queue = getattr(port, "queue", None)
        if queue is None:
            return False
        for slot_point in list(getattr(queue, "slots", [])):
            if slot_point.distance(position) <= 1.1:
                return True
        return False

    def _is_same_port_queue_yielder(self, observed_state, port):
        if port is None:
            return False
        if self._base_state_name(observed_state) != "QUEUING":
            return False
        if self._target_port_for_state(observed_state) != port:
            return False
        return not self._occupies_front_zone_footprint(observed_state, port)

    def _exit_owner_blocks_front_zone(self, agent_state, port):
        return self._occupies_front_zone_footprint(agent_state, port)

    def _front_zone_node_footprint(self, port, gridmap=None, queue_nodes=None):
        if port is None:
            return set()
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return set()
        queue_nodes = queue_nodes or self._queue_nodes_for_port(port, gridmap)
        front_nodes = self._front_zone_nodes(queue_nodes)
        footprint = set(front_nodes) | self._front_zone_approach_nodes(port, gridmap, queue_nodes)
        for node in front_nodes:
            for neighbor in gridmap.neighbors(node):
                footprint.add(neighbor)
        return footprint

    def _operation_zone_node_footprint(self, port, gridmap=None, queue_nodes=None):
        if port is None:
            return set()
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return set()
        queue_nodes = queue_nodes or self._queue_nodes_for_port(port, gridmap)
        if not queue_nodes:
            return set()

        operation_node = queue_nodes[0]
        queue_side_node = queue_nodes[1] if len(queue_nodes) > 1 else None
        footprint = {operation_node}
        footprint.update(self._front_zone_approach_nodes(port, gridmap, queue_nodes))
        for neighbor in gridmap.neighbors(operation_node):
            # queue-side slot 仍交给 front-zone 规则管理，这里只把 owner 本体周边的贴身区域硬封住。
            if queue_side_node is not None and neighbor == queue_side_node:
                continue
            footprint.add(neighbor)
        return footprint

    def _active_operation_owner_footprint(self, sensor_observation, port, gridmap=None, queue_nodes=None):
        if port is None:
            return set()
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return set()
        queue_nodes = queue_nodes or self._queue_nodes_for_port(port, gridmap)
        footprint = self._operation_zone_node_footprint(port, gridmap, queue_nodes)
        if not footprint:
            return footprint

        queue_side_node = queue_nodes[1] if len(queue_nodes) > 1 else None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if not self._is_active_operation_owner(observed_state, port):
                continue
            position = getattr(observed_state, "position", None)
            if position is None:
                continue
            owner_node = agent_to_gridmap(position, gridmap)
            footprint.add(owner_node)
            for neighbor in gridmap.neighbors(owner_node):
                if queue_side_node is not None and neighbor == queue_side_node:
                    continue
                footprint.add(neighbor)
        return footprint

    def _is_retreating_from_active_operation_owner(self, position, sequence_of_poses, blocker_state):
        target_port = self._target_port()
        if target_port is None:
            return False
        if not self._is_active_operation_owner(blocker_state, target_port):
            return False
        if self._base_state_name(self.agent) not in {"CRUISE", "PREQUEUE"}:
            return False

        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return False
        next_waypoints = list(sequence_of_poses)[:2]
        if not next_waypoints:
            return False

        current_distance = position.distance(blocker_position)
        next_distance = next_waypoints[0].distance(blocker_position)
        if next_distance <= current_distance + 1e-3:
            return False

        operation_zone = getattr(target_port, "operation_zone", None)
        if operation_zone is None:
            return True
        return next_waypoints[0].distance(operation_zone) > position.distance(operation_zone) + 1e-3

    def _is_retreating_from_source_port_owner(self, position, sequence_of_poses, blocker_state):
        if not self._is_exiting_port(self.agent):
            return False
        source_port = self._source_port_for_state(self.agent)
        if source_port is None:
            return False
        if not self._is_active_operation_owner(blocker_state, source_port):
            return False

        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return False
        next_waypoints = list(sequence_of_poses)[:2]
        if not next_waypoints:
            return False

        current_distance = position.distance(blocker_position)
        next_distance = next_waypoints[0].distance(blocker_position)
        if next_distance <= current_distance + 1e-3:
            return False

        operation_zone = getattr(source_port, "operation_zone", None)
        if operation_zone is None:
            return True
        return next_waypoints[0].distance(operation_zone) > position.distance(operation_zone) + 1e-3

    def _occupies_front_zone_footprint(self, agent_state, port, gridmap=None):
        if port is None:
            return False
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return False
        footprint = self._front_zone_node_footprint(port, gridmap)
        if not footprint:
            return False

        position = getattr(agent_state, "position", None)
        if position is not None and agent_to_gridmap(position, gridmap) in footprint:
            return True
        for pose in list(getattr(agent_state, "sequence_of_poses", []) or [])[:2]:
            if agent_to_gridmap(pose, gridmap) in footprint:
                return True
        return False

    def _blocker_still_occupies_port_corridor(self, position, sequence_of_poses, blocker_state):
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return False
        if self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position):
            return True
        for port in self._all_ports():
            if self._occupies_front_zone_footprint(blocker_state, port):
                return True
            if self._is_near_port_front_zone(blocker_position, port):
                return True
        return False

    def _stopping_should_not_block_self(self, observed_state):
        if self._base_state_name(self.agent) in {"CRUISE", "PREQUEUE", "QUEUING"}:
            position = getattr(observed_state, "position", None)
            for port in self._all_ports():
                if self._occupies_front_zone_footprint(observed_state, port) or self._is_near_port_front_zone(position, port):
                    # v4 的口前区不能把“已经为我停下”的车直接视为可穿过，
                    # 否则 queue head 会在对方身体尚未离开 mouth 时被错误放行。
                    return False
        return super(LayeredAStarQueueAware, self)._stopping_should_not_block_self(observed_state)

    def _yielding_blocker_still_controls_exit_corridor(self, observed_state, port, position=None, sequence_of_poses=None):
        if port is None:
            return False

        blocker_position = getattr(observed_state, "position", None)
        if blocker_position is None:
            return False

        # 只有在 source-port mouth/front-zone 真正清空后，exiting owner 才能把让行车视为“已释放”。
        if self._occupies_front_zone_footprint(observed_state, port):
            return True
        if self._is_near_port_front_zone(blocker_position, port):
            return True

        if position is None:
            position = getattr(self.agent, "position", None)
        if sequence_of_poses is None:
            sequence_of_poses = getattr(self.agent, "sequence_of_poses", None)
        if position is None or not sequence_of_poses:
            return False
        return self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position)

    def _can_ignore_yielding_blocker(self, observed_state, port, position=None, sequence_of_poses=None):
        if not self._is_same_port_yielding_to_self(observed_state, port):
            return False
        if self._base_state_name(observed_state) in {"LOADING", "UNLOADING"}:
            return False
        if self._yielding_blocker_still_controls_exit_corridor(
            observed_state,
            port,
            position,
            sequence_of_poses,
        ):
            return False
        return True
