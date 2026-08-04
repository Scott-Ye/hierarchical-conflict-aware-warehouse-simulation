from math import pi, sqrt
import json
import urllib.request

from geometry import Point, Vector, compute_direction, edge_edge_shortest_square_distance
from global_planners.sample_global_planner import PriorityQueue
from global_planners.layered_astar_planner import LayeredAStar
from representation.float_to_grid import agent_to_gridmap
from task_managers.task_manager import TaskType


class LayeredAStarBaselineTrafficAware(LayeredAStar):
    """Independent baseline branch that avoids inheriting v1-v4 queue logic."""
    _DEBUG_ENDPOINT_DISABLED = object()
    _debug_endpoint_cache = {}
    PASSIVE_STOPPING_REASONS = {
        "baseline_zone_wait",
        "baseline_port_admission_wait",
        "baseline_same_port_queue_hold",
        "baseline_same_target_approach_hold",
        "baseline_exit_same_source_hold",
        "baseline_non_port_corridor_hold",
    }

    SENSOR_RANGE = 6.0
    SENSOR_ANGLE = pi
    PATH_WINDOW = 6.0
    LOOKAHEAD_POINTS = 4
    REPLAN_DISTANCE = 1.35
    CORRIDOR_DISTANCE = 1.45
    STOP_DISTANCE = 1.35
    MOVING_BLOCKER_STOP_DISTANCE = 1.8
    KEYZONE_APPROACH_DISTANCE = 3.1
    PORT_ACTIVE_DISTANCE = 2.8
    PORT_ADMISSION_DISTANCE = 3.0
    CONFLICT_ENTRY_DISTANCE = 2.15
    CONFLICT_INNER_DISTANCE = 0.95
    SAME_PORT_SAFETY_HOLD_DISTANCE = 3.9
    SAME_PORT_OWNER_BACKOFF_DISTANCE = 1.2
    SAME_PORT_QUEUE_CHAIN_HOLD_DISTANCE = 1.8
    SAME_SOURCE_SAFETY_HOLD_DISTANCE = 3.75
    SAME_SOURCE_BACKOFF_DISTANCE = 1.1
    EARLY_CORRIDOR_REPLAN_DISTANCE = 3.75
    YIELD_DISTANCE = 1.15
    YIELD_SEGMENT_DISTANCE = 0.9
    EXIT_ZONE_RADIUS = 3.0
    FRONT_ZONE_DEPTH = 2
    QUEUE_SLOT_EPSILON = 1e-3
    RESERVATION_WAYPOINTS = 5
    RESERVATION_WINDOW_DISTANCE = 6.5
    RESERVATION_DECAY = 0.84
    RESERVATION_NODE_INFLATION = 18.0
    RESERVATION_EDGE_INFLATION = 22.0
    RESERVATION_CONFLICT_WINDOW = 4
    PREDICTIVE_REPLAN_DISTANCE = 4.8
    PREDICTIVE_CORRIDOR_HOLD_DISTANCE = 2.8
    NON_PORT_YIELD_BACKOFF_DISTANCE = 1.9

    BASE_NODE_INFLATION = 12.0
    BASE_EDGE_INFLATION = 7.0
    OPERATION_NODE_INFLATION = 34.0
    OPERATION_EDGE_INFLATION = 22.0
    FRONT_OWNER_NODE_INFLATION = 52.0
    FRONT_OWNER_EDGE_INFLATION = 30.0
    FORCED_BYPASS_NODE_INFLATION = 96.0
    FORCED_BYPASS_EDGE_INFLATION = 64.0
    DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-no-double-stop.env"
    DEBUG_FALLBACK_URL = "http://127.0.0.1:7780/event"
    DEBUG_SESSION_ID = "global-no-double-stop"
    MULTI_STOP_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\multi-stop-chain.env"
    MULTI_STOP_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
    MULTI_STOP_DEBUG_SESSION_ID = "multi-stop-chain"
    MULTI_STOP_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-multi-stop-chain.ndjson"
    GUI_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\gui-global-slowdown.env"
    GUI_DEBUG_FALLBACK_URL = "http://127.0.0.1:7779/event"
    GUI_DEBUG_SESSION_ID = "gui-global-slowdown"
    COLLISION_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-traffic-collision.env"
    COLLISION_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
    COLLISION_DEBUG_SESSION_ID = "baseline-traffic-collision"
    COLLISION_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-traffic-collision.ndjson"
    SAME_PORT_EXIT_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\same-port-exit.env"
    SAME_PORT_EXIT_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
    SAME_PORT_EXIT_DEBUG_SESSION_ID = "same-port-exit"
    SAME_PORT_EXIT_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-same-port-exit.ndjson"
    MIDMAP_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-midmap-collision.env"
    MIDMAP_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
    MIDMAP_DEBUG_SESSION_ID = "baseline-midmap-collision"
    MIDMAP_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-midmap-collision.ndjson"
    PORT_SPEED_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\port-speed-replan.env"
    PORT_SPEED_DEBUG_FALLBACK_URL = "http://127.0.0.1:7781/event"
    PORT_SPEED_DEBUG_SESSION_ID = "port-speed-replan"

    def __init__(self, agent):
        super(LayeredAStarBaselineTrafficAware, self).__init__(agent)
        self.active_yield_blockers = {}
        self._forced_bypass_blocker_id = None
        self._forbidden_transitions = set()

    def begin_forced_bypass(self, blocker_id):
        self._forced_bypass_blocker_id = blocker_id

    def end_forced_bypass(self):
        self._forced_bypass_blocker_id = None

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

    def _resolve_debug_endpoint(self, cache_key, env_path, fallback_url, fallback_session_id):
        cached = self._debug_endpoint_cache.get(cache_key, None)
        if cached is self._DEBUG_ENDPOINT_DISABLED:
            return None
        if cached is not None:
            return cached
        try:
            debug_url, session_id = fallback_url, fallback_session_id
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
            if session_id in {self.COLLISION_DEBUG_SESSION_ID, self.SAME_PORT_EXIT_DEBUG_SESSION_ID, self.PORT_SPEED_DEBUG_SESSION_ID}:
                debug_enabled = True
            endpoint = (debug_url, session_id) if debug_enabled else self._DEBUG_ENDPOINT_DISABLED
        except Exception:
            endpoint = self._DEBUG_ENDPOINT_DISABLED
        self._debug_endpoint_cache[cache_key] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_DISABLED:
            return None
        return endpoint

    def _post_debug_event(self, cache_key, env_path, fallback_url, fallback_session_id, hypothesis_id, location, message, data):
        endpoint = self._resolve_debug_endpoint(cache_key, env_path, fallback_url, fallback_session_id)
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

    def _debug_event(self, hypothesis_id, location, message, data):
        self._post_debug_event(
            "baseline_planner",
            self.DEBUG_ENV_PATH,
            self.DEBUG_FALLBACK_URL,
            self.DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_gui_event(self, hypothesis_id, location, message, data):
        self._post_debug_event(
            "baseline_planner_gui",
            self.GUI_DEBUG_ENV_PATH,
            self.GUI_DEBUG_FALLBACK_URL,
            self.GUI_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_port_speed_event(self, hypothesis_id, location, message, data):
        self._post_debug_event(
            "baseline_planner_port_speed",
            self.PORT_SPEED_DEBUG_ENV_PATH,
            self.PORT_SPEED_DEBUG_FALLBACK_URL,
            self.PORT_SPEED_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_multi_stop_event(self, hypothesis_id, location, message, data):
        try:
            with open(self.MULTI_STOP_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": self.MULTI_STOP_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._post_debug_event(
            "baseline_planner_multi_stop",
            self.MULTI_STOP_DEBUG_ENV_PATH,
            self.MULTI_STOP_DEBUG_FALLBACK_URL,
            self.MULTI_STOP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_collision_event(self, hypothesis_id, location, message, data):
        try:
            with open(self.COLLISION_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": self.COLLISION_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._post_debug_event(
            "baseline_planner_collision",
            self.COLLISION_DEBUG_ENV_PATH,
            self.COLLISION_DEBUG_FALLBACK_URL,
            self.COLLISION_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )
        try:
            with open(self.MIDMAP_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": self.MIDMAP_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._post_debug_event(
            "baseline_planner_midmap_collision",
            self.MIDMAP_DEBUG_ENV_PATH,
            self.MIDMAP_DEBUG_FALLBACK_URL,
            self.MIDMAP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_same_port_exit_event(self, hypothesis_id, location, message, data):
        try:
            with open(self.SAME_PORT_EXIT_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": self.SAME_PORT_EXIT_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._post_debug_event(
            "baseline_planner_same_port_exit",
            self.SAME_PORT_EXIT_DEBUG_ENV_PATH,
            self.SAME_PORT_EXIT_DEBUG_FALLBACK_URL,
            self.SAME_PORT_EXIT_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _conflict_debug_payload(self, blocker_state, port=None):
        own_target_port = self._target_port_for_state(self.agent)
        blocker_target_port = self._target_port_for_state(blocker_state)
        active_port = port or self._active_port_for_state(self.agent) or self._active_port_for_state(blocker_state)
        return {
            "own_state": self._observed_state_name(self.agent),
            "own_base_state": self._base_state_name(self.agent),
            "own_target_port_id": getattr(own_target_port, "id", None),
            "own_target_port_type": getattr(own_target_port, "port_type", None),
            "own_queue_progress": self._queue_progress_score(self.agent),
            "blocker_state_name": self._observed_state_name(blocker_state),
            "blocker_base_state": self._base_state_name(blocker_state),
            "blocker_target_port_id": getattr(blocker_target_port, "id", None),
            "blocker_target_port_type": getattr(blocker_target_port, "port_type", None),
            "blocker_queue_progress": self._queue_progress_score(blocker_state),
            "debug_active_port_id": getattr(active_port, "id", None),
            "debug_active_port_type": getattr(active_port, "port_type", None),
            "same_task_target_port": self._ports_match(own_target_port, blocker_target_port),
        }

    def _should_trace_same_port_exit_pair(self, observed_state=None):
        own_id = getattr(self.agent, "id", None)
        other_id = getattr(observed_state, "id", None) if observed_state is not None else None
        if observed_state is None:
            return own_id == 5
        return own_id == 5 and other_id == 6

    def get_dynamic_layer(self, gridmap, sensor_observation, inflation=None):
        dynamic_layer = {}
        self._forbidden_transitions = set()
        current_position = getattr(self.agent, "position", None)
        sequence_of_poses = list(getattr(self.agent, "sequence_of_poses", []) or [])
        target_port = self._target_port_for_state(self.agent)
        if current_position is None:
            return dynamic_layer

        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            blocker_position = getattr(observed_state, "position", None)
            predictive_profile = self._predictive_conflict_profile(
                current_position,
                sequence_of_poses,
                observed_agent,
                gridmap,
            )
            if blocker_id is None or blocker_id == getattr(self.agent, "id", None):
                continue
            if blocker_position is None:
                continue

            if blocker_id == self._forced_bypass_blocker_id:
                self._apply_blocker_barrier(
                    dynamic_layer,
                    gridmap,
                    blocker_position,
                    node_inflation=self.FORCED_BYPASS_NODE_INFLATION,
                    edge_inflation=self.FORCED_BYPASS_EDGE_INFLATION,
                )
                continue

            if not self._should_consider_for_dynamic_layer(
                current_position,
                sequence_of_poses,
                observed_state,
                observed_agent=observed_agent,
                gridmap=gridmap,
                predictive_profile=predictive_profile,
            ):
                continue

            node_inflation = self.BASE_NODE_INFLATION
            edge_inflation = self.BASE_EDGE_INFLATION
            if self._is_operation_owner(observed_state) or self._is_absolute_stopping_blocker(
                current_position,
                sequence_of_poses,
                observed_state,
            ):
                node_inflation = self.OPERATION_NODE_INFLATION
                edge_inflation = self.OPERATION_EDGE_INFLATION
                # #region debug-point B:keyzone-dynamic-layer
                self._debug_event(
                    "B",
                    "layered_a_star_baseline_traffic_aware_planner.py:get_dynamic_layer",
                    "[DEBUG] baseline planner injected key-zone blocker into dynamic layer",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "same_target_port": self._is_same_target_port_queue_or_owner(observed_state),
                        "shared_active_port": self._shares_active_port(self.agent, observed_state),
                        "corridor_conflict": self._blocker_on_current_corridor(
                            current_position,
                            sequence_of_poses,
                            blocker_position,
                        ),
                    },
                )
                # #endregion
            self._apply_blocker_barrier(
                dynamic_layer,
                gridmap,
                blocker_position,
                node_inflation=node_inflation,
                edge_inflation=edge_inflation,
            )
            if predictive_profile["active"]:
                reservation_nodes = predictive_profile["observed_nodes"]
                node_scale = 1.0 + min(0.6, 0.15 * predictive_profile["shared_nodes"])
                edge_scale = 1.0 + 0.35 * predictive_profile["aligned_edges"] + 0.6 * predictive_profile["reverse_edges"]
                if self._is_same_target_port_queue_or_owner(observed_state) or self._shares_active_port(self.agent, observed_state):
                    edge_scale += 0.35
                self._apply_reservation_window(
                    dynamic_layer,
                    gridmap,
                    reservation_nodes,
                    node_scale=node_scale,
                    edge_scale=edge_scale,
                )
        for port in self._all_ports():
            if not self._port_is_preplan_relevant(port, current_position, sequence_of_poses):
                continue
            queue_nodes = self._queue_nodes_for_port(port, gridmap)
            if not queue_nodes:
                continue
            if self._port_has_front_owner(sensor_observation, port):
                self._apply_front_owner_entry_gate(
                    dynamic_layer,
                    gridmap,
                    sensor_observation,
                    port,
                    queue_nodes,
                )
            if self._port_has_active_operation_owner(sensor_observation, port):
                protected_nodes = self._active_operation_owner_footprint(
                    sensor_observation,
                    port,
                    gridmap,
                    queue_nodes,
                )
                self._mark_protected_node_entries_forbidden(gridmap, protected_nodes)
                for node in protected_nodes:
                    self._reserve_node_entries(dynamic_layer, gridmap, node, self.OPERATION_NODE_INFLATION * 0.7)
        return dynamic_layer

    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold=1):
        if not sequence_of_poses:
            return False

        visible_waypoints = []
        remaining_distance = current_position.distance(sequence_of_poses[-1])
        detected_obstacles = sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        )

        for pose in sequence_of_poses:
            if (
                current_position.distance(pose) < self.PATH_WINDOW
                and sequence_of_poses[-1].distance(pose) < remaining_distance
            ):
                visible_waypoints.append(pose)
        visible_waypoints = visible_waypoints[: self.LOOKAHEAD_POINTS]
        nearest_blocker_distance = None
        nearest_blocker_id = None
        nearest_blocker_state = None

        for observed_agent in detected_obstacles:
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_position = getattr(observed_state, "position", None)
            if blocker_position is None:
                continue
            blocker_distance = current_position.distance(blocker_position)
            predictive_profile = self._predictive_conflict_profile(
                current_position,
                sequence_of_poses,
                observed_agent,
                gridmap,
            )
            corridor_conflict = self._blocker_on_current_corridor(
                current_position,
                sequence_of_poses,
                blocker_position,
            )
            if nearest_blocker_distance is None or blocker_distance < nearest_blocker_distance:
                nearest_blocker_distance = blocker_distance
                nearest_blocker_id = getattr(observed_state, "id", None)
                nearest_blocker_state = observed_state
            if self._requires_early_corridor_replan(
                current_position,
                sequence_of_poses,
                observed_state,
                blocker_distance,
                corridor_conflict,
            ):
                self._debug_event(
                    "A",
                    "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                    "[DEBUG] baseline planner requested early corridor safety replan",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": getattr(observed_state, "id", None),
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "blocker_distance": round(blocker_distance, 3),
                        "corridor_conflict": corridor_conflict,
                        "same_target_port": self._is_same_target_port_queue_or_owner(observed_state),
                        "shared_active_port": self._shares_active_port(self.agent, observed_state),
                    },
                )
                return True
            if self._requires_predictive_replan(
                current_position,
                sequence_of_poses,
                observed_state,
                blocker_distance,
                predictive_profile,
            ):
                self._debug_event(
                    "A",
                    "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                    "[DEBUG] baseline planner requested predictive corridor replan from reservation overlap",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": getattr(observed_state, "id", None),
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "blocker_distance": round(blocker_distance, 3),
                        "shared_nodes": predictive_profile["shared_nodes"],
                        "aligned_edges": predictive_profile["aligned_edges"],
                        "reverse_edges": predictive_profile["reverse_edges"],
                        "own_nodes": list(predictive_profile["own_nodes"]),
                        "observed_nodes": list(predictive_profile["observed_nodes"]),
                    },
                )
                return True
            if self._should_consider_for_dynamic_layer(
                current_position,
                sequence_of_poses,
                observed_state,
                observed_agent=observed_agent,
                gridmap=gridmap,
                predictive_profile=predictive_profile,
            ):
                self._debug_gui_event(
                    "A",
                    "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                    "[DEBUG] GUI slowdown session triggered dynamic replan",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": getattr(observed_state, "id", None),
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "same_target_port": self._is_same_target_port_queue_or_owner(observed_state),
                        "shared_active_port": self._shares_active_port(self.agent, observed_state),
                        "corridor_conflict": corridor_conflict,
                        "own_port_relevant": self._port_relevance_for_self(
                            self._target_port_for_state(self.agent),
                            current_position,
                            sequence_of_poses,
                        ),
                        "distance": round(current_position.distance(blocker_position), 3),
                        "path_preview": self._point_preview(sequence_of_poses),
                    },
                )
                # #region debug-point A:observe-path-replan
                self._debug_event(
                    "A",
                    "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                    "[DEBUG] baseline planner requested replan before entering potential conflict zone",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": getattr(observed_state, "id", None),
                        "blocker_state": self._observed_state_name(observed_state),
                        "same_target_port": self._is_same_target_port_queue_or_owner(observed_state),
                        "shared_active_port": self._shares_active_port(self.agent, observed_state),
                        "corridor_conflict": corridor_conflict,
                    },
                )
                # #endregion
                return True
            for waypoint in visible_waypoints:
                if self._is_same_target_port_queue_or_owner(observed_state):
                    target_port = self._target_port_for_state(self.agent)
                    if (
                        target_port is None
                        or not self._port_relevance_for_self(target_port, current_position, sequence_of_poses)
                        or not self._occupies_front_zone_footprint(observed_state, target_port)
                        or current_position.distance(blocker_position) > self.KEYZONE_APPROACH_DISTANCE
                    ):
                        continue
                if waypoint.distance(blocker_position) <= self.REPLAN_DISTANCE:
                    self._debug_gui_event(
                        "A",
                        "layered_a_star_baseline_traffic_aware_planner.py:observe_path:waypoint",
                        "[DEBUG] GUI slowdown session triggered waypoint replan",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": getattr(observed_state, "id", None),
                            "blocker_state": self._observed_state_name(observed_state),
                            "distance": round(current_position.distance(blocker_position), 3),
                            "waypoint": [round(waypoint.x, 3), round(waypoint.y, 3)],
                            "path_preview": self._point_preview(sequence_of_poses),
                        },
                    )
                    # #region debug-point A:observe-waypoint-replan
                    self._debug_event(
                        "A",
                        "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                        "[DEBUG] baseline planner requested replan from near-future waypoint conflict",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": getattr(observed_state, "id", None),
                            "blocker_state": self._observed_state_name(observed_state),
                            "same_target_port": self._is_same_target_port_queue_or_owner(observed_state),
                            "shared_active_port": self._shares_active_port(self.agent, observed_state),
                            "waypoint": [round(waypoint.x, 3), round(waypoint.y, 3)],
                        },
                    )
                    # #endregion
                    return True
        if nearest_blocker_distance is not None and nearest_blocker_distance <= self.CONFLICT_ENTRY_DISTANCE + 0.6:
            # #region debug-point C:baseline-near-blocker-no-replan
            self._debug_collision_event(
                "C",
                "layered_a_star_baseline_traffic_aware_planner.py:observe_path",
                "[DEBUG] baseline planner kept current path despite nearby blocker",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "nearest_blocker_id": nearest_blocker_id,
                    "nearest_blocker_state": self._observed_state_name(nearest_blocker_state) if nearest_blocker_state is not None else None,
                    "nearest_blocker_distance": round(nearest_blocker_distance, 3),
                    "path_preview": self._point_preview(sequence_of_poses),
                },
            )
            # #endregion
        return False

    def compute_avoidance_response(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return None

        if own_state_name in {"CRUISE", "PREQUEUE"}:
            same_target_port_response = self._same_target_port_approach_response(
                position,
                sensor_observation,
                sequence_of_poses,
            )
            if same_target_port_response is not None:
                return same_target_port_response

        port_admission_response = self._port_admission_response(position, sensor_observation, sequence_of_poses)
        if port_admission_response is not None:
            return port_admission_response

        candidate = None
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            if blocker_id is None or blocker_id == getattr(self.agent, "id", None):
                continue
            conflict_info = self._conflict_info(position, sequence_of_poses, observed_state, observed_agent)
            if not conflict_info["conflicts"]:
                continue
            blocker_distance = conflict_info["blocker_distance"]
            score = (
                conflict_info["anchor_distance"],
                blocker_distance,
                blocker_id,
            )
            if candidate is None or score < candidate["score"]:
                candidate = {
                    "score": score,
                    "blocker_distance": blocker_distance,
                    "blocker_id": blocker_id,
                    "blocker_state": observed_state,
                    "blocker_agent": observed_agent,
                    "info": conflict_info,
                    "anchor_distance": conflict_info["anchor_distance"],
                }

        if candidate is None:
            return None

        blocker_distance = candidate["blocker_distance"]
        blocker_id = candidate["blocker_id"]
        blocker_state = candidate["blocker_state"]
        conflict_info = candidate["info"]
        should_yield = self._should_yield_to(blocker_state)
        allow_stop_fallback = should_yield
        non_port_moving_pair = self._is_non_port_moving_pair(own_state_name, blocker_state, conflict_info)
        same_port_role = self._same_target_port_conflict_role(blocker_state, conflict_info)
        same_port_predecessor = same_port_role == "following"
        same_port_leader = same_port_role == "leading"
        own_queue_progress = self._queue_progress_score(self.agent)
        blocker_queue_progress = self._queue_progress_score(blocker_state)
        blocker_base_state = self._base_state_name(blocker_state)
        early_same_port_hold = self._requires_same_port_safety_hold(
            blocker_state,
            conflict_info,
            blocker_distance,
            same_port_predecessor,
            own_queue_progress,
            blocker_queue_progress,
        )
        # #region debug-point A:same-port-candidate-selection
        if self._should_trace_same_port_exit_pair(blocker_state):
            self._debug_same_port_exit_event(
                "A",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] same-port exit pair selected as avoidance candidate",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "own_state": own_state_name,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_base_state": blocker_base_state,
                    "blocker_distance": round(blocker_distance, 3),
                    "same_port_role": same_port_role,
                    "should_yield": should_yield,
                    "early_same_port_hold": early_same_port_hold,
                    "same_target_port": conflict_info.get("same_target_port"),
                    "shared_active_port": conflict_info.get("shared_active_port"),
                    "operation_owner": conflict_info.get("operation_owner"),
                    "corridor_conflict": conflict_info.get("corridor_conflict"),
                    "predictive_conflict": conflict_info.get("predictive_conflict"),
                    "own_front_port_id": getattr(self._front_zone_port_for_state(self.agent), "id", None),
                    "blocker_front_port_id": getattr(self._front_zone_port_for_state(blocker_state), "id", None),
                    "blocker_is_exiting": self._is_exiting_port(blocker_state),
                    "blocker_source_port_id": getattr(self._source_port_for_state(blocker_state), "id", None),
                },
            )
        elif self._should_trace_same_port_exit_pair():
            self._debug_same_port_exit_event(
                "A",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] same-port exit follower selected a different avoidance candidate",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "own_state": own_state_name,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_base_state": blocker_base_state,
                    "blocker_distance": round(blocker_distance, 3),
                    "same_port_role": same_port_role,
                    "should_yield": should_yield,
                    "early_same_port_hold": early_same_port_hold,
                    "same_target_port": conflict_info.get("same_target_port"),
                    "shared_active_port": conflict_info.get("shared_active_port"),
                    "operation_owner": conflict_info.get("operation_owner"),
                    "corridor_conflict": conflict_info.get("corridor_conflict"),
                    "predictive_conflict": conflict_info.get("predictive_conflict"),
                },
            )
        # #endregion
        # #region debug-point A:baseline-collision-candidate
        self._debug_collision_event(
            "A",
            "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
            "[DEBUG] baseline planner selected conflict candidate",
            {
                "agent_id": getattr(self.agent, "id", None),
                "agent_state": own_state_name,
                "blocker_id": blocker_id,
                "blocker_state": self._observed_state_name(blocker_state),
                "blocker_base_state": blocker_base_state,
                "blocker_distance": round(blocker_distance, 3),
                "anchor_distance": round(conflict_info["anchor_distance"], 3),
                "blocker_anchor_distance": round(conflict_info["blocker_anchor_distance"], 3),
                "should_yield": should_yield,
                "non_port_moving_pair": non_port_moving_pair,
                "same_port_role": same_port_role,
                "own_queue_progress": own_queue_progress,
                "blocker_queue_progress": blocker_queue_progress,
                "corridor_conflict": conflict_info.get("corridor_conflict"),
                "future_conflict": conflict_info.get("future_conflict"),
                **self._conflict_debug_payload(blocker_state),
            },
        )
        # #endregion
        if non_port_moving_pair:
            allow_stop_fallback = False
        if getattr(blocker_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None):
            allow_stop_fallback = False
        exiting_source_port = self._source_port_for_state(self.agent)
        blocker_exiting_source_port = self._source_port_for_state(blocker_state)
        exiting_owner_keep_path = (
            self._is_exiting_port(self.agent)
            and exiting_source_port is not None
            and blocker_base_state in {"CRUISE", "PREQUEUE", "QUEUING"}
            and self._ports_match(self._target_port_for_state(blocker_state), exiting_source_port)
        )
        entering_same_source_hold = (
            own_state_name in {"CRUISE", "PREQUEUE", "QUEUING"}
            and self._is_exiting_port(blocker_state)
            and blocker_exiting_source_port is not None
            and self._ports_match(self._target_port_for_state(self.agent), blocker_exiting_source_port)
        )
        early_same_source_hold = self._requires_same_source_safety_hold(
            conflict_info,
            blocker_distance,
            entering_same_source_hold,
        )
        same_port_hold_distance = self.CONFLICT_ENTRY_DISTANCE + 0.35
        if (
            same_port_predecessor
            and own_queue_progress > 0
            and blocker_queue_progress > 0
            and blocker_base_state == "QUEUING"
        ):
            # 已入队 follower 不应在远距离就被 predecessor 长时间锁死；
            # 这类情形交给近距离安全停和 front-slot hold 接管即可。
            same_port_hold_distance = self.SAME_PORT_QUEUE_CHAIN_HOLD_DISTANCE
        if same_port_predecessor and (
            early_same_port_hold or blocker_distance <= same_port_hold_distance
        ):
            target_port = self._target_port_for_state(self.agent)
            if (
                conflict_info["operation_owner"]
                and blocker_distance <= self.SAME_PORT_OWNER_BACKOFF_DISTANCE
            ):
                backoff_command = self._port_admission_backoff_command(
                    position,
                    sequence_of_poses,
                    blocker_state,
                    target_port,
                )
                if backoff_command is not None:
                    self._debug_event(
                        "D",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner selected backoff for same-port follower near operation owner",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "blocker_distance": round(blocker_distance, 3),
                            "same_port_role": same_port_role,
                            "owner_backoff_distance": self.SAME_PORT_OWNER_BACKOFF_DISTANCE,
                            **self._conflict_debug_payload(blocker_state, target_port),
                        },
                    )
                    self._debug_collision_event(
                        "B",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose same-port owner backoff",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "decision": "baseline_same_port_queue_backoff",
                            "blocker_distance": round(blocker_distance, 3),
                            "same_port_role": same_port_role,
                        },
                    )
                    return {
                        "command": backoff_command,
                        "reason": "baseline_same_port_queue_backoff",
                        "blocker_id": blocker_id,
                    }
            if own_queue_progress <= 0 and blocker_base_state in {"QUEUING", "LOADING", "UNLOADING"}:
                if (
                    blocker_distance > same_port_hold_distance
                    and not early_same_port_hold
                ):
                    return None
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner held trailing same-port queue agent",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": round(blocker_distance, 3),
                    "own_queue_progress": own_queue_progress,
                    "blocker_queue_progress": blocker_queue_progress,
                    "early_same_port_hold": early_same_port_hold,
                    "same_port_hold_distance": same_port_hold_distance,
                    **self._conflict_debug_payload(blocker_state),
                },
            )
            # #region debug-point B:baseline-collision-decision
            self._debug_collision_event(
                "B",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner chose same-port queue hold",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "decision": "baseline_same_port_queue_hold",
                    "blocker_distance": round(blocker_distance, 3),
                    "early_same_port_hold": early_same_port_hold,
                    "same_port_hold_distance": same_port_hold_distance,
                },
            )
            # #endregion
            # #region debug-point E:same-port-hold
            self._debug_multi_stop_event(
                "E",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] multi-stop planner chose same-port queue hold",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_base_state": blocker_base_state,
                    "blocker_distance": round(blocker_distance, 3),
                    "same_port_role": same_port_role,
                    "early_same_port_hold": early_same_port_hold,
                    "own_queue_progress": own_queue_progress,
                    "blocker_queue_progress": blocker_queue_progress,
                    "same_port_hold_distance": same_port_hold_distance,
                },
            )
            # #endregion
            return {
                "command": (0.0, 0.0),
                "reason": "baseline_same_port_queue_hold",
                "blocker_id": blocker_id,
            }
        if same_port_predecessor:
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner kept same-port queue follower on current path",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": round(blocker_distance, 3),
                    "own_queue_progress": own_queue_progress,
                    "blocker_queue_progress": blocker_queue_progress,
                    **self._conflict_debug_payload(blocker_state),
                },
            )
            # #region debug-point B:baseline-collision-decision
            self._debug_collision_event(
                "B",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner left same-port leader/follower on current path",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "decision": "none_same_port_queue_keep_path",
                    "blocker_distance": round(blocker_distance, 3),
                    "same_port_role": same_port_role,
                },
            )
            # #endregion
            return None
        if entering_same_source_hold and (
            early_same_source_hold or blocker_distance <= self.PORT_ADMISSION_DISTANCE + 0.6
        ):
            # same-source 进港侧若等到几乎贴身才退避，会和离港 owner 在执行层互相打架；
            # 这里把退避触发提到物理安全半径之外，尽量在接触前就开始拉开。
            if (
                blocker_distance <= self.SAME_SOURCE_BACKOFF_DISTANCE
                or (
                    blocker_base_state in {"CRUISE", "PREQUEUE", "QUEUING"}
                    and blocker_distance <= self.MOVING_BLOCKER_STOP_DISTANCE
                    and (
                        conflict_info.get("corridor_conflict")
                        or conflict_info.get("predictive_conflict")
                    )
                )
            ):
                backoff_command = self._same_source_backoff_command(
                    position, sequence_of_poses, blocker_state
                )
                if backoff_command is not None:
                    self._debug_event(
                        "D",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner selected backoff for entering same-source agent",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "blocker_distance": round(blocker_distance, 3),
                            "source_port_id": getattr(blocker_exiting_source_port, "id", None),
                            "source_port_type": getattr(blocker_exiting_source_port, "port_type", None),
                            "early_same_source_hold": early_same_source_hold,
                            **self._conflict_debug_payload(blocker_state, blocker_exiting_source_port),
                        },
                    )
                    # #region debug-point B:baseline-collision-decision
                    self._debug_collision_event(
                        "B",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose same-source backoff",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "decision": "baseline_exit_same_source_backoff",
                            "blocker_distance": round(blocker_distance, 3),
                            "early_same_source_hold": early_same_source_hold,
                        },
                    )
                    # #endregion
                    return {
                        "command": backoff_command,
                        "reason": "baseline_exit_same_source_backoff",
                        "blocker_id": blocker_id,
                    }
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner held entering agent for exiting same-source owner",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": round(blocker_distance, 3),
                    "source_port_id": getattr(blocker_exiting_source_port, "id", None),
                    "source_port_type": getattr(blocker_exiting_source_port, "port_type", None),
                    "early_same_source_hold": early_same_source_hold,
                    **self._conflict_debug_payload(blocker_state, blocker_exiting_source_port),
                },
            )
            # #region debug-point B:baseline-collision-decision
            self._debug_collision_event(
                "B",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner chose same-source hold",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "decision": "baseline_exit_same_source_hold",
                    "blocker_distance": round(blocker_distance, 3),
                    "early_same_source_hold": early_same_source_hold,
                },
            )
            # #endregion
            return {
                "command": (0.0, 0.0),
                "reason": "baseline_exit_same_source_hold",
                "blocker_id": blocker_id,
            }
        if same_port_leader:
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner kept same-port queue leader on current path",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": round(blocker_distance, 3),
                    "own_queue_progress": self._queue_progress_score(self.agent),
                    "blocker_queue_progress": self._queue_progress_score(blocker_state),
                    **self._conflict_debug_payload(blocker_state),
                },
            )
            return None
        if (
            exiting_owner_keep_path
            and blocker_base_state in {"QUEUING", "LOADING", "UNLOADING"}
            and blocker_distance <= self.MOVING_BLOCKER_STOP_DISTANCE
        ):
            backoff_command = self._same_source_backoff_command(
                position, sequence_of_poses, blocker_state
            )
            if backoff_command is not None:
                self._debug_event(
                    "D",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner selected emergency backoff for exiting owner near queued blocker",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "blocker_distance": round(blocker_distance, 3),
                        "source_port_id": getattr(exiting_source_port, "id", None),
                        "source_port_type": getattr(exiting_source_port, "port_type", None),
                        **self._conflict_debug_payload(blocker_state, exiting_source_port),
                    },
                )
                self._debug_collision_event(
                    "B",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner chose emergency backoff for exiting owner",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "decision": "baseline_exit_same_source_owner_backoff",
                        "blocker_distance": round(blocker_distance, 3),
                    },
                )
                return {
                    "command": backoff_command,
                    "reason": "baseline_exit_same_source_owner_backoff",
                    "blocker_id": blocker_id,
                }
        if (
            exiting_owner_keep_path
            and blocker_base_state not in {"QUEUING", "LOADING", "UNLOADING"}
            and blocker_distance <= max(self.MOVING_BLOCKER_STOP_DISTANCE, self.CONFLICT_ENTRY_DISTANCE + 0.7)
        ):
            bypass_command = self._direct_bypass_command(position, sequence_of_poses, blocker_state)
            if bypass_command is not None:
                self._debug_event(
                    "D",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner selected direct bypass for exiting same-source owner",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "blocker_distance": round(blocker_distance, 3),
                        "source_port_id": getattr(exiting_source_port, "id", None),
                        "source_port_type": getattr(exiting_source_port, "port_type", None),
                        **self._conflict_debug_payload(blocker_state, exiting_source_port),
                    },
                )
                # #region debug-point B:baseline-collision-decision
                self._debug_collision_event(
                    "B",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner chose exiting owner bypass",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "decision": "baseline_exit_same_source_bypass",
                        "blocker_distance": round(blocker_distance, 3),
                    },
                )
                # #endregion
                return {
                    "command": bypass_command,
                    "reason": "baseline_exit_same_source_bypass",
                    "blocker_id": blocker_id,
                }
        if exiting_owner_keep_path:
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner selected replan for exiting owner",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_distance": round(blocker_distance, 3),
                    "source_port_id": getattr(exiting_source_port, "id", None),
                    "source_port_type": getattr(exiting_source_port, "port_type", None),
                    **self._conflict_debug_payload(blocker_state, exiting_source_port),
                },
            )
            # #region debug-point B:baseline-collision-decision
            self._debug_collision_event(
                "B",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner chose exiting owner replan",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "decision": "exit_same_source_replan",
                    "blocker_distance": round(blocker_distance, 3),
                },
            )
            # #endregion
            return {
                "replan": True,
                "fallback_command": (0.0, 0.0),
                "reason": "exit_same_source_replan",
                "blocker_id": blocker_id,
                "stop_distance": self._stop_distance_for_state(blocker_state),
                "allow_stop_fallback": False,
                "suppress_stop_when_path_clear": True,
                "force_blocker_barrier": True,
            }
        if self._should_wait_before_entry(conflict_info, allow_stop_fallback):
            # #region debug-point D:baseline-zone-wait-selected
            self._debug_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner held non-owner outside conflict entry",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "allow_stop_fallback": allow_stop_fallback,
                    "anchor_distance": round(conflict_info["anchor_distance"], 3),
                    "blocker_anchor_distance": round(conflict_info["blocker_anchor_distance"], 3),
                    "anchor": self._point_to_list(conflict_info["anchor"]),
                    "same_target_port": conflict_info["same_target_port"],
                    "shared_active_port": conflict_info["shared_active_port"],
                    "operation_owner": conflict_info["operation_owner"],
                    **self._conflict_debug_payload(blocker_state),
                },
            )
            # #endregion
            # #region debug-point B:baseline-collision-decision
            self._debug_collision_event(
                "B",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] baseline planner chose conflict-entry hold",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "decision": "baseline_zone_wait",
                    "blocker_distance": round(blocker_distance, 3),
                },
            )
            # #endregion
            # #region debug-point E:zone-wait
            self._debug_multi_stop_event(
                "E",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] multi-stop planner chose conflict-entry hold",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "blocker_id": blocker_id,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_base_state": blocker_base_state,
                    "blocker_distance": round(blocker_distance, 3),
                    "allow_stop_fallback": allow_stop_fallback,
                    "same_target_port": conflict_info["same_target_port"],
                    "shared_active_port": conflict_info["shared_active_port"],
                    "operation_owner": conflict_info["operation_owner"],
                },
            )
            # #endregion
            # #region debug-point D:agent1-zone-wait
            if getattr(self.agent, "id", None) == 1:
                self._debug_port_speed_event(
                    "D",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] agent1 planner chose baseline_zone_wait near congested port entry",
                    {
                        "agent_id": 1,
                        "blocker_id": blocker_id,
                        "blocker_state": self._observed_state_name(blocker_state),
                        "blocker_distance": round(blocker_distance, 3),
                        "allow_stop_fallback": allow_stop_fallback,
                        "same_target_port": conflict_info["same_target_port"],
                        "shared_active_port": conflict_info["shared_active_port"],
                        "operation_owner": conflict_info["operation_owner"],
                    },
                )
            # #endregion
            return {
                "command": (0.0, 0.0),
                "reason": "baseline_zone_wait",
                "blocker_id": blocker_id,
            }
        if (
            not allow_stop_fallback
            and own_state_name in {"CRUISE", "PREQUEUE"}
            and not conflict_info["same_target_port"]
            # 会车过程中，对侧可能先一步切进 QUEUING，但实际 corridor 冲突还没有解除，
            # 让行侧需要继续沿用 direct bypass，不能中途退回普通 replan。
            and blocker_base_state in {"CRUISE", "PREQUEUE", "QUEUING"}
            and blocker_distance <= max(self.MOVING_BLOCKER_STOP_DISTANCE, self.CONFLICT_ENTRY_DISTANCE + 0.7)
        ):
            if (
                non_port_moving_pair
                and should_yield
                and (
                    (
                        conflict_info.get("corridor_conflict")
                        and blocker_distance <= self.MOVING_BLOCKER_STOP_DISTANCE
                    )
                    or (
                        conflict_info.get("predictive_conflict")
                        and blocker_distance <= self.PREDICTIVE_CORRIDOR_HOLD_DISTANCE
                    )
                )
            ):
                backoff_command = None
                if blocker_distance <= self.NON_PORT_YIELD_BACKOFF_DISTANCE:
                    backoff_command = self._corridor_yield_backoff_command(
                        position,
                        sequence_of_poses,
                        blocker_state,
                    )
                if backoff_command is not None:
                    self._debug_event(
                        "C",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose active backoff for close predictive corridor yield",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "blocker_distance": round(blocker_distance, 3),
                            "blocker_state": self._observed_state_name(blocker_state),
                            "corridor_conflict": conflict_info.get("corridor_conflict"),
                            "predictive_conflict": conflict_info.get("predictive_conflict"),
                        },
                    )
                    self._debug_collision_event(
                        "B",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose close-corridor yield backoff",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "decision": "baseline_non_port_corridor_backoff",
                            "blocker_distance": round(blocker_distance, 3),
                            "command": [round(backoff_command[0], 3), round(backoff_command[1], 3)],
                        },
                    )
                    # #region debug-point D:agent1-corridor-backoff
                    if getattr(self.agent, "id", None) == 1:
                        self._debug_port_speed_event(
                            "D",
                            "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                            "[DEBUG] agent1 planner chose corridor backoff near congested port corridor",
                            {
                                "agent_id": 1,
                                "blocker_id": blocker_id,
                                "blocker_state": self._observed_state_name(blocker_state),
                                "blocker_distance": round(blocker_distance, 3),
                                "corridor_conflict": conflict_info.get("corridor_conflict"),
                                "predictive_conflict": conflict_info.get("predictive_conflict"),
                            },
                        )
                    # #endregion
                    return {
                        "command": backoff_command,
                        "reason": "baseline_non_port_corridor_backoff",
                        "blocker_id": blocker_id,
                    }
                self._debug_event(
                    "C",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner held yielding through-agent inside predictive corridor conflict",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "blocker_distance": round(blocker_distance, 3),
                        "blocker_state": self._observed_state_name(blocker_state),
                        "corridor_conflict": conflict_info.get("corridor_conflict"),
                        "predictive_conflict": conflict_info.get("predictive_conflict"),
                    },
                )
                self._debug_collision_event(
                    "B",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner chose predictive close-corridor yield hold",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "decision": "baseline_non_port_corridor_hold",
                        "blocker_distance": round(blocker_distance, 3),
                    },
                )
                # #region debug-point D:agent1-corridor-hold
                if getattr(self.agent, "id", None) == 1:
                    self._debug_port_speed_event(
                        "D",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] agent1 planner chose corridor hold near congested port corridor",
                        {
                            "agent_id": 1,
                            "blocker_id": blocker_id,
                            "blocker_state": self._observed_state_name(blocker_state),
                            "blocker_distance": round(blocker_distance, 3),
                            "corridor_conflict": conflict_info.get("corridor_conflict"),
                            "predictive_conflict": conflict_info.get("predictive_conflict"),
                        },
                    )
                # #endregion
                return {
                    "command": (0.0, 0.0),
                    "reason": "baseline_non_port_corridor_hold",
                    "blocker_id": blocker_id,
                }
            if (
                non_port_moving_pair
                and not should_yield
                and conflict_info.get("future_conflict")
                and blocker_distance <= self.PREDICTIVE_CORRIDOR_HOLD_DISTANCE
            ):
                escape_command = self._corridor_yield_backoff_command(
                    position,
                    sequence_of_poses,
                    blocker_state,
                )
                if escape_command is not None:
                    self._debug_event(
                        "C",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose active escape for close non-yielding corridor conflict",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "blocker_distance": round(blocker_distance, 3),
                            "blocker_state": self._observed_state_name(blocker_state),
                            "future_conflict": conflict_info.get("future_conflict"),
                        },
                    )
                    self._debug_collision_event(
                        "B",
                        "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                        "[DEBUG] baseline planner chose close-corridor escape",
                        {
                            "agent_id": getattr(self.agent, "id", None),
                            "blocker_id": blocker_id,
                            "decision": "baseline_non_port_corridor_escape",
                            "blocker_distance": round(blocker_distance, 3),
                            "command": [round(escape_command[0], 3), round(escape_command[1], 3)],
                        },
                    )
                    # #region debug-point D:agent1-corridor-escape
                    if getattr(self.agent, "id", None) == 1:
                        self._debug_port_speed_event(
                            "D",
                            "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                            "[DEBUG] agent1 planner chose corridor escape near congested port corridor",
                            {
                                "agent_id": 1,
                                "blocker_id": blocker_id,
                                "blocker_state": self._observed_state_name(blocker_state),
                                "blocker_distance": round(blocker_distance, 3),
                                "future_conflict": conflict_info.get("future_conflict"),
                            },
                        )
                    # #endregion
                    return {
                        "command": escape_command,
                        "reason": "baseline_non_port_corridor_escape",
                        "blocker_id": blocker_id,
                    }
            bypass_command = self._direct_bypass_command(position, sequence_of_poses, blocker_state)
            if bypass_command is not None:
                # #region debug-point C:baseline-bypass-selected
                self._debug_event(
                    "C",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner selected direct bypass for through-agent",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "blocker_distance": round(blocker_distance, 3),
                        "blocker_state": self._observed_state_name(blocker_state),
                        "blocker_base_state": blocker_base_state,
                        "allow_stop_fallback": allow_stop_fallback,
                    },
                )
                # #endregion
                # #region debug-point B:baseline-collision-decision
                self._debug_collision_event(
                    "B",
                    "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                    "[DEBUG] baseline planner chose direct bypass",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "decision": "baseline_direct_bypass",
                        "blocker_distance": round(blocker_distance, 3),
                        "command": [round(bypass_command[0], 3), round(bypass_command[1], 3)],
                    },
                )
                # #endregion
                return {
                    "command": bypass_command,
                    "reason": "baseline_direct_bypass",
                    "blocker_id": blocker_id,
                }

        # #region debug-point C:baseline-replan-selected
        self._debug_event(
            "C",
            "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
            "[DEBUG] baseline planner selected replan for direct conflict",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": blocker_id,
                "blocker_distance": round(blocker_distance, 3),
                "blocker_state": self._observed_state_name(blocker_state),
                "blocker_base_state": blocker_base_state,
                "allow_stop_fallback": allow_stop_fallback,
                **self._conflict_debug_payload(blocker_state),
            },
        )
        # #endregion
        # #region debug-point B:baseline-collision-decision
        self._debug_collision_event(
            "B",
            "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
            "[DEBUG] baseline planner chose predicted-collision replan",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": blocker_id,
                "decision": "predicted_collision",
                "blocker_distance": round(blocker_distance, 3),
                "allow_stop_fallback": allow_stop_fallback,
            },
        )
        # #endregion
        # #region debug-point D:agent1-direct-replan
        if getattr(self.agent, "id", None) == 1:
            self._debug_port_speed_event(
                "D",
                "layered_a_star_baseline_traffic_aware_planner.py:compute_avoidance_response",
                "[DEBUG] agent1 planner chose predicted-collision replan near congested port",
                {
                    "agent_id": 1,
                    "blocker_id": blocker_id,
                    "blocker_state": self._observed_state_name(blocker_state),
                    "blocker_distance": round(blocker_distance, 3),
                    "allow_stop_fallback": allow_stop_fallback,
                    "same_target_port": conflict_info.get("same_target_port"),
                    "shared_active_port": conflict_info.get("shared_active_port"),
                    "operation_owner": conflict_info.get("operation_owner"),
                },
            )
        # #endregion
        return {
            "replan": True,
            "fallback_command": (0.0, 0.0),
            "reason": "predicted_collision",
            "blocker_id": blocker_id,
            "stop_distance": self._stop_distance_for_state(blocker_state),
            "allow_stop_fallback": allow_stop_fallback,
        }

    def path_conflicts_with_blocker(self, position, sequence_of_poses, sensor_observation, blocker_id):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is None:
            return False
        blocker_state = getattr(blocker, "userData", blocker)
        return self._conflicts_with_blocker_state(position, sequence_of_poses, blocker_state, blocker)

    def _conflicts_with_blocker_state(self, position, sequence_of_poses, blocker_state, blocker_agent=None):
        if self._is_explicit_yielder_for_self(blocker_state):
            return False
        if self._port_owner_should_hold(blocker_state):
            return False
        return self._conflict_info(position, sequence_of_poses, blocker_state, blocker_agent)["conflicts"]

    def _should_consider_for_dynamic_layer(
        self,
        position,
        sequence_of_poses,
        observed_state,
        observed_agent=None,
        gridmap=None,
        predictive_profile=None,
    ):
        blocker_position = getattr(observed_state, "position", None)
        if blocker_position is None:
            return False
        if self._is_explicit_yielder_for_self(observed_state):
            return False
        if self._port_owner_should_hold(observed_state):
            return False
        if predictive_profile is None:
            predictive_profile = self._predictive_conflict_profile(
                position,
                sequence_of_poses,
                observed_agent,
                gridmap,
            )
        if self._is_same_target_port_queue_or_owner(observed_state):
            target_port = self._target_port_for_state(self.agent)
            if not self._port_relevance_for_self(target_port, position, sequence_of_poses):
                return False
            if not (
                self._occupies_front_zone_footprint(observed_state, target_port)
                or self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position)
                or predictive_profile["active"]
            ):
                return False
            return (
                position.distance(blocker_position) <= self.KEYZONE_APPROACH_DISTANCE
                or predictive_profile["active"]
            )
        if predictive_profile["active"]:
            return True
        if self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position):
            return True
        for pose in list(sequence_of_poses)[: self.LOOKAHEAD_POINTS]:
            if pose.distance(blocker_position) <= self.REPLAN_DISTANCE:
                return True
        return False

    def _find_observed_agent_by_id(self, sensor_observation, blocker_id):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) == blocker_id:
                return observed_agent
        return None

    def _distance_to_blocker(self, position, sensor_observation, blocker_id):
        blocker = self._find_observed_agent_by_id(sensor_observation, blocker_id)
        if blocker is None:
            return float("inf")
        blocker_state = getattr(blocker, "userData", blocker)
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return float("inf")
        return position.distance(blocker_position)

    def _collect_reservation_nodes(self, observed_agent, gridmap):
        if observed_agent is None or gridmap is None:
            return []
        observed_state = getattr(observed_agent, "userData", observed_agent)
        current_position = getattr(observed_state, "position", None)
        if current_position is None:
            return []

        nodes = [agent_to_gridmap(current_position, gridmap)]
        path_nodes = self._path_nodes_from_waypoints(observed_state, gridmap, current_position)
        if path_nodes:
            nodes.extend(path_nodes)
        else:
            nodes.extend(self._path_nodes_from_velocity(observed_agent, observed_state, gridmap, current_position))

        deduped = []
        for node in nodes:
            if not deduped or node != deduped[-1]:
                deduped.append(node)
        return deduped[: self.RESERVATION_WAYPOINTS + 1]

    def _collect_own_reservation_nodes(self, gridmap, current_position, sequence_of_poses):
        if gridmap is None or current_position is None:
            return []
        nodes = [agent_to_gridmap(current_position, gridmap)]
        for pose in list(sequence_of_poses)[: self.RESERVATION_WAYPOINTS]:
            if current_position.distance(pose) > self.RESERVATION_WINDOW_DISTANCE:
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
            if current_position.distance(pose) > self.RESERVATION_WINDOW_DISTANCE:
                continue
            nodes.append(agent_to_gridmap(pose, gridmap))
            if len(nodes) >= self.RESERVATION_WAYPOINTS:
                break
        return nodes

    def _path_nodes_from_velocity(self, observed_agent, observed_state, gridmap, current_position):
        velocity = getattr(observed_state, "linear_velocity", None)
        if velocity is None:
            velocity = getattr(observed_agent, "linearVelocity", None)
        if velocity is None:
            return []

        vx = float(getattr(velocity, "x", 0.0) if hasattr(velocity, "x") else 0.0)
        vy = float(getattr(velocity, "y", 0.0) if hasattr(velocity, "y") else 0.0)
        if not hasattr(velocity, "x"):
            try:
                vx = float(velocity[0])
                vy = float(velocity[1])
            except Exception:
                vx, vy = 0.0, 0.0
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
        if not reservation_nodes:
            return
        for step_idx, node in enumerate(reservation_nodes):
            decay = self.RESERVATION_DECAY ** step_idx
            node_inflation = self.RESERVATION_NODE_INFLATION * node_scale * decay
            edge_inflation = self.RESERVATION_EDGE_INFLATION * edge_scale * decay
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

    def _reservation_conflict_profile(self, own_nodes, observed_nodes):
        own_slice = own_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1]
        observed_slice = observed_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1]
        shared_nodes = len(set(own_slice).intersection(observed_slice))

        observed_edges = set(zip(observed_slice, observed_slice[1:]))
        reverse_edges = 0
        aligned_edges = 0
        for own_start, own_end in zip(own_slice, own_slice[1:]):
            if (own_end, own_start) in observed_edges:
                reverse_edges += 1
            if (own_start, own_end) in observed_edges:
                aligned_edges += 1
        return {
            "shared_nodes": shared_nodes,
            "aligned_edges": aligned_edges,
            "reverse_edges": reverse_edges,
        }

    def _predictive_conflict_profile(self, position, sequence_of_poses, observed_agent, gridmap):
        if observed_agent is None or gridmap is None or position is None:
            return {
                "active": False,
                "shared_nodes": 0,
                "aligned_edges": 0,
                "reverse_edges": 0,
                "own_nodes": [],
                "observed_nodes": [],
            }
        own_nodes = self._collect_own_reservation_nodes(gridmap, position, sequence_of_poses)
        observed_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
        if not own_nodes or not observed_nodes:
            return {
                "active": False,
                "shared_nodes": 0,
                "aligned_edges": 0,
                "reverse_edges": 0,
                "own_nodes": own_nodes,
                "observed_nodes": observed_nodes,
            }
        profile = self._reservation_conflict_profile(own_nodes, observed_nodes)
        profile["active"] = (
            profile["reverse_edges"] > 0
            or profile["aligned_edges"] > 0
            or profile["shared_nodes"] >= 2
        )
        profile["own_nodes"] = own_nodes
        profile["observed_nodes"] = observed_nodes
        return profile

    def _requires_predictive_replan(
        self,
        position,
        sequence_of_poses,
        observed_state,
        blocker_distance,
        predictive_profile,
    ):
        if not predictive_profile["active"]:
            return False
        if blocker_distance > self.PREDICTIVE_REPLAN_DISTANCE:
            return False
        if self._is_explicit_yielder_for_self(observed_state):
            return False
        if self._port_owner_should_hold(observed_state):
            return False
        if self._is_same_target_port_queue_or_owner(observed_state):
            target_port = self._target_port_for_state(self.agent)
            return self._port_relevance_for_self(target_port, position, sequence_of_poses)
        return True

    def _apply_blocker_barrier(self, dynamic_layer, gridmap, blocker_position, node_inflation, edge_inflation):
        blocker_origin = agent_to_gridmap(blocker_position, gridmap)
        self._inflate_transition_ring(dynamic_layer, gridmap, blocker_origin, node_inflation)
        for first in gridmap.neighbors(blocker_origin):
            self._inflate_transition_ring(dynamic_layer, gridmap, first, edge_inflation)
            for second in gridmap.neighbors(first):
                if second == blocker_origin:
                    continue
                self._inflate_transition_ring(dynamic_layer, gridmap, second, edge_inflation * 0.45)

    def _inflate_transition_ring(self, dynamic_layer, gridmap, node, inflation):
        for neighbor in gridmap.neighbors(node):
            self.add_inflation(dynamic_layer, node, neighbor, inflation)
            self.add_inflation(dynamic_layer, neighbor, node, inflation)

    def _blocker_on_current_corridor(self, position, sequence_of_poses, blocker_position):
        if position.distance(blocker_position) <= self.CORRIDOR_DISTANCE:
            return True
        for pose in list(sequence_of_poses)[:2]:
            if pose.distance(blocker_position) <= self.CORRIDOR_DISTANCE:
                return True
        return False

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
                    own_start,
                    own_end,
                    other_start,
                    other_end,
                )
                if square_distance <= self.YIELD_SEGMENT_DISTANCE ** 2:
                    return True

        for own_point in own_future[:3]:
            for other_point in observed_future[:3]:
                if own_point.distance(other_point) <= self.YIELD_DISTANCE:
                    return True
        return False

    def _conflict_info(self, position, sequence_of_poses, blocker_state, blocker_agent=None):
        blocker_position = getattr(blocker_state, "position", None)
        if blocker_position is None:
            return {
                "conflicts": False,
                "anchor": None,
                "anchor_distance": float("inf"),
                "blocker_anchor_distance": float("inf"),
                "blocker_distance": float("inf"),
                "corridor_conflict": False,
                "predictive_conflict": False,
                "future_conflict": False,
                "same_target_port": False,
                "shared_active_port": False,
                "operation_owner": False,
            }
        if self._is_explicit_yielder_for_self(blocker_state):
            return {
                "conflicts": False,
                "anchor": None,
                "anchor_distance": float("inf"),
                "blocker_anchor_distance": float("inf"),
                "blocker_distance": position.distance(blocker_position),
                "corridor_conflict": False,
                "predictive_conflict": False,
                "future_conflict": False,
                "same_target_port": False,
                "shared_active_port": False,
                "operation_owner": False,
            }
        if self._port_owner_should_hold(blocker_state):
            return {
                "conflicts": False,
                "anchor": None,
                "anchor_distance": float("inf"),
                "blocker_anchor_distance": float("inf"),
                "blocker_distance": position.distance(blocker_position),
                "corridor_conflict": False,
                "predictive_conflict": False,
                "future_conflict": False,
                "same_target_port": False,
                "shared_active_port": False,
                "operation_owner": False,
            }

        blocker_distance = position.distance(blocker_position)
        same_target_port = self._is_same_target_port_queue_or_owner(blocker_state)
        shared_active_port = self._shares_active_port(self.agent, blocker_state)
        operation_owner = self._is_operation_owner(blocker_state)
        corridor_conflict = self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position)
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        blocker_future = self._future_points(
            blocker_position,
            getattr(blocker_state, "sequence_of_poses", []),
            getattr(blocker_state, "linear_velocity", None)
            or getattr(blocker_agent, "linearVelocity", None),
        )
        future_conflict = self._future_paths_conflict(own_future, blocker_future)
        predictive_profile = self._predictive_conflict_profile(
            position,
            sequence_of_poses,
            blocker_agent,
            getattr(self.agent, "static_environment", None),
        )
        predictive_conflict = (
            predictive_profile["active"]
            and blocker_distance <= self.PREDICTIVE_REPLAN_DISTANCE
        )
        absolute_stopping = self._is_absolute_stopping_blocker(position, sequence_of_poses, blocker_state)
        corridor_conflict = corridor_conflict or predictive_conflict
        conflicts = (
            absolute_stopping
            or future_conflict
            or predictive_conflict
            or (corridor_conflict and blocker_distance <= self.STOP_DISTANCE)
            or same_target_port
            or shared_active_port
        )
        anchor = self._conflict_anchor_point(position, blocker_position, own_future, blocker_future, blocker_state)
        anchor_distance = position.distance(anchor) if anchor is not None else float("inf")
        blocker_anchor_distance = blocker_position.distance(anchor) if anchor is not None else float("inf")
        return {
            "conflicts": conflicts,
            "anchor": anchor,
            "anchor_distance": anchor_distance,
            "blocker_anchor_distance": blocker_anchor_distance,
            "blocker_distance": blocker_distance,
            "corridor_conflict": corridor_conflict,
            "predictive_conflict": predictive_conflict,
            "future_conflict": future_conflict,
            "same_target_port": same_target_port,
            "shared_active_port": shared_active_port,
            "operation_owner": operation_owner,
        }

    def _conflict_anchor_point(self, position, blocker_position, own_future, blocker_future, blocker_state):
        active_port = self._active_port_for_state(self.agent) or self._active_port_for_state(blocker_state)
        if self._shares_active_port(self.agent, blocker_state) and active_port is not None:
            return self._port_anchor(active_port)
        if self._is_same_target_port_queue_or_owner(blocker_state) or self._is_operation_owner(blocker_state):
            return Point(blocker_position.x, blocker_position.y)

        best_anchor = None
        best_score = None
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            for other_index in range(len(blocker_future) - 1):
                other_start = blocker_future[other_index]
                other_end = blocker_future[other_index + 1]
                square_distance, nearest_pair = edge_edge_shortest_square_distance(
                    own_start,
                    own_end,
                    other_start,
                    other_end,
                )
                if square_distance > self.YIELD_SEGMENT_DISTANCE ** 2:
                    continue
                if nearest_pair is not None:
                    anchor = nearest_pair
                else:
                    anchor = Point(
                        (own_start.x + own_end.x + other_start.x + other_end.x) / 4.0,
                        (own_start.y + own_end.y + other_start.y + other_end.y) / 4.0,
                    )
                score = (
                    own_index + other_index,
                    position.distance(anchor),
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_anchor = anchor

        if best_anchor is not None:
            return best_anchor
        return Point(blocker_position.x, blocker_position.y)

    def _should_wait_before_entry(self, conflict_info, allow_stop_fallback):
        anchor = conflict_info["anchor"]
        if anchor is None:
            return False
        anchor_distance = conflict_info["anchor_distance"]
        if anchor_distance <= self.CONFLICT_INNER_DISTANCE:
            return False
        if anchor_distance > self.CONFLICT_ENTRY_DISTANCE:
            return False
        if not allow_stop_fallback:
            return False
        if conflict_info["operation_owner"] or conflict_info["same_target_port"] or conflict_info["shared_active_port"]:
            return True
        return conflict_info["blocker_anchor_distance"] <= self.CONFLICT_ENTRY_DISTANCE + 0.35

    def _point_to_list(self, point):
        if point is None:
            return None
        return [round(point.x, 3), round(point.y, 3)]

    def _future_path_hits_stopping_agent(self, own_future, stopping_position):
        stop_point = Point(stopping_position.x, stopping_position.y)
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            square_distance, _ = edge_edge_shortest_square_distance(
                own_start,
                own_end,
                stop_point,
                stop_point,
            )
            if square_distance <= self.STOP_DISTANCE ** 2:
                return True
        for own_point in own_future[:3]:
            if own_point.distance(stop_point) <= self.STOP_DISTANCE:
                return True
        return False

    def _is_absolute_stopping_blocker(self, position, sequence_of_poses, observed_state):
        if self._observed_state_name(observed_state) != "STOPPING":
            return False
        if getattr(observed_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None):
            return False
        blocker_position = getattr(observed_state, "position", None)
        if blocker_position is None:
            return False
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        return self._future_path_hits_stopping_agent(own_future, blocker_position)

    def _should_yield_to(self, observed_state):
        if self._port_owner_should_hold(observed_state):
            return False
        if self._must_yield_to_port_owner(observed_state):
            return True
        own_source_port = self._source_port_for_state(self.agent)
        observed_source_port = self._source_port_for_state(observed_state)
        if (
            self._is_exiting_port(self.agent)
            and own_source_port is not None
            and self._ports_match(self._target_port_for_state(observed_state), own_source_port)
        ):
            return False
        if (
            self._is_exiting_port(observed_state)
            and observed_source_port is not None
            and self._ports_match(self._target_port_for_state(self.agent), observed_source_port)
        ):
            return True
        if self._observed_state_name(observed_state) == "STOPPING":
            return getattr(observed_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None)
        observed_priority = self._priority_tuple(observed_state)
        own_priority = self._priority_tuple(self.agent)
        if observed_priority != own_priority:
            return observed_priority > own_priority
        return getattr(observed_state, "id", 0) < getattr(self.agent, "id", 0)

    def _requires_same_port_safety_hold(
        self,
        blocker_state,
        conflict_info,
        blocker_distance,
        same_port_predecessor,
        own_queue_progress=0,
        blocker_queue_progress=0,
    ):
        if not same_port_predecessor:
            return False
        if blocker_distance > self.SAME_PORT_SAFETY_HOLD_DISTANCE:
            return False
        blocker_base_state = self._base_state_name(blocker_state)
        if (
            blocker_base_state == "QUEUING"
            and own_queue_progress > 0
            and blocker_queue_progress > 0
        ):
            if blocker_distance > self.SAME_PORT_QUEUE_CHAIN_HOLD_DISTANCE:
                return False
            return conflict_info.get("corridor_conflict", False) or conflict_info.get("predictive_conflict", False)
        if conflict_info["shared_active_port"] or conflict_info["operation_owner"]:
            return True
        return conflict_info.get("corridor_conflict", False) or conflict_info.get("predictive_conflict", False)

    def _requires_same_source_safety_hold(self, conflict_info, blocker_distance, entering_same_source_hold):
        if not entering_same_source_hold:
            return False
        if blocker_distance > self.SAME_SOURCE_SAFETY_HOLD_DISTANCE:
            return False
        return (
            conflict_info.get("corridor_conflict", False)
            or conflict_info.get("predictive_conflict", False)
            or conflict_info["shared_active_port"]
            or conflict_info["operation_owner"]
        )

    def _requires_early_corridor_replan(
        self,
        position,
        sequence_of_poses,
        observed_state,
        blocker_distance,
        corridor_conflict,
    ):
        if blocker_distance > self.EARLY_CORRIDOR_REPLAN_DISTANCE or not corridor_conflict:
            return False
        if self._is_explicit_yielder_for_self(observed_state):
            return False
        if self._is_same_target_port_queue_or_owner(observed_state):
            target_port = self._target_port_for_state(self.agent)
            return self._port_relevance_for_self(target_port, position, sequence_of_poses)
        source_port = self._source_port_for_state(observed_state)
        return (
            self._is_exiting_port(observed_state)
            and source_port is not None
            and self._ports_match(self._target_port_for_state(self.agent), source_port)
        )

    def _is_non_port_moving_pair(self, own_state_name, blocker_state, conflict_info):
        blocker_base_state = self._base_state_name(blocker_state)
        moving_states = {"CRUISE", "PREQUEUE", "QUEUING"}
        if own_state_name not in moving_states:
            return False
        if blocker_base_state not in moving_states:
            return False
        return not (
            conflict_info["same_target_port"]
            or conflict_info["shared_active_port"]
            or conflict_info["operation_owner"]
        )

    def _same_target_port_conflict_role(self, blocker_state, conflict_info):
        target_port = self._target_port_for_state(self.agent)
        blocker_target_port = self._target_port_for_state(blocker_state)
        if not (
            conflict_info["same_target_port"]
            or self._ports_match(blocker_target_port, target_port)
        ):
            return None
        blocker_base_state = self._base_state_name(blocker_state)
        if blocker_base_state not in {"CRUISE", "PREQUEUE", "QUEUING", "LOADING", "UNLOADING"}:
            return None
        own_progress = self._queue_progress_score(self.agent)
        blocker_progress = self._queue_progress_score(blocker_state)
        if target_port is None:
            target_port = blocker_target_port
        if blocker_base_state in {"LOADING", "UNLOADING"}:
            return "following"
        if own_progress > 0 and blocker_progress <= 0:
            return "leading"
        if blocker_progress <= 0:
            return self._same_target_port_approach_role(blocker_state, target_port, conflict_info)
        if own_progress <= 0:
            return "following"
        if blocker_progress > own_progress:
            return "following"
        if blocker_progress < own_progress:
            return "leading"
        return None

    def _same_target_port_approach_role(self, blocker_state, target_port, conflict_info):
        if target_port is None:
            return None
        if not (
            conflict_info.get("corridor_conflict")
            or conflict_info["shared_active_port"]
            or conflict_info["same_target_port"]
        ):
            return None

        own_distance = self._distance_to_port_anchor(self.agent, target_port)
        blocker_distance = self._distance_to_port_anchor(blocker_state, target_port)
        if own_distance is None or blocker_distance is None:
            return None

        if blocker_distance + 1e-3 < own_distance:
            return "following"
        if own_distance + 1e-3 < blocker_distance:
            return "leading"
        if getattr(blocker_state, "id", 0) < getattr(self.agent, "id", 0):
            return "following"
        if getattr(blocker_state, "id", 0) > getattr(self.agent, "id", 0):
            return "leading"
        return None

    def _priority_tuple(self, agent_state):
        return (
            self._priority_score(agent_state),
            self._queue_progress_score(agent_state),
            -getattr(agent_state, "id", 0),
        )

    def _same_target_port_approach_response(self, position, sensor_observation, sequence_of_poses):
        target_port = self._target_port_for_state(self.agent)
        if target_port is None:
            return None
        own_anchor_distance = self._distance_to_port_anchor(self.agent, target_port)
        if own_anchor_distance is None:
            return None

        own_radius = float(getattr(getattr(self.agent, "shape", None), "get_radius", lambda: 0.5)() or 0.5)
        trigger_distance = max(self.KEYZONE_APPROACH_DISTANCE, own_radius * 6.0)
        candidate = None

        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            blocker_position = getattr(observed_state, "position", None)
            blocker_base_state = self._base_state_name(observed_state)
            if blocker_id is None or blocker_id == getattr(self.agent, "id", None) or blocker_position is None:
                continue
            if blocker_base_state not in {"CRUISE", "PREQUEUE"}:
                continue
            if not self._ports_match(self._target_port_for_state(observed_state), target_port):
                continue

            blocker_anchor_distance = self._distance_to_port_anchor(observed_state, target_port)
            if blocker_anchor_distance is None or blocker_anchor_distance + 1e-3 >= own_anchor_distance:
                continue

            blocker_distance = position.distance(blocker_position)
            corridor_conflict = self._blocker_on_current_corridor(position, sequence_of_poses, blocker_position)
            if not corridor_conflict:
                continue

            score = (
                blocker_anchor_distance,
                blocker_distance,
                blocker_id,
            )
            if candidate is None or score < candidate["score"]:
                candidate = {
                    "score": score,
                    "blocker_id": blocker_id,
                    "blocker_distance": blocker_distance,
                    "blocker_anchor_distance": blocker_anchor_distance,
                    "blocker_state": observed_state,
                    "corridor_conflict": corridor_conflict,
                }

        if candidate is None:
            return None

        self._debug_collision_event(
            "B",
            "layered_a_star_baseline_traffic_aware_planner.py:_same_target_port_approach_response",
            "[DEBUG] baseline planner held trailing same-target approach agent before queue entry",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": candidate["blocker_id"],
                "blocker_distance": round(candidate["blocker_distance"], 3),
                "blocker_anchor_distance": round(candidate["blocker_anchor_distance"], 3),
                "own_anchor_distance": round(own_anchor_distance, 3),
                "corridor_conflict": candidate["corridor_conflict"],
                "target_port_id": getattr(target_port, "id", None),
                "target_port_type": getattr(target_port, "port_type", None),
            },
        )
        # #region debug-point E:same-target-approach-hold
        self._debug_multi_stop_event(
            "E",
            "layered_a_star_baseline_traffic_aware_planner.py:_same_target_port_approach_response",
            "[DEBUG] multi-stop planner chose same-target approach hold",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": candidate["blocker_id"],
                "blocker_distance": round(candidate["blocker_distance"], 3),
                "blocker_anchor_distance": round(candidate["blocker_anchor_distance"], 3),
                "own_anchor_distance": round(own_anchor_distance, 3),
                "corridor_conflict": candidate["corridor_conflict"],
                "target_port_id": getattr(target_port, "id", None),
            },
        )
        # #endregion
        return {
            "command": (0.0, 0.0),
            "reason": "baseline_same_target_approach_hold",
            "blocker_id": candidate["blocker_id"],
        }

    def _priority_score(self, agent_state):
        state_name = self._observed_state_name(agent_state)
        if state_name == "STOPPING":
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
        queue = getattr(port, "queue", None) if port is not None else None
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

    def _stop_distance_for_state(self, blocker_state):
        blocker_base_state = self._base_state_name(blocker_state)
        if blocker_base_state in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return self.MOVING_BLOCKER_STOP_DISTANCE
        return self.STOP_DISTANCE

    def _observed_state_name(self, agent_state):
        if self._has_blocking_stopping_overlay(agent_state):
            return "STOPPING"
        return getattr(getattr(agent_state, "state", None), "name", None)

    def _base_state_name(self, agent_state):
        if getattr(agent_state, "stopping_active", False):
            return getattr(getattr(agent_state, "stopping_base_state", None), "name", None)
        return getattr(getattr(agent_state, "state", None), "name", None)

    def _has_blocking_stopping_overlay(self, agent_state):
        if not getattr(agent_state, "stopping_active", False):
            return False
        return getattr(agent_state, "stopping_reason", None) not in self.PASSIVE_STOPPING_REASONS

    def _is_operation_owner(self, agent_state):
        if self._base_state_name(agent_state) not in {"LOADING", "UNLOADING"}:
            return False
        port = self._target_port_for_state(agent_state)
        anchor = self._port_anchor(port)
        position = getattr(agent_state, "position", None)
        if port is None or anchor is None or position is None:
            return False
        tolerance = max(0.08, float(getattr(port, "operation_entry_tolerance", 0.08) or 0.08))
        return position.distance(anchor) <= tolerance

    def _port_anchor(self, port):
        return getattr(port, "operation_zone", None) or getattr(port, "location", None)

    def _distance_to_port_anchor(self, agent_state, port):
        position = getattr(agent_state, "position", None)
        anchor = self._port_anchor(port)
        if position is None or anchor is None:
            return None
        return position.distance(anchor)

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
            port
            for port in self._all_ports()
            if getattr(port, "port_type", None) == port_type
        ]
        if not candidate_ports:
            return None
        return min(candidate_ports, key=lambda port: position.distance(self._port_anchor(port)))

    def _is_exiting_port(self, agent_state):
        if self._base_state_name(agent_state) != "CRUISE":
            return False
        source_port = self._source_port_for_state(agent_state)
        target_port = self._target_port_for_state(agent_state)
        position = getattr(agent_state, "position", None)
        anchor = self._port_anchor(source_port)
        if source_port is None or position is None or anchor is None:
            return False
        target_anchor = self._port_anchor(target_port)
        if target_anchor is not None and position.distance(target_anchor) <= position.distance(anchor):
            return False
        return position.distance(anchor) <= self.EXIT_ZONE_RADIUS

    def _ports_match(self, lhs, rhs):
        if lhs is None or rhs is None:
            return False
        if lhs == rhs:
            return True
        return (
            getattr(lhs, "id", None) == getattr(rhs, "id", None)
            and getattr(lhs, "port_type", None) == getattr(rhs, "port_type", None)
        )

    def _all_ports(self):
        server = getattr(self.agent, "server", None)
        if server is None:
            return []
        return list(getattr(server, "loading_ports", [])) + list(getattr(server, "unloading_ports", []))

    def _queue_nodes_for_port(self, port, gridmap):
        queue = getattr(port, "queue", None) if port is not None else None
        if queue is None or gridmap is None:
            return []

        nodes = []
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

    def _front_zone_nodes(self, queue_nodes):
        return list(queue_nodes[: self.FRONT_ZONE_DEPTH])

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

    def _path_touches_front_zone(self, position, sequence_of_poses, port, gridmap=None):
        if position is None or port is None:
            return False
        gridmap = gridmap or getattr(self.agent, "static_environment", None)
        if gridmap is None:
            return False
        footprint = self._front_zone_node_footprint(port, gridmap)
        if not footprint:
            return False

        points = [position] + list(sequence_of_poses)[:2]
        for point in points:
            if point is not None and agent_to_gridmap(point, gridmap) in footprint:
                return True
        return False

    def _front_zone_port_for_state(self, agent_state):
        if self._is_exiting_port(agent_state):
            source_port = self._source_port_for_state(agent_state)
            if self._occupies_front_zone_footprint(agent_state, source_port):
                return source_port

        target_port = self._target_port_for_state(agent_state)
        state_name = self._base_state_name(agent_state)
        if state_name in {"LOADING", "UNLOADING"}:
            return target_port
        if state_name == "QUEUING" and self._occupies_front_zone_footprint(agent_state, target_port):
            return target_port
        if (
            self._observed_state_name(agent_state) == "STOPPING"
            and target_port is not None
            and self._occupies_front_zone_footprint(agent_state, target_port)
        ):
            return target_port
        return None

    def _port_relevance_for_self(self, port, position=None, sequence_of_poses=None):
        if port is None:
            return False
        target_port = self._target_port_for_state(self.agent)
        if self._ports_match(self._front_zone_port_for_state(self.agent), port):
            return True
        if not self._ports_match(target_port, port):
            return False
        position = position or getattr(self.agent, "position", None)
        if position is None:
            return False
        if self._occupies_front_zone_footprint(self.agent, port):
            return True
        preview_poses = sequence_of_poses
        if preview_poses is None:
            preview_poses = getattr(self.agent, "sequence_of_poses", [])
        if self._path_touches_front_zone(position, preview_poses, port):
            return True
        return False

    def _port_role_rank(self, agent_state, port):
        if port is None:
            return 0
        if self._is_exiting_port(agent_state) and self._front_zone_port_for_state(agent_state) == port:
            return 4
        state_name = self._base_state_name(agent_state)
        if state_name in {"LOADING", "UNLOADING"} and self._target_port_for_state(agent_state) == port:
            return 3
        if self._front_zone_port_for_state(agent_state) == port:
            return 2
        return 0

    def _port_priority_key(self, agent_state, port):
        queue_progress = self._queue_progress_score(agent_state) if self._target_port_for_state(agent_state) == port else 0
        return (
            self._port_role_rank(agent_state, port),
            queue_progress,
            -getattr(agent_state, "id", 0),
        )

    def _must_yield_to_port_owner(self, observed_state, position=None, sequence_of_poses=None):
        port = self._front_zone_port_for_state(observed_state)
        if port is None:
            return False
        if not self._port_relevance_for_self(port, position, sequence_of_poses):
            return False
        own_role = self._port_role_rank(self.agent, port)
        observed_role = self._port_role_rank(observed_state, port)
        return observed_role > own_role

    def _is_explicit_yielder_for_self(self, agent_state):
        if getattr(agent_state, "stopping_for_agent_id", None) != getattr(self.agent, "id", None):
            return False
        return getattr(agent_state, "stopping_reason", None) in {
            "baseline_zone_wait",
            "baseline_port_admission_wait",
            "baseline_same_port_queue_hold",
            "baseline_same_target_approach_hold",
            "baseline_exit_same_source_hold",
            "baseline_non_port_corridor_hold",
            "baseline_runtime_safety_hold",
            "baseline_runtime_safety_backoff",
        }

    def _port_owner_should_hold(self, observed_state):
        own_port = self._front_zone_port_for_state(self.agent)
        observed_port = self._front_zone_port_for_state(observed_state)
        if own_port is None or observed_port is None or own_port != observed_port:
            return False
        own_role = self._port_role_rank(self.agent, own_port)
        observed_role = self._port_role_rank(observed_state, own_port)
        return own_role >= 3 and own_role > observed_role

    def _port_admission_response(self, position, sensor_observation, sequence_of_poses):
        own_state_name = self._base_state_name(self.agent)
        if own_state_name not in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return None

        candidate = None
        own_front_port = self._front_zone_port_for_state(self.agent)
        for observed_agent in sensor_observation.other_agents_state_in_range_of(
            self.SENSOR_RANGE, self.SENSOR_ANGLE
        ):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_id = getattr(observed_state, "id", None)
            blocker_position = getattr(observed_state, "position", None)
            admission_port = self._front_zone_port_for_state(observed_state)
            if blocker_id is None or blocker_position is None or admission_port is None:
                continue
            blocker_distance = position.distance(blocker_position)
            own_key = self._port_priority_key(self.agent, admission_port)
            observed_key = self._port_priority_key(observed_state, admission_port)
            should_yield_to_owner = self._must_yield_to_port_owner(observed_state, position, sequence_of_poses)
            # #region debug-point C:same-port-admission-gate
            if self._should_trace_same_port_exit_pair(observed_state):
                self._debug_same_port_exit_event(
                    "C",
                    "layered_a_star_baseline_traffic_aware_planner.py:_port_admission_response",
                    "[DEBUG] same-port exit admission gate evaluated owner/follower pair",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": blocker_id,
                        "own_state": own_state_name,
                        "blocker_state": self._observed_state_name(observed_state),
                        "blocker_base_state": self._base_state_name(observed_state),
                        "blocker_distance": round(blocker_distance, 3),
                        "trigger_distance": round(self.PORT_ADMISSION_DISTANCE, 3),
                        "admission_port_id": getattr(admission_port, "id", None),
                        "admission_port_type": getattr(admission_port, "port_type", None),
                        "own_front_port_id": getattr(own_front_port, "id", None),
                        "blocker_front_port_id": getattr(admission_port, "id", None),
                        "own_role": own_key[0],
                        "blocker_role": observed_key[0],
                        "own_queue_progress": own_key[1],
                        "blocker_queue_progress": observed_key[1],
                        "should_yield_to_owner": should_yield_to_owner,
                        "blocker_is_exiting": self._is_exiting_port(observed_state),
                        "blocker_source_port_id": getattr(self._source_port_for_state(observed_state), "id", None),
                        "path_touches_front_zone": self._path_touches_front_zone(position, sequence_of_poses, admission_port),
                    },
                )
            # #endregion
            if getattr(observed_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None):
                continue
            if not should_yield_to_owner:
                continue
            if observed_key <= own_key:
                continue
            if observed_key[0] < own_key[0]:
                continue
            trigger_distance = self.PORT_ADMISSION_DISTANCE
            if blocker_distance > trigger_distance:
                continue
            if (
                getattr(observed_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None)
                and not self._occupies_front_zone_footprint(observed_state, admission_port)
            ):
                continue

            score = (
                -observed_key[0],
                -observed_key[1],
                position.distance(blocker_position),
                blocker_id,
            )
            if candidate is None or score < candidate["score"]:
                candidate = {
                    "score": score,
                    "blocker_id": blocker_id,
                    "blocker_state": observed_state,
                    "port": admission_port,
                    "blocker_distance": blocker_distance,
                    "own_key": own_key,
                    "observed_key": observed_key,
                    "trigger_distance": trigger_distance,
                }

        if candidate is None:
            return None

        self._debug_event(
            "D",
            "layered_a_star_baseline_traffic_aware_planner.py:_port_admission_response",
            "[DEBUG] baseline planner held agent outside port front zone",
            {
                "agent_id": getattr(self.agent, "id", None),
                "blocker_id": candidate["blocker_id"],
                "port_id": getattr(candidate["port"], "id", None),
                "port_type": getattr(candidate["port"], "port_type", None),
                "own_role": candidate["own_key"][0],
                "observed_role": candidate["observed_key"][0],
                "own_queue_progress": candidate["own_key"][1],
                "observed_queue_progress": candidate["observed_key"][1],
                "blocker_state": self._observed_state_name(candidate["blocker_state"]),
                "blocker_base_state": self._base_state_name(candidate["blocker_state"]),
                "blocker_distance": round(candidate["blocker_distance"], 3),
                "trigger_distance": round(candidate["trigger_distance"], 3),
                "path_preview": self._point_preview(sequence_of_poses),
                **self._conflict_debug_payload(candidate["blocker_state"], candidate["port"]),
            },
        )
        admission_backoff = (
            candidate["port"] == self._target_port_for_state(self.agent)
            and self._is_exiting_port(candidate["blocker_state"])
            and self._source_port_for_state(candidate["blocker_state"]) == candidate["port"]
        )
        if admission_backoff:
            backoff_command = self._port_admission_backoff_command(
                position, sequence_of_poses, candidate["blocker_state"], candidate["port"]
            )
            if backoff_command is not None:
                self._debug_event(
                    "D",
                    "layered_a_star_baseline_traffic_aware_planner.py:_port_admission_response",
                    "[DEBUG] baseline planner selected backoff while waiting outside port front zone",
                    {
                        "agent_id": getattr(self.agent, "id", None),
                        "blocker_id": candidate["blocker_id"],
                        "port_id": getattr(candidate["port"], "id", None),
                        "port_type": getattr(candidate["port"], "port_type", None),
                        "blocker_distance": round(candidate["blocker_distance"], 3),
                        **self._conflict_debug_payload(candidate["blocker_state"], candidate["port"]),
                    },
                )
                return {
                    "command": backoff_command,
                    "reason": "baseline_port_admission_backoff",
                    "blocker_id": candidate["blocker_id"],
                }
        return {
            "command": (0.0, 0.0),
            "reason": "baseline_port_admission_wait",
            "blocker_id": candidate["blocker_id"],
        }

    def _nearest_active_port(self, agent_state):
        position = getattr(agent_state, "position", None)
        if position is None:
            return None
        candidate_ports = []
        for port in self._all_ports():
            anchor = self._port_anchor(port)
            if anchor is None:
                continue
            candidate_ports.append((position.distance(anchor), port))
        if not candidate_ports:
            return None
        distance, nearest = min(candidate_ports, key=lambda item: item[0])
        if distance <= self.PORT_ACTIVE_DISTANCE:
            return nearest
        return None

    def _active_port_for_state(self, agent_state):
        state_name = self._base_state_name(agent_state)
        front_zone_port = self._front_zone_port_for_state(agent_state)
        if front_zone_port is not None:
            return front_zone_port
        target_port = self._target_port_for_state(agent_state)
        if state_name in {"QUEUING", "LOADING", "UNLOADING"}:
            if target_port is not None:
                return target_port
        if state_name in {"CRUISE", "PREQUEUE"} and target_port is not None:
            if self._occupies_front_zone_footprint(agent_state, target_port):
                return target_port
        return None

    def _is_active_operation_owner(self, observed_state, port):
        if port is None:
            return False
        return self._is_operation_owner(observed_state) and self._ports_match(
            self._target_port_for_state(observed_state),
            port,
        )

    def _port_has_active_operation_owner(self, sensor_observation, port):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._is_active_operation_owner(observed_state, port):
                return True
        return False

    def _port_has_front_owner(self, sensor_observation, port):
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) == getattr(self.agent, "id", None):
                continue
            if self._front_zone_port_for_state(observed_state) == port and self._port_role_rank(observed_state, port) >= 2:
                return True
        return False

    def _front_owner_reservation_profile(self, observed_agent, port, gridmap=None, queue_nodes=None):
        observed_state = getattr(observed_agent, "userData", observed_agent)
        if port is None or gridmap is None:
            return [], set()
        queue_nodes = queue_nodes or self._queue_nodes_for_port(port, gridmap)
        front_footprint = self._front_zone_node_footprint(port, gridmap, queue_nodes)
        if not front_footprint:
            return [], set()

        current_position = getattr(observed_state, "position", None)
        current_node = agent_to_gridmap(current_position, gridmap) if current_position is not None else None
        reservation_nodes = []
        for node in self._collect_reservation_nodes(observed_agent, gridmap):
            if node in front_footprint and (not reservation_nodes or node != reservation_nodes[-1]):
                reservation_nodes.append(node)
        if not reservation_nodes and current_node in front_footprint:
            reservation_nodes.append(current_node)

        protected_nodes = set()
        if self._is_active_operation_owner(observed_state, port):
            protected_nodes.update(front_footprint)
            protected_nodes.update(self._operation_zone_node_footprint(port, gridmap, queue_nodes))
        for node in reservation_nodes:
            protected_nodes.add(node)
            for neighbor in gridmap.neighbors(node):
                if neighbor in front_footprint:
                    protected_nodes.add(neighbor)
        return reservation_nodes[: self.RESERVATION_CONFLICT_WINDOW + 1], protected_nodes

    def _apply_front_owner_entry_gate(self, dynamic_layer, gridmap, sensor_observation, port, queue_nodes=None):
        combined_protected_nodes = set()
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._front_zone_port_for_state(observed_state) != port:
                continue
            if getattr(observed_state, "id", None) == getattr(self.agent, "id", None):
                continue
            if self._port_role_rank(observed_state, port) < 2:
                continue
            reservation_nodes, protected_nodes = self._front_owner_reservation_profile(
                observed_agent,
                port,
                gridmap,
                queue_nodes,
            )
            if reservation_nodes:
                self._apply_reservation_window(
                    dynamic_layer,
                    gridmap,
                    reservation_nodes,
                    node_scale=self.FRONT_OWNER_NODE_INFLATION / self.RESERVATION_NODE_INFLATION,
                    edge_scale=self.FRONT_OWNER_EDGE_INFLATION / self.RESERVATION_EDGE_INFLATION,
                )
            combined_protected_nodes.update(protected_nodes)
        if combined_protected_nodes:
            self._mark_protected_node_entries_forbidden(gridmap, combined_protected_nodes)
            for node in combined_protected_nodes:
                self._reserve_node_entries(dynamic_layer, gridmap, node, self.FRONT_OWNER_NODE_INFLATION * 0.45)

    def _mark_protected_node_entries_forbidden(self, gridmap, protected_nodes):
        for node in protected_nodes:
            for neighbor in gridmap.neighbors(node):
                if neighbor in protected_nodes:
                    continue
                self._forbidden_transitions.add((neighbor, node))

    def _port_is_preplan_relevant(self, port, position=None, sequence_of_poses=None):
        if port is None:
            return False
        if self._ports_match(port, self._target_port_for_state(self.agent)):
            return True
        if self._ports_match(port, self._source_port_for_state(self.agent)):
            return True
        return self._port_relevance_for_self(port, position, sequence_of_poses)

    def _shares_active_port(self, lhs_state, rhs_state):
        return self._ports_match(
            self._active_port_for_state(lhs_state),
            self._active_port_for_state(rhs_state),
        )

    def _is_same_target_port_queue_or_owner(self, observed_state):
        observed_state_name = self._base_state_name(observed_state)
        if observed_state_name not in {"QUEUING", "LOADING", "UNLOADING"}:
            return False
        return self._ports_match(
            self._target_port_for_state(observed_state),
            self._target_port_for_state(self.agent),
        )

    def _point_preview(self, points):
        preview = []
        for point in list(points)[:4]:
            preview.append((round(point.x, 2), round(point.y, 2)))
        return preview

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
        blocker_distance = position.distance(blocker_position)
        blocker_base_state = self._base_state_name(blocker_state)
        tangent_a = Vector(-radial_dir.y, radial_dir.x)
        tangent_b = Vector(radial_dir.y, -radial_dir.x)
        tangent_dir = tangent_a if tangent_a.dot(goal_dir) >= tangent_b.dot(goal_dir) else tangent_b
        # When the blocker is already very close, prioritize peeling sideways
        # and opening clearance immediately instead of continuing to "cut through"
        # toward the goal for a few more frames.
        close_factor = max(0.0, min(1.0, (1.15 - blocker_distance) / 0.55))
        goal_weight = 1.0 - 0.45 * close_factor
        tangent_weight = 1.4 + 0.9 * close_factor
        radial_weight = 0.45 + 0.65 * close_factor
        blended = (
            goal_dir.scale(goal_weight)
            + tangent_dir.scale(tangent_weight)
            + radial_dir.scale(radial_weight)
        )
        try:
            return blended.normalize().scale(self.agent.cruise_speed).to_tuple()
        except Exception:
            return None

    def _same_source_backoff_command(self, position, sequence_of_poses, blocker_state):
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

        radial_dir = compute_direction(blocker_position, position)
        reverse_goal_dir = compute_direction(goal_pose, position) if goal_pose is not None else Vector(0.0, 0.0)
        blended = radial_dir.scale(1.35) + reverse_goal_dir.scale(0.95)
        try:
            return blended.normalize().scale(self.agent.cruise_speed).to_tuple()
        except Exception:
            return None

    def _port_admission_backoff_command(self, position, sequence_of_poses, blocker_state, port):
        blocker_position = getattr(blocker_state, "position", None)
        port_anchor = self._port_anchor(port) if port is not None else None
        if blocker_position is None and port_anchor is None:
            return None

        goal_pose = None
        for pose in list(sequence_of_poses)[:2]:
            if pose is not None:
                goal_pose = pose
                break
        if goal_pose is None:
            goal_pose = getattr(self.agent, "destination_location", None)

        reverse_goal_dir = compute_direction(goal_pose, position) if goal_pose is not None else Vector(0.0, 0.0)
        reverse_port_dir = compute_direction(port_anchor, position) if port_anchor is not None else Vector(0.0, 0.0)
        radial_dir = compute_direction(blocker_position, position) if blocker_position is not None else Vector(0.0, 0.0)
        blended = reverse_goal_dir.scale(1.1) + reverse_port_dir.scale(0.9) + radial_dir.scale(0.4)
        try:
            return blended.normalize().scale(self.agent.cruise_speed).to_tuple()
        except Exception:
            return None

    def _corridor_yield_backoff_command(self, position, sequence_of_poses, blocker_state):
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

        radial_dir = compute_direction(blocker_position, position)
        reverse_goal_dir = compute_direction(goal_pose, position) if goal_pose is not None else Vector(0.0, 0.0)
        tangent_a = Vector(-radial_dir.y, radial_dir.x)
        tangent_b = Vector(radial_dir.y, -radial_dir.x)
        tangent_dir = tangent_a if tangent_a.dot(reverse_goal_dir) >= tangent_b.dot(reverse_goal_dir) else tangent_b
        blended = radial_dir.scale(1.55) + tangent_dir.scale(0.9) + reverse_goal_dir.scale(0.7)
        try:
            return blended.normalize().scale(self.agent.cruise_speed).to_tuple()
        except Exception:
            return None
