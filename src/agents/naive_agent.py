# @date : 2018-07
# @author : xiaoyu.ge@dorabot.com
# @brief : Implementation of a agent (e.g., mars vheicles) using naive strategy

import json
import time
import urllib.request

from agents.agent import *
from agents.agent_state_machine import AgentState, move_if_next_slot_available
from geometry import Point, Vector, compute_direction, edge_edge_shortest_square_distance
from global_planners.layered_astar_planner import LayeredAStar
from representation.gridmap_a import GridmapWithNeighbors
from random import *

DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env"
DEBUG_FALLBACK_URL = "http://127.0.0.1:7778/event"
DEBUG_SESSION_ID = "v4-port-collision"
BASELINE_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-global-reservation.env"
BASELINE_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
BASELINE_DEBUG_SESSION_ID = "baseline-global-reservation"
GLOBAL_CHECK_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-no-double-stop.env"
GLOBAL_CHECK_DEBUG_FALLBACK_URL = "http://127.0.0.1:7780/event"
GLOBAL_CHECK_DEBUG_SESSION_ID = "global-no-double-stop"
GUI_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\gui-global-slowdown.env"
GUI_DEBUG_FALLBACK_URL = "http://127.0.0.1:7779/event"
GUI_DEBUG_SESSION_ID = "gui-global-slowdown"
SLOWDOWN_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-slowdown-cascade.env"
SLOWDOWN_DEBUG_FALLBACK_URL = "http://127.0.0.1:7778/event"
SLOWDOWN_DEBUG_SESSION_ID = "global-slowdown-cascade"
COLLISION_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-traffic-collision.env"
COLLISION_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
COLLISION_DEBUG_SESSION_ID = "baseline-traffic-collision"
COLLISION_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-traffic-collision.ndjson"
MIDMAP_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-midmap-collision.env"
MIDMAP_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
MIDMAP_DEBUG_SESSION_ID = "baseline-midmap-collision"
MIDMAP_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-midmap-collision.ndjson"
DEADLOCK_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-deadlock-gui.env"
DEADLOCK_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
DEADLOCK_DEBUG_SESSION_ID = "baseline-deadlock-gui"
DEADLOCK_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-deadlock-gui.ndjson"
MULTI_STOP_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\multi-stop-chain.env"
MULTI_STOP_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
MULTI_STOP_DEBUG_SESSION_ID = "multi-stop-chain"
MULTI_STOP_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-multi-stop-chain.ndjson"
PORT_SPEED_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\port-speed-replan.env"
PORT_SPEED_DEBUG_FALLBACK_URL = "http://127.0.0.1:7781/event"
PORT_SPEED_DEBUG_SESSION_ID = "port-speed-replan"
BASELINE_DEADLOCK_RELEASE_REASONS = {
    "baseline_same_target_approach_hold",
    "baseline_same_port_queue_hold",
    "baseline_exit_same_source_hold",
    "baseline_runtime_safety_hold",
    "baseline_runtime_safety_backoff",
}
BASELINE_DEADLOCK_PLANNER_HOLD_REASONS = {
    "baseline_same_target_approach_hold",
    "baseline_same_port_queue_hold",
    "baseline_exit_same_source_hold",
    "baseline_non_port_corridor_hold",
}
BASELINE_DEADLOCK_RELEASE_THRESHOLD = 6
BASELINE_DEADLOCK_RELEASE_COOLDOWN_STEPS = 18
BASELINE_DEADLOCK_RELEASE_GRACE_STEPS = 10
BASELINE_DEADLOCK_COMMIT_GO_STEPS = 72
BASELINE_DEADLOCK_COMMIT_YIELD_STEPS = 72
BASELINE_DEADLOCK_RELEASE_FAIL_COOLDOWN_STEPS = 36
BASELINE_DEADLOCK_CYCLE_RELEASE_THRESHOLD = 8
BASELINE_RUNTIME_SPEED_RECOVERY_RATIO = 0.82
BASELINE_RUNTIME_SPEED_RECOVERY_FAST_RATIO = 0.92
BASELINE_RUNTIME_SPEED_RECOVERY_CLEARANCE = 1.05
BASELINE_RUNTIME_WALL_MARGIN = 0.18
BASELINE_RUNTIME_ACTIVE_BACKOFF_MARGIN = 0.55
BASELINE_REPLAN_STABILIZATION_STEPS = 8

class NaiveAgent(Agent):
    _DEBUG_ENDPOINT_UNAVAILABLE = object()
    _debug_endpoint_cache = {}

    def _resolve_debug_endpoint(self, cache_key, env_path, fallback_url, fallback_session_id):
        cached = self._debug_endpoint_cache.get(cache_key, None)
        if cached is self._DEBUG_ENDPOINT_UNAVAILABLE:
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
            if session_id == "gui-global-slowdown":
                debug_enabled = True
            if session_id in {COLLISION_DEBUG_SESSION_ID, PORT_SPEED_DEBUG_SESSION_ID}:
                debug_enabled = True
            endpoint = (debug_url, session_id) if debug_enabled else self._DEBUG_ENDPOINT_UNAVAILABLE
        except Exception:
            endpoint = self._DEBUG_ENDPOINT_UNAVAILABLE

        self._debug_endpoint_cache[cache_key] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_UNAVAILABLE:
            return None
        return endpoint

    def _emit_debug_event(self, cache_key, env_path, fallback_url, fallback_session_id, hypothesis_id, location, message, data):
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
            self._debug_endpoint_cache[cache_key] = self._DEBUG_ENDPOINT_UNAVAILABLE

    def _debug_port_speed_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "port_speed_replan",
            PORT_SPEED_DEBUG_ENV_PATH,
            PORT_SPEED_DEBUG_FALLBACK_URL,
            PORT_SPEED_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _current_simulator_step(self):
        port = getattr(getattr(self, "task", None), "port", None)
        port_class = getattr(port, "__class__", None)
        return getattr(port_class, "simulator_step", None)

    def _baseline_path_signature(self, path_like):
        signature = []
        for pose in list(path_like)[:4]:
            if pose is None:
                continue
            signature.append((round(getattr(pose, "x", 0.0), 3), round(getattr(pose, "y", 0.0), 3)))
        return tuple(signature)

    def _baseline_should_skip_dynamic_replan(self):
        if not self._is_baseline_traffic_aware():
            return False
        current_step = self._current_simulator_step()
        block_until = getattr(self, "_baseline_noop_replan_block_until", None)
        if current_step is None or block_until is None:
            return False
        if current_step > block_until:
            return False
        if len(getattr(self, "sequence_of_poses", [])) == 0:
            return False
        current_signature = self._baseline_path_signature(self.sequence_of_poses)
        if len(current_signature) == 0:
            return False
        return current_signature == getattr(self, "_baseline_noop_replan_signature", None)

    def _should_debug_midmap_cruise_pair(self):
        step = self._current_simulator_step()
        return getattr(self, "id", None) in {1, 2} and step is not None and 165 <= step <= 180

    def _update_runtime_state(self, next_state):
        if self.state != next_state:
            self.state = next_state
            if self.server is not None:
                self.server.update_data(self)

    def _set_stopping_overlay(self, enabled, blocker_id=None, reason=None):
        previous_overlay = {
            "active": bool(getattr(self, "stopping_active", False)),
            "blocker_id": getattr(self, "stopping_for_agent_id", None),
            "reason": getattr(self, "stopping_reason", None),
        }
        current_step = self._current_simulator_step()
        if enabled:
            if not self.stopping_active:
                self.stopping_base_state = self.state
            self.stopping_active = True
            self.stopping_for_agent_id = blocker_id
            self.stopping_reason = reason
            last_record_step = getattr(self, "_baseline_stop_record_step", None)
            if current_step is None or current_step != last_record_step:
                if (
                    getattr(self, "_baseline_stop_record_blocker_id", None) == blocker_id
                    and getattr(self, "_baseline_stop_record_reason", None) == reason
                ):
                    self._baseline_stop_record_count = getattr(self, "_baseline_stop_record_count", 0) + 1
                else:
                    self._baseline_stop_record_count = 1
                self._baseline_stop_record_step = current_step
                self._baseline_stop_record_blocker_id = blocker_id
                self._baseline_stop_record_reason = reason
            if (
                not previous_overlay["active"]
                or previous_overlay["blocker_id"] != blocker_id
                or previous_overlay["reason"] != reason
            ):
                # #region debug-point C:overlay-enabled
                self._debug_deadlock_event(
                    "C",
                    "naive_agent.py:_set_stopping_overlay",
                    "[DEBUG] stopping overlay enabled or updated",
                    {
                        "agent_id": getattr(self, "id", None),
                        "state": getattr(getattr(self, "state", None), "name", None),
                        "previous_overlay": previous_overlay,
                        "next_overlay": {
                            "active": True,
                            "blocker_id": blocker_id,
                            "reason": reason,
                        },
                    },
                )
                # #endregion
                # #region debug-point A:overlay-transition
                self._debug_multi_stop_event(
                    "A",
                    "naive_agent.py:_set_stopping_overlay",
                    "[DEBUG] multi-stop overlay enabled or updated",
                    {
                        "agent_id": getattr(self, "id", None),
                        "state": getattr(getattr(self, "state", None), "name", None),
                        "blocker_id": blocker_id,
                        "reason": reason,
                        "previous_overlay": previous_overlay,
                        "stop_record_count": getattr(self, "_baseline_stop_record_count", 0),
                        "stopping_base_state": getattr(getattr(self, "stopping_base_state", None), "name", None),
                        "step": current_step,
                    },
                )
                # #endregion
            return
        self.stopping_active = False
        self.stopping_base_state = None
        self.stopping_for_agent_id = None
        self.stopping_reason = None
        self._baseline_stop_record_count = 0
        self._baseline_stop_record_step = current_step
        self._baseline_stop_record_blocker_id = None
        self._baseline_stop_record_reason = None
        if previous_overlay["active"]:
            # #region debug-point C:overlay-cleared
            self._debug_deadlock_event(
                "C",
                "naive_agent.py:_set_stopping_overlay",
                "[DEBUG] stopping overlay cleared",
                {
                    "agent_id": getattr(self, "id", None),
                    "state": getattr(getattr(self, "state", None), "name", None),
                    "previous_overlay": previous_overlay,
                },
            )
            # #endregion
            # #region debug-point A:overlay-cleared
            self._debug_multi_stop_event(
                "A",
                "naive_agent.py:_set_stopping_overlay",
                "[DEBUG] multi-stop overlay cleared",
                {
                    "agent_id": getattr(self, "id", None),
                    "state": getattr(getattr(self, "state", None), "name", None),
                    "previous_overlay": previous_overlay,
                    "step": current_step,
                },
            )
            # #endregion

    def _debug_v4_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "v4",
            DEBUG_ENV_PATH,
            DEBUG_FALLBACK_URL,
            DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_baseline_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "baseline",
            BASELINE_DEBUG_ENV_PATH,
            BASELINE_DEBUG_FALLBACK_URL,
            BASELINE_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_global_check_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "global_check",
            GLOBAL_CHECK_DEBUG_ENV_PATH,
            GLOBAL_CHECK_DEBUG_FALLBACK_URL,
            GLOBAL_CHECK_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )
        try:
            with open(MIDMAP_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": MIDMAP_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._emit_debug_event(
            "midmap_global_check",
            MIDMAP_DEBUG_ENV_PATH,
            MIDMAP_DEBUG_FALLBACK_URL,
            MIDMAP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_collision_event(self, hypothesis_id, location, message, data):
        try:
            with open(COLLISION_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps({
                    "sessionId": COLLISION_DEBUG_SESSION_ID,
                    "runId": "pre-fix",
                    "hypothesisId": hypothesis_id,
                    "location": location,
                    "msg": message,
                    "data": data,
                    "ts": 0,
                }, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._emit_debug_event(
            "baseline_collision",
            COLLISION_DEBUG_ENV_PATH,
            COLLISION_DEBUG_FALLBACK_URL,
            COLLISION_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )
        self._emit_debug_event(
            "midmap_collision",
            MIDMAP_DEBUG_ENV_PATH,
            MIDMAP_DEBUG_FALLBACK_URL,
            MIDMAP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_deadlock_event(self, hypothesis_id, location, message, data):
        debug_url, session_id = DEADLOCK_DEBUG_FALLBACK_URL, DEADLOCK_DEBUG_SESSION_ID
        try:
            with open(DEADLOCK_DEBUG_ENV_PATH, "r", encoding="utf-8") as env_file:
                env_content = env_file.read()
            for line in env_content.splitlines():
                if line.startswith("DEBUG_SERVER_URL="):
                    debug_url = line.split("=", 1)[1]
                elif line.startswith("DEBUG_SESSION_ID="):
                    session_id = line.split("=", 1)[1]
        except Exception:
            pass
        payload = {
            "sessionId": session_id,
            "runId": "pre-fix",
            "hypothesisId": hypothesis_id,
            "location": location,
            "msg": message,
            "data": data,
            "ts": 0,
        }
        try:
            urllib.request.urlopen(
                urllib.request.Request(
                    debug_url,
                    data=json.dumps(payload).encode(),
                    headers={"Content-Type": "application/json"},
                ),
                timeout=0.2,
            ).read()
        except Exception:
            pass
        try:
            with open(DEADLOCK_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception:
            pass

    def _debug_multi_stop_event(self, hypothesis_id, location, message, data):
        payload = {
            "sessionId": MULTI_STOP_DEBUG_SESSION_ID,
            "runId": "pre-fix",
            "hypothesisId": hypothesis_id,
            "location": location,
            "msg": message,
            "data": data,
            "ts": 0,
        }
        try:
            with open(MULTI_STOP_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                debug_file.write(json.dumps(payload, ensure_ascii=False) + "\n")
        except Exception:
            pass
        self._emit_debug_event(
            "multi_stop",
            MULTI_STOP_DEBUG_ENV_PATH,
            MULTI_STOP_DEBUG_FALLBACK_URL,
            MULTI_STOP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )

    def _is_baseline_traffic_aware(self):
        return type(getattr(self, "global_planner", None)).__name__ == "LayeredAStarBaselineTrafficAware"

    def _baseline_time_step(self):
        port = getattr(getattr(self, "task", None), "port", None)
        time_step = getattr(port, "simulator_time_step", None)
        if isinstance(time_step, (int, float)) and time_step > 0:
            return float(time_step)
        return 1.0 / 20.0

    def _vector_components(self, value):
        if value is None:
            return 0.0, 0.0
        if hasattr(value, "x") and hasattr(value, "y"):
            return float(value.x), float(value.y)
        try:
            return float(value[0]), float(value[1])
        except Exception:
            return 0.0, 0.0

    def _baseline_should_yield_runtime(self, observed_state):
        planner = getattr(self, "global_planner", None)
        if planner is not None and hasattr(planner, "_is_explicit_yielder_for_self"):
            try:
                if planner._is_explicit_yielder_for_self(observed_state):
                    return False
            except Exception:
                pass
        if planner is not None and hasattr(planner, "_should_yield_to"):
            try:
                return bool(planner._should_yield_to(observed_state))
            except Exception:
                pass
        return getattr(observed_state, "id", 0) < getattr(self, "id", 0)

    def _baseline_runtime_backoff_command(self, blocker_position):
        if self.position is None or blocker_position is None:
            return (0.0, 0.0)
        separation_dir = compute_direction(blocker_position, self.position)
        try:
            return separation_dir.normalize().scale(self.cruise_speed).to_tuple()
        except Exception:
            return (0.0, 0.0)

    def _baseline_runtime_priority_tuple(self, agent_state):
        planner = getattr(self, "global_planner", None)
        if planner is not None and hasattr(planner, "_priority_tuple"):
            try:
                return tuple(planner._priority_tuple(agent_state))
            except Exception:
                pass

        state_name = getattr(getattr(agent_state, "state", None), "name", None)
        if getattr(agent_state, "stopping_active", False):
            state_name = "STOPPING"
        score_map = {
            "STOPPING": 6,
            "LOADING": 5,
            "UNLOADING": 5,
            "QUEUING": 4,
            "PREQUEUE": 3,
            "CRUISE": 2,
        }
        return (
            score_map.get(state_name, 1),
            0,
            -getattr(agent_state, "id", 0),
        )

    def _baseline_runtime_min_square_distance(self, own_start, own_velocity, other_start, other_velocity, time_step, horizon_scales):
        predicted_min_square_distance = float("inf")
        for scale in horizon_scales:
            horizon = time_step * scale
            own_future = Point(
                own_start.x + own_velocity[0] * horizon,
                own_start.y + own_velocity[1] * horizon,
            )
            other_future = Point(
                other_start.x + other_velocity[0] * horizon,
                other_start.y + other_velocity[1] * horizon,
            )
            square_distance, _ = edge_edge_shortest_square_distance(
                own_start,
                own_future,
                other_start,
                other_future,
            )
            predicted_min_square_distance = min(predicted_min_square_distance, square_distance)
        return predicted_min_square_distance

    def _baseline_runtime_port_clearance_risk(self, command_vx, command_vy, own_radius, time_step):
        if self.position is None:
            return None
        planner = getattr(self, "global_planner", None)
        if planner is None:
            return None
        if not all(hasattr(planner, attr) for attr in ("_all_ports", "_ports_match", "_target_port_for_state")):
            return None

        target_port = planner._target_port_for_state(self)
        source_port = planner._source_port_for_state(self) if hasattr(planner, "_source_port_for_state") else None
        is_exiting_source = bool(hasattr(planner, "_is_exiting_port") and planner._is_exiting_port(self))
        own_half_dim = (own_radius, own_radius)
        future_point = Point(
            self.position.x + command_vx * time_step * 4.0,
            self.position.y + command_vy * time_step * 4.0,
        )

        best_risk = None
        for port in planner._all_ports():
            allow_port_entry = False
            if planner._ports_match(target_port, port) and getattr(getattr(self, "state", None), "name", None) in {"QUEUING", "LOADING", "UNLOADING"}:
                allow_port_entry = True
            elif is_exiting_source and planner._ports_match(source_port, port):
                allow_port_entry = True
            if allow_port_entry:
                continue

            touches_port = (
                port.is_too_close((future_point.x, future_point.y), own_half_dim)
                or port.check_obstacle_edge_collision(self.position, future_point, radius=own_radius)
            )
            if not touches_port:
                continue

            anchor = getattr(port, "operation_zone", None) or getattr(port, "center", None) or getattr(port, "location", None)
            distance = self.position.distance(anchor) if anchor is not None else 0.0
            risk_score = (distance, getattr(port, "id", getattr(port, "identifier", 0)))
            if best_risk is None or risk_score < best_risk["score"]:
                best_risk = {
                    "score": risk_score,
                    "port": port,
                    "distance": distance,
                    "anchor": anchor,
                }
        return best_risk

    def _baseline_runtime_same_port_owner_risk(self, command_vx, command_vy, own_radius, time_step):
        if self.position is None:
            return None
        planner = getattr(self, "global_planner", None)
        if planner is None:
            return None
        required_attrs = (
            "_target_port_for_state",
            "_ports_match",
            "_is_operation_owner",
        )
        if not all(hasattr(planner, attr) for attr in required_attrs):
            return None

        own_state_name = getattr(getattr(self, "state", None), "name", None)
        if own_state_name not in {"CRUISE", "PREQUEUE", "QUEUING"}:
            return None

        target_port = planner._target_port_for_state(self)
        queue = getattr(target_port, "queue", None) if target_port is not None else None
        operation_zone = getattr(target_port, "operation_zone", None) if target_port is not None else None
        if target_port is None or queue is None or operation_zone is None:
            return None

        try:
            assigned_slot = target_port.get_slot(self)
        except Exception:
            assigned_slot = None
        assigned_to_operation_zone = False
        if assigned_slot is not None:
            try:
                assigned_to_operation_zone = bool(target_port.is_at_operation_point(assigned_slot, tolerance=1e-3))
            except Exception:
                assigned_to_operation_zone = False
        if assigned_to_operation_zone:
            return None

        owner_agent = None
        for queued_agent in list(getattr(queue, "agents", []) or []):
            if getattr(queued_agent, "id", None) == getattr(self, "id", None):
                continue
            if not planner._ports_match(planner._target_port_for_state(queued_agent), target_port):
                continue
            if not planner._is_operation_owner(queued_agent):
                continue
            owner_agent = queued_agent
            break
        if owner_agent is None:
            return None

        owner_position = getattr(owner_agent, "position", None)
        if owner_position is None:
            return None

        clearance = max(0.82, own_radius * 2.0 + 0.10)
        future_point = Point(
            self.position.x + command_vx * time_step * 4.0,
            self.position.y + command_vy * time_step * 4.0,
        )
        current_distance = self.position.distance(operation_zone)
        future_distance = future_point.distance(operation_zone)
        square_distance, _ = edge_edge_shortest_square_distance(
            self.position,
            future_point,
            operation_zone,
            operation_zone,
        )
        path_min_distance = sqrt(max(0.0, square_distance))
        if min(current_distance, future_distance, path_min_distance) > clearance:
            return None

        slot_distance = assigned_slot.distance(operation_zone) if assigned_slot is not None else None
        return {
            "port": target_port,
            "owner_id": getattr(owner_agent, "id", None),
            "owner_state": getattr(getattr(owner_agent, "state", None), "name", None),
            "current_distance": current_distance,
            "future_distance": future_distance,
            "path_min_distance": path_min_distance,
            "clearance": clearance,
            "owner_position": owner_position,
            "slot_distance": slot_distance,
            "anchor": operation_zone,
        }

    def _baseline_deadlock_release_candidate(self):
        if not self._is_baseline_traffic_aware() or self.position is None:
            return None
        if not bool(getattr(self, "stopping_active", False)):
            return None

        planner = getattr(self, "global_planner", None)
        if planner is None or not hasattr(planner, "_find_observed_agent_by_id"):
            return None
        blocker_id = getattr(self, "stopping_for_agent_id", None)
        stop_reason = getattr(self, "stopping_reason", None)
        stop_count = int(getattr(self, "_baseline_stop_record_count", 0) or 0)
        current_step = self._current_simulator_step()
        last_release_step = getattr(self, "_baseline_deadlock_release_step", None)
        if blocker_id is None or stop_reason not in BASELINE_DEADLOCK_RELEASE_REASONS:
            return None
        if self._baseline_deadlock_commit_is_active(blocker_id=blocker_id):
            return None
        fail_cooldown_until = getattr(self, "_baseline_deadlock_release_fail_cooldown_until", None)
        fail_cooldown_blocker_id = getattr(self, "_baseline_deadlock_release_fail_blocker_id", None)
        if (
            current_step is not None
            and fail_cooldown_until is not None
            and current_step <= fail_cooldown_until
            and fail_cooldown_blocker_id == blocker_id
        ):
            return None
        if stop_count < BASELINE_DEADLOCK_RELEASE_THRESHOLD:
            return None
        if (
            current_step is not None
            and last_release_step is not None
            and current_step - last_release_step < BASELINE_DEADLOCK_RELEASE_COOLDOWN_STEPS
        ):
            return None

        best_candidate = None
        observed_agent = planner._find_observed_agent_by_id(self.perception_module, blocker_id)
        if observed_agent is not None:
            observed_state = getattr(observed_agent, "userData", observed_agent)
            blocker_position = getattr(observed_state, "position", None)
            if blocker_position is not None:
                current_distance = self.position.distance(blocker_position)
                has_conflict = current_distance <= 2.8
                if hasattr(planner, "_conflicts_with_blocker_state"):
                    try:
                        has_conflict = has_conflict or bool(
                            planner._conflicts_with_blocker_state(
                                self.position,
                                self.sequence_of_poses,
                                observed_state,
                                observed_agent,
                            )
                        )
                    except Exception:
                        pass
                explicit_yielder = False
                if hasattr(planner, "_is_explicit_yielder_for_self"):
                    try:
                        explicit_yielder = bool(planner._is_explicit_yielder_for_self(observed_state))
                    except Exception:
                        explicit_yielder = False
                if stop_reason == "baseline_runtime_safety_hold" or getattr(observed_state, "stopping_reason", None) == "baseline_runtime_safety_hold":
                    observed_priority = self._baseline_runtime_priority_tuple(observed_state)
                    own_priority = self._baseline_runtime_priority_tuple(self)
                    if observed_priority != own_priority:
                        should_yield = observed_priority > own_priority
                    else:
                        should_yield = getattr(observed_state, "id", 0) < getattr(self, "id", 0)
                    explicit_yielder = False
                else:
                    should_yield = False if explicit_yielder else self._baseline_should_yield_runtime(observed_state)
                if has_conflict and not should_yield:
                    best_candidate = {
                        "score": (0 if getattr(observed_state, "stopping_active", False) else 1, current_distance, blocker_id),
                        "blocker_id": blocker_id,
                        "distance": current_distance,
                        "explicit_yielder": explicit_yielder,
                        "blocker_reason": getattr(observed_state, "stopping_reason", None),
                        "stop_count": stop_count,
                    }
        # #region debug-point A:deadlock-candidate-scan
        self._debug_deadlock_event(
            "A",
            "naive_agent.py:_baseline_deadlock_release_candidate",
            "[DEBUG] deadlock release candidate scan completed",
            {
                "agent_id": getattr(self, "id", None),
                "state": getattr(getattr(self, "state", None), "name", None),
                "stopping_reason": getattr(self, "stopping_reason", None),
                "stop_count": stop_count,
                "selected_blocker_id": best_candidate["blocker_id"] if best_candidate is not None else None,
                "selected_distance": round(best_candidate["distance"], 3) if best_candidate is not None else None,
                "selected_blocker_reason": best_candidate["blocker_reason"] if best_candidate is not None else None,
            },
        )
        # #endregion
        return best_candidate

    def _baseline_deadlock_commit_is_active(self, blocker_id=None, role=None):
        current_step = self._current_simulator_step()
        commit_until = getattr(self, "deadlock_commit_until", None)
        if current_step is None or commit_until is None or current_step > commit_until:
            return False
        if blocker_id is not None and getattr(self, "deadlock_commit_blocker_id", None) != blocker_id:
            return False
        if role is not None and getattr(self, "deadlock_commit_role", None) != role:
            return False
        return True

    def _baseline_deadlock_set_commit(self, role, blocker_id, duration_steps):
        current_step = self._current_simulator_step()
        self.deadlock_commit_role = role
        self.deadlock_commit_blocker_id = blocker_id
        self.deadlock_commit_until = (
            current_step + duration_steps if current_step is not None else None
        )

    def _baseline_deadlock_set_peer_commit(self, peer_id, peer_role, peer_blocker_id, duration_steps, fail_cooldown_steps=None):
        planner = getattr(self, "global_planner", None)
        if planner is None or not hasattr(planner, "_find_observed_agent_by_id"):
            return
        peer_agent = planner._find_observed_agent_by_id(self.perception_module, peer_id)
        if peer_agent is None:
            return
        peer_state = getattr(peer_agent, "userData", peer_agent)
        current_step = self._current_simulator_step()
        setattr(peer_state, "deadlock_commit_role", peer_role)
        setattr(peer_state, "deadlock_commit_blocker_id", peer_blocker_id)
        setattr(
            peer_state,
            "deadlock_commit_until",
            current_step + duration_steps if current_step is not None else None,
        )
        if fail_cooldown_steps is not None:
            setattr(peer_state, "_baseline_deadlock_release_fail_blocker_id", peer_blocker_id)
            setattr(
                peer_state,
                "_baseline_deadlock_release_fail_cooldown_until",
                current_step + fail_cooldown_steps if current_step is not None else None,
            )

    def _baseline_deadlock_clear_commit_if_expired(self):
        current_step = self._current_simulator_step()
        commit_until = getattr(self, "deadlock_commit_until", None)
        if current_step is None or commit_until is None or current_step <= commit_until:
            return
        self.deadlock_commit_role = None
        self.deadlock_commit_blocker_id = None
        self.deadlock_commit_until = None

    def _baseline_deadlock_should_keep_path(self, avoidance_response):
        if not self._is_baseline_traffic_aware() or avoidance_response is None:
            return False
        if not avoidance_response.get("replan"):
            return False
        blocker_id = avoidance_response.get("blocker_id")
        if blocker_id is None:
            return False
        if not self._baseline_deadlock_commit_is_active(blocker_id=blocker_id, role="go"):
            return False
        if len(getattr(self, "sequence_of_poses", [])) == 0:
            return False

        reason = avoidance_response.get("reason")
        if reason not in {"predicted_collision", "exit_same_source_replan"}:
            return False

        # The release winner already owns this conflict pair for the current
        # commit window. Replanning again every step tends to bounce it between
        # near-identical paths and drags throughput down. Let the current path
        # continue and keep runtime safety as the last gate.
        return True

    def _baseline_deadlock_cycle_release_candidate(self):
        if not self._is_baseline_traffic_aware() or self.position is None:
            return None
        if not bool(getattr(self, "stopping_active", False)):
            return None
        stop_reason = getattr(self, "stopping_reason", None)
        if stop_reason not in BASELINE_DEADLOCK_RELEASE_REASONS:
            return None
        stop_count = int(getattr(self, "_baseline_stop_record_count", 0) or 0)
        if stop_count < BASELINE_DEADLOCK_CYCLE_RELEASE_THRESHOLD:
            return None

        planner = getattr(self, "global_planner", None)
        if planner is None or not hasattr(planner, "_find_observed_agent_by_id"):
            return None

        cycle_ids = []
        visited = set()
        current_state = self
        for _ in range(4):
            current_id = getattr(current_state, "id", None)
            blocker_id = getattr(current_state, "stopping_for_agent_id", None)
            if current_id is None or blocker_id is None:
                return None
            cycle_ids.append(current_id)
            visited.add(current_id)
            if blocker_id == getattr(self, "id", None):
                cycle_ids.append(blocker_id)
                break
            if blocker_id in visited:
                return None
            next_agent = planner._find_observed_agent_by_id(self.perception_module, blocker_id)
            if next_agent is None:
                return None
            next_state = getattr(next_agent, "userData", next_agent)
            if not bool(getattr(next_state, "stopping_active", False)):
                return None
            if getattr(next_state, "stopping_reason", None) not in BASELINE_DEADLOCK_RELEASE_REASONS:
                return None
            current_state = next_state

        if len(cycle_ids) < 3 or cycle_ids[-1] != getattr(self, "id", None):
            return None

        ring_ids = cycle_ids[:-1]
        winner_id = max(ring_ids)
        if winner_id != getattr(self, "id", None):
            return None
        blocker_id = getattr(self, "stopping_for_agent_id", None)
        return {
            "blocker_id": blocker_id,
            "cycle_ids": ring_ids,
            "distance": self.position.distance(getattr(current_state, "position", self.position)),
            "blocker_reason": stop_reason,
            "explicit_yielder": False,
            "stop_count": stop_count,
        }

    def _baseline_runtime_wall_risk(self, command_vx, command_vy, own_radius, time_step):
        if self.position is None or self.perception_module is None:
            return None
        future_scales = (2.0, 4.0, 6.0)
        min_distance = None
        environment = getattr(self, "static_environment", None)
        env_width = getattr(environment, "width_in_meters", None)
        env_height = getattr(environment, "height_in_meters", None)
        for scale in future_scales:
            future_point = Point(
                self.position.x + command_vx * time_step * scale,
                self.position.y + command_vy * time_step * scale,
            )
            if env_width is not None and env_height is not None:
                if not (
                    own_radius <= future_point.x <= env_width - own_radius
                    and own_radius <= future_point.y <= env_height - own_radius
                ):
                    return {
                        "distance": 0.0,
                        "anchor": future_point,
                        "reason": "bounds",
                    }
            if environment is not None and hasattr(environment, "subpath_obstacles_collision"):
                if environment.subpath_obstacles_collision(
                    self.position,
                    future_point,
                    (own_radius, own_radius),
                ):
                    return {
                        "distance": 0.0,
                        "anchor": future_point,
                        "reason": "obstacle_edge",
                    }
            for wall in self.perception_module.walls_in_range_of(max(2.0, own_radius * 6.0), 2 * pi):
                wall_state = getattr(wall, "userData", wall)
                wall_location = getattr(wall_state, "location", None)
                if wall_location is None:
                    continue
                wall_distance = future_point.distance(wall_location)
                if min_distance is None or wall_distance < min_distance:
                    min_distance = wall_distance
                if wall_distance <= own_radius + BASELINE_RUNTIME_WALL_MARGIN:
                    return {
                        "distance": wall_distance,
                        "anchor": wall_location,
                        "reason": "wall_proximity",
                    }
        return None

    def _baseline_runtime_restore_speed(self, command, runtime_hold=None, reason=None):
        if not self._is_baseline_traffic_aware() or command is None or runtime_hold is not None:
            return command
        command_vx, command_vy = self._vector_components(command)
        command_speed = sqrt(command_vx ** 2 + command_vy ** 2)
        if command_speed <= 1e-6:
            return command
        if command_speed >= self.cruise_speed * BASELINE_RUNTIME_SPEED_RECOVERY_RATIO:
            return (command_vx, command_vy)

        planner = getattr(self, "global_planner", None)
        blocker_clear = True
        if planner is not None and hasattr(planner, "_find_observed_agent_by_id"):
            blocker_id = getattr(self, "stopping_for_agent_id", None)
            if blocker_id is not None:
                blocker = planner._find_observed_agent_by_id(self.perception_module, blocker_id)
                blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
                blocker_position = getattr(blocker_state, "position", None) if blocker_state is not None else None
                if blocker_position is not None:
                    blocker_clear = self.position.distance(blocker_position) >= BASELINE_RUNTIME_SPEED_RECOVERY_CLEARANCE

        if not blocker_clear:
            # #region debug-point A:agent1-speed-recovery-blocked
            if getattr(self, "id", None) == 1:
                self._debug_port_speed_event(
                    "A",
                    "naive_agent.py:_baseline_runtime_restore_speed",
                    "[DEBUG] agent1 speed recovery stayed limited because blocker clearance was not met",
                    {
                        "agent_id": 1,
                        "state": getattr(getattr(self, "state", None), "name", None),
                        "reason": reason,
                        "command_speed": round(command_speed, 4),
                        "cruise_speed": round(getattr(self, "cruise_speed", 0.0), 4),
                        "blocker_id": getattr(self, "stopping_for_agent_id", None),
                        "clearance_threshold": BASELINE_RUNTIME_SPEED_RECOVERY_CLEARANCE,
                        "step": self._current_simulator_step(),
                    },
                )
            # #endregion
            return (command_vx, command_vy)

        desired_ratio = BASELINE_RUNTIME_SPEED_RECOVERY_RATIO
        if (
            getattr(self, "current_state", None) is not None
            and getattr(self.current_state, "name", None) in {"CRUISE", "PREQUEUE"}
            and reason not in {
                "baseline_zone_wait",
                "baseline_same_port_queue_hold",
                "baseline_non_port_corridor_hold",
            }
            and command_speed >= self.cruise_speed * 0.52
        ):
            desired_ratio = BASELINE_RUNTIME_SPEED_RECOVERY_FAST_RATIO

        desired_speed = min(self.cruise_speed, max(command_speed, self.cruise_speed * desired_ratio))
        scale = desired_speed / command_speed if command_speed > 1e-6 else 1.0
        restored_command = (command_vx * scale, command_vy * scale)
        # #region debug-point A:agent1-speed-recovery-applied
        if getattr(self, "id", None) == 1:
            self._debug_port_speed_event(
                "A",
                "naive_agent.py:_baseline_runtime_restore_speed",
                "[DEBUG] agent1 speed recovery increased final command speed",
                {
                    "agent_id": 1,
                    "state": getattr(getattr(self, "state", None), "name", None),
                    "reason": reason,
                    "command_speed_before": round(command_speed, 4),
                    "command_speed_after": round(desired_speed, 4),
                    "desired_ratio": desired_ratio,
                    "blocker_id": getattr(self, "stopping_for_agent_id", None),
                    "step": self._current_simulator_step(),
                },
            )
        # #endregion
        return restored_command

    def _baseline_runtime_safety_override(self, command, reason=None, blocker_id=None):
        if not self._is_baseline_traffic_aware() or command is None or self.position is None:
            return command, None

        command_vx, command_vy = self._vector_components(command)
        command_speed = sqrt(command_vx ** 2 + command_vy ** 2)
        if command_speed <= 1e-6:
            return (command_vx, command_vy), None

        own_radius = float(getattr(getattr(self, "shape", None), "get_radius", lambda: 0.5)() or 0.5)
        time_step = self._baseline_time_step()
        # Global safety gate: every final command must be the unique admissible
        # winner inside its short-horizon conflict set, and it must remain safe
        # even if the conflicting peer brakes immediately.
        horizon_scales = (1.0, 2.0, 4.0, 6.0, 8.0, 10.0)
        scan_radius = max(4.8, own_radius * 7.0 + command_speed * time_step * 9.0)
        best_risk = None
        own_priority = self._baseline_runtime_priority_tuple(self)

        for observed_agent in self.perception_module.other_agents_state_in_range_of(scan_radius, 2 * pi):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            observed_id = getattr(observed_state, "id", None)
            blocker_position = getattr(observed_state, "position", None)
            if observed_id is None or observed_id == getattr(self, "id", None) or blocker_position is None:
                continue

            other_radius = float(getattr(getattr(observed_state, "shape", None), "get_radius", lambda: own_radius)() or own_radius)
            safety_margin = max(0.12, 0.24 * (own_radius + other_radius))
            safety_radius = own_radius + other_radius + safety_margin
            other_vx, other_vy = (0.0, 0.0) if getattr(observed_state, "stopping_active", False) else self._vector_components(
                getattr(observed_state, "linear_velocity", None) or getattr(observed_agent, "linearVelocity", None)
            )
            other_speed = sqrt(other_vx ** 2 + other_vy ** 2)

            current_distance = self.position.distance(blocker_position)
            predicted_min_square_distance = self._baseline_runtime_min_square_distance(
                self.position,
                (command_vx, command_vy),
                blocker_position,
                (other_vx, other_vy),
                time_step,
                horizon_scales,
            )
            stopped_min_square_distance = self._baseline_runtime_min_square_distance(
                self.position,
                (command_vx, command_vy),
                blocker_position,
                (0.0, 0.0),
                time_step,
                horizon_scales,
            )

            planner = getattr(self, "global_planner", None)
            observed_explicit_yielder = False
            if planner is not None and hasattr(planner, "_is_explicit_yielder_for_self"):
                try:
                    observed_explicit_yielder = bool(planner._is_explicit_yielder_for_self(observed_state))
                except Exception:
                    observed_explicit_yielder = False
            should_yield = False if observed_explicit_yielder else self._baseline_should_yield_runtime(observed_state)
            observed_priority = self._baseline_runtime_priority_tuple(observed_state)
            hard_block = (
                getattr(observed_state, "stopping_active", False)
                or other_speed <= max(0.05, self.cruise_speed * 0.12)
                or current_distance <= safety_radius + max(0.05, own_radius * 0.12)
            )
            moving_conflict = predicted_min_square_distance <= safety_radius ** 2
            stopped_conflict = stopped_min_square_distance <= safety_radius ** 2
            effective_conflict = (
                stopped_conflict
                if not should_yield
                else (moving_conflict or stopped_conflict)
            )
            if not effective_conflict:
                continue

            risk_score = (
                0 if should_yield else 1,
                0 if stopped_conflict else 1,
                0 if hard_block else 1,
                0 if observed_id == blocker_id else 1,
                min(predicted_min_square_distance, stopped_min_square_distance),
                current_distance,
                observed_id,
            )
            if best_risk is None or risk_score < best_risk["score"]:
                best_risk = {
                    "score": risk_score,
                    "risk_type": "agent",
                    "blocker_id": observed_id,
                    "blocker_state": getattr(getattr(observed_state, "state", None), "name", None),
                    "current_distance": current_distance,
                    "predicted_min_distance": sqrt(max(0.0, predicted_min_square_distance)),
                    "stopped_min_distance": sqrt(max(0.0, stopped_min_square_distance)),
                    "safety_radius": safety_radius,
                    "hard_block": hard_block,
                    "moving_conflict": moving_conflict,
                    "stopped_conflict": stopped_conflict,
                    "should_yield": should_yield,
                    "observed_priority": list(observed_priority),
                    "own_priority": list(own_priority),
                    "blocker_position": blocker_position,
                    "command": (command_vx, command_vy),
                    "reason": reason,
                }

        same_port_owner_risk = self._baseline_runtime_same_port_owner_risk(
            command_vx,
            command_vy,
            own_radius,
            time_step,
        )
        if same_port_owner_risk is not None:
            risk_score = (
                0,
                0,
                0,
                0,
                same_port_owner_risk["path_min_distance"],
                same_port_owner_risk["current_distance"],
                same_port_owner_risk["owner_id"] if same_port_owner_risk["owner_id"] is not None else -1,
            )
            if best_risk is None or risk_score < best_risk["score"]:
                best_risk = {
                    "score": risk_score,
                    "risk_type": "same_port_owner",
                    "blocker_id": same_port_owner_risk["owner_id"],
                    "blocker_state": same_port_owner_risk["owner_state"],
                    "current_distance": same_port_owner_risk["current_distance"],
                    "predicted_min_distance": same_port_owner_risk["future_distance"],
                    "stopped_min_distance": same_port_owner_risk["path_min_distance"],
                    "safety_radius": same_port_owner_risk["clearance"],
                    "hard_block": True,
                    "moving_conflict": True,
                    "stopped_conflict": True,
                    "should_yield": True,
                    "observed_priority": None,
                    "own_priority": list(own_priority),
                    "blocker_position": same_port_owner_risk["owner_position"],
                    "reason": reason,
                    "port_id": getattr(same_port_owner_risk["port"], "id", getattr(same_port_owner_risk["port"], "identifier", None)),
                    "port_type": getattr(same_port_owner_risk["port"], "port_type", None),
                    "slot_distance": round(same_port_owner_risk["slot_distance"], 3) if same_port_owner_risk["slot_distance"] is not None else None,
                }

        static_port_risk = self._baseline_runtime_port_clearance_risk(
            command_vx,
            command_vy,
            own_radius,
            time_step,
        )
        if static_port_risk is not None:
            risk_score = (
                0,
                0,
                0,
                1,
                static_port_risk["distance"],
                getattr(static_port_risk["port"], "id", getattr(static_port_risk["port"], "identifier", 0)),
            )
            if best_risk is None or risk_score < best_risk["score"]:
                best_risk = {
                    "score": risk_score,
                    "risk_type": "port",
                    "blocker_id": None,
                    "blocker_state": "PORT",
                    "current_distance": static_port_risk["distance"],
                    "predicted_min_distance": 0.0,
                    "stopped_min_distance": 0.0,
                    "safety_radius": own_radius,
                    "hard_block": True,
                    "moving_conflict": True,
                    "stopped_conflict": True,
                    "should_yield": True,
                    "observed_priority": None,
                    "own_priority": list(own_priority),
                    "blocker_position": static_port_risk["anchor"],
                    "reason": reason,
                    "port_id": getattr(static_port_risk["port"], "id", getattr(static_port_risk["port"], "identifier", None)),
                    "port_type": getattr(static_port_risk["port"], "port_type", None),
                }

        wall_risk = self._baseline_runtime_wall_risk(
            command_vx,
            command_vy,
            own_radius,
            time_step,
        )
        if wall_risk is not None:
            risk_score = (
                0,
                0,
                0,
                0,
                wall_risk["distance"],
                -1,
            )
            if best_risk is None or risk_score < best_risk["score"]:
                best_risk = {
                    "score": risk_score,
                    "risk_type": "wall",
                    "blocker_id": None,
                    "blocker_state": "WALL",
                    "current_distance": wall_risk["distance"],
                    "predicted_min_distance": wall_risk["distance"],
                    "stopped_min_distance": wall_risk["distance"],
                    "safety_radius": own_radius + BASELINE_RUNTIME_WALL_MARGIN,
                    "hard_block": True,
                    "moving_conflict": True,
                    "stopped_conflict": True,
                    "should_yield": True,
                    "observed_priority": None,
                    "own_priority": list(own_priority),
                    "blocker_position": wall_risk["anchor"],
                    "reason": reason,
                    "wall_reason": wall_risk["reason"],
                }

        if best_risk is None:
            return (command_vx, command_vy), None

        current_step = self._current_simulator_step()
        release_grace_until = getattr(self, "_baseline_deadlock_release_grace_until", None)
        release_grace_blocker_id = getattr(self, "_baseline_deadlock_release_grace_blocker_id", None)
        observed_commit_role = None
        if (
            best_risk["risk_type"] == "agent"
            and getattr(self, "global_planner", None) is not None
            and hasattr(self.global_planner, "_find_observed_agent_by_id")
        ):
            observed_agent = self.global_planner._find_observed_agent_by_id(self.perception_module, best_risk["blocker_id"])
            if observed_agent is not None:
                observed_state = getattr(observed_agent, "userData", observed_agent)
                observed_commit_role = getattr(observed_state, "deadlock_commit_role", None)
        if (
            best_risk["risk_type"] == "agent"
            and current_step is not None
            and release_grace_until is not None
            and current_step <= release_grace_until
            and release_grace_blocker_id == best_risk["blocker_id"]
            and self._baseline_deadlock_commit_is_active(best_risk["blocker_id"], "go")
            and observed_commit_role == "yield"
            and not best_risk["should_yield"]
            and (
                (not best_risk["moving_conflict"] and best_risk["current_distance"] > best_risk["safety_radius"] * 0.98)
                or best_risk["current_distance"] > best_risk["safety_radius"] + 0.02
            )
        ):
            # #region debug-point E:release-grace-pass
            self._debug_deadlock_event(
                "E",
                "naive_agent.py:_baseline_runtime_safety_override",
                "[DEBUG] anti-deadlock release grace allowed winner to keep moving past blocker",
                {
                    "agent_id": getattr(self, "id", None),
                    "blocker_id": best_risk["blocker_id"],
                    "reason_before": reason,
                    "current_distance": round(best_risk["current_distance"], 3),
                    "safety_radius": round(best_risk["safety_radius"], 3),
                    "grace_until": release_grace_until,
                    "step": current_step,
                },
            )
            # #endregion
            return (command_vx, command_vy), None

        if (
            best_risk["risk_type"] == "agent"
            and reason is None
            and not best_risk["should_yield"]
        ):
            return (command_vx, command_vy), None

        hold_reason = "baseline_runtime_safety_hold"
        safe_command = (0.0, 0.0)
        prefer_active_backoff = (
            best_risk["risk_type"] == "agent"
            and reason in {
                "baseline_direct_bypass",
                "baseline_exit_same_source_backoff",
                "baseline_non_port_corridor_backoff",
                "baseline_port_admission_backoff",
            }
            and best_risk["predicted_min_distance"] <= best_risk["safety_radius"]
            and best_risk["current_distance"] <= best_risk["safety_radius"] + BASELINE_RUNTIME_ACTIVE_BACKOFF_MARGIN
        )
        if best_risk["current_distance"] <= best_risk["safety_radius"] or prefer_active_backoff:
            hold_reason = "baseline_runtime_safety_backoff"
            safe_command = self._baseline_runtime_backoff_command(best_risk["blocker_position"])

        # #region debug-point C:agent1-runtime-safety-override
        if getattr(self, "id", None) == 1:
            safe_vx, safe_vy = self._vector_components(safe_command)
            self._debug_port_speed_event(
                "C",
                "naive_agent.py:_baseline_runtime_safety_override",
                "[DEBUG] agent1 runtime safety overrode the planner command",
                {
                    "agent_id": 1,
                    "state": getattr(getattr(self, "state", None), "name", None),
                    "planner_reason": reason,
                    "runtime_reason": hold_reason,
                    "risk_type": best_risk.get("risk_type"),
                    "risk_blocker_id": best_risk.get("blocker_id"),
                    "current_distance": round(best_risk.get("current_distance", 0.0), 4),
                    "predicted_min_distance": round(best_risk.get("predicted_min_distance", 0.0), 4),
                    "safety_radius": round(best_risk.get("safety_radius", 0.0), 4),
                    "input_speed": round(command_speed, 4),
                    "output_speed": round(sqrt(safe_vx ** 2 + safe_vy ** 2), 4),
                    "step": self._current_simulator_step(),
                },
            )
        # #endregion

        # #region debug-point B:runtime-safety-hold
        self._debug_multi_stop_event(
            "B",
            "naive_agent.py:_baseline_runtime_safety_override",
            "[DEBUG] multi-stop runtime safety override produced fallback",
            {
                "agent_id": getattr(self, "id", None),
                "state": getattr(getattr(self, "state", None), "name", None),
                "reason_before": reason,
                "input_blocker_id": blocker_id,
                "risk_type": best_risk["risk_type"],
                "risk_blocker_id": best_risk["blocker_id"],
                "risk_blocker_state": best_risk["blocker_state"],
                "should_yield": best_risk["should_yield"],
                "current_distance": round(best_risk["current_distance"], 3),
                "predicted_min_distance": round(best_risk["predicted_min_distance"], 3),
                "stopped_min_distance": round(best_risk["stopped_min_distance"], 3),
                "safety_radius": round(best_risk["safety_radius"], 3),
                "hold_reason": hold_reason,
                "safe_command": [round(safe_command[0], 3), round(safe_command[1], 3)],
                "step": current_step,
            },
        )
        # #endregion

        self._debug_collision_event(
            "F",
            "naive_agent.py:_baseline_runtime_safety_override",
            "[DEBUG] baseline global safety gate converted unsafe final command into safe local fallback",
            {
                "agent_id": getattr(self, "id", None),
                "state": getattr(getattr(self, "state", None), "name", None),
                "reason_before": reason,
                "blocker_id_before": blocker_id,
                "risk_type": best_risk["risk_type"],
                "risk_blocker_id": best_risk["blocker_id"],
                "risk_blocker_state": best_risk["blocker_state"],
                "risk_port_id": best_risk.get("port_id"),
                "risk_port_type": best_risk.get("port_type"),
                "current_distance": round(best_risk["current_distance"], 3),
                "predicted_min_distance": round(best_risk["predicted_min_distance"], 3),
                "stopped_min_distance": round(best_risk["stopped_min_distance"], 3),
                "safety_radius": round(best_risk["safety_radius"], 3),
                "hard_block": best_risk["hard_block"],
                "moving_conflict": best_risk["moving_conflict"],
                "stopped_conflict": best_risk["stopped_conflict"],
                "should_yield": best_risk["should_yield"],
                "own_priority": best_risk["own_priority"],
                "observed_priority": best_risk.get("observed_priority"),
                "command": [round(command_vx, 3), round(command_vy, 3)],
                "safe_command": [round(safe_command[0], 3), round(safe_command[1], 3)],
                "reason_after": hold_reason,
            },
        )
        return safe_command, {
            "reason": hold_reason,
            "blocker_id": best_risk["blocker_id"],
        }

    def act(self):
        pass

    def observe(self, observe):
        """ Observation
        Obtain updated map (if necessary)
        Obtain sensor data
        Obtain server commands (high-priority command such as emergent halt)
        """
        localization = self.perception_module.localization()
        self.position = Point(localization[0], localization[1])
        self.angle = localization[2]
        self.pose = localization
        self.ray_length_list = observe
        self.potential_collision = False
        self.server_command = None
        if len(self.sequence_of_poses)>0 and isinstance(self.global_planner, LayeredAStar):
            self.replan = self.global_planner.observe_path(self.static_environment, self.position, self.perception_module, self.sequence_of_poses)
            if getattr(self, "enable_stuck_recovery", False) and self.replan and self.destination_location is not None:
                near_queue_goal = (
                    getattr(getattr(self, "state", None), "name", None) in {"QUEUING", "PREQUEUE"}
                    and len(self.sequence_of_poses) <= 1
                    and self.position.distance(self.destination_location) < getattr(self, "queue_goal_replan_guard_distance", 0.8)
                )
                if near_queue_goal:
                    self.replan = False
        self._baseline_deadlock_clear_commit_if_expired()
        self._baseline_deadlock_release_blocker_id = None
        if self._is_baseline_traffic_aware():
            release_candidate = self._baseline_deadlock_release_candidate()
            if release_candidate is None:
                release_candidate = self._baseline_deadlock_cycle_release_candidate()
            if release_candidate is not None:
                self._baseline_deadlock_release_blocker_id = release_candidate["blocker_id"]
                self._baseline_deadlock_release_step = self._current_simulator_step()
                if self._baseline_deadlock_release_step is not None:
                    self._baseline_deadlock_release_grace_until = (
                        self._baseline_deadlock_release_step + BASELINE_DEADLOCK_RELEASE_GRACE_STEPS
                    )
                else:
                    self._baseline_deadlock_release_grace_until = None
                self._baseline_deadlock_release_grace_blocker_id = release_candidate["blocker_id"]
                self.replan = True
                # #region debug-point B:deadlock-release-elected
                self._debug_collision_event(
                    "F",
                    "naive_agent.py:observe",
                    "[DEBUG] baseline anti-deadlock gate elected this stopped agent to resume with forced bypass replan",
                    {
                        "agent_id": getattr(self, "id", None),
                        "state": getattr(getattr(self, "state", None), "name", None),
                        "release_blocker_id": release_candidate["blocker_id"],
                        "release_blocker_distance": round(release_candidate["distance"], 3),
                        "release_blocker_reason": release_candidate["blocker_reason"],
                        "explicit_yielder": release_candidate["explicit_yielder"],
                        "cycle_ids": release_candidate.get("cycle_ids"),
                    },
                )
                self._debug_deadlock_event(
                    "B",
                    "naive_agent.py:observe",
                    "[DEBUG] anti-deadlock gate elected release candidate",
                    {
                        "agent_id": getattr(self, "id", None),
                        "release_blocker_id": release_candidate["blocker_id"],
                        "release_blocker_distance": round(release_candidate["distance"], 3),
                        "release_blocker_reason": release_candidate["blocker_reason"],
                        "explicit_yielder": release_candidate["explicit_yielder"],
                        "replan_flag": bool(self.replan),
                        "stop_count": release_candidate.get("stop_count"),
                        "grace_until": self._baseline_deadlock_release_grace_until,
                        "cycle_ids": release_candidate.get("cycle_ids"),
                    },
                )
                # #endregion
    def plan(self):
        """ Do high level state-transitions """
        previous_overlay = {
            "active": bool(getattr(self, "stopping_active", False)),
            "base_state": getattr(getattr(self, "stopping_base_state", None), "name", None),
            "blocker_id": getattr(self, "stopping_for_agent_id", None),
            "reason": getattr(self, "stopping_reason", None),
        }
        state_before_transition = getattr(getattr(self, "state", None), "name", None)
        func = self.state_machine.next_state(self)
        func(self, self.server)
        if previous_overlay["active"] and previous_overlay["reason"] == "port_corridor_yield":
            # #region debug-point F:state-machine-after-port-hold
            self._debug_v4_event(
                "F",
                "naive_agent.py:plan:state-machine-after-port-hold",
                "[DEBUG] state machine ran while previous port corridor hold overlay was active",
                {
                    "agent_id": getattr(self, "id", None),
                    "step": self._current_simulator_step(),
                    "transition": getattr(func, "__name__", None),
                    "state_before": state_before_transition,
                    "state_after": getattr(getattr(self, "state", None), "name", None),
                    "overlay_before": previous_overlay,
                    "position": [
                        round(getattr(self.position, "x", 0.0), 3),
                        round(getattr(self.position, "y", 0.0), 3),
                    ] if self.position is not None else None,
                    "destination": [
                        round(getattr(self.destination_location, "x", 0.0), 3),
                        round(getattr(self.destination_location, "y", 0.0), 3),
                    ] if self.destination_location is not None else None,
                },
            )
            # #endregion
        if self.state == AgentState.LOADING:
            self._set_stopping_overlay(False)
            self.stop()
            return
        if self.has_destination():
            if self.state_machine.arrive_at_destination(self.position,self.destination_location)==False\
                or self.goal_changed == True or self.replan == True:
                """ Run global planner """
                goal_pose = self.task.destination_location
                if self.goal_changed == True or self.replan == True:
                    self.destination_location = self.task.destination_location
                    goal_pose = self.task.destination_location
                    replan_reason = None
                    if self.goal_changed:
                        replan_reason = "goal_changed"
                    elif self.replan:
                        replan_reason = "dynamic_replan"
                    if replan_reason == "dynamic_replan" and self._baseline_should_skip_dynamic_replan():
                        # #region debug-point B:agent1-dynamic-replan-skipped
                        if getattr(self, "id", None) == 1:
                            self._debug_port_speed_event(
                                "B",
                                "naive_agent.py:plan:dynamic-replan-skipped",
                                "[DEBUG] baseline stabilization guard skipped repeated dynamic replan for agent1",
                                {
                                    "agent_id": 1,
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "overlay_active": bool(getattr(self, "stopping_active", False)),
                                    "overlay_reason": getattr(self, "stopping_reason", None),
                                    "overlay_blocker_id": getattr(self, "stopping_for_agent_id", None),
                                    "path_preview": self.global_planner._point_preview(self.sequence_of_poses) if self.global_planner is not None and hasattr(self.global_planner, "_point_preview") else [],
                                    "last_replan_step": getattr(self, "_baseline_last_dynamic_replan_step", None),
                                    "step": self._current_simulator_step(),
                                },
                            )
                        # #endregion
                        self.replan = False
                        replan_reason = None
                    skip_global_replan = replan_reason is None and not self.goal_changed
                    """ If there is a global planner module """
                    if skip_global_replan:
                        pass
                    elif self.global_planner != None:
                        self.global_plan_calls += 1
                        if self.replan:
                            self.replan_events += 1
                        self.last_replan_reason = replan_reason
                        path_signature_before = self._baseline_path_signature(self.sequence_of_poses)
                        path_preview_before = self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else []
                        # #region debug-point B:agent1-dynamic-replan-start
                        if getattr(self, "id", None) == 1 and replan_reason == "dynamic_replan":
                            self._debug_port_speed_event(
                                "B",
                                "naive_agent.py:plan:dynamic-replan-start",
                                "[DEBUG] agent1 started a dynamic global replan",
                                {
                                    "agent_id": 1,
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "destination": [
                                        round(getattr(self.destination_location, "x", 0.0), 3),
                                        round(getattr(self.destination_location, "y", 0.0), 3),
                                    ] if self.destination_location is not None else None,
                                    "overlay_active": bool(getattr(self, "stopping_active", False)),
                                    "overlay_reason": getattr(self, "stopping_reason", None),
                                    "overlay_blocker_id": getattr(self, "stopping_for_agent_id", None),
                                    "path_preview_before": path_preview_before,
                                    "step": self._current_simulator_step(),
                                },
                            )
                        # #endregion
                        _dbg_global_plan_start = time.perf_counter()
                        forced_release_bypass = (
                            getattr(self, "_baseline_deadlock_release_blocker_id", None) is not None
                            and hasattr(self.global_planner, "begin_forced_bypass")
                            and hasattr(self.global_planner, "end_forced_bypass")
                        )
                        if forced_release_bypass:
                            # #region debug-point D:forced-release-begin
                            self._debug_deadlock_event(
                                "D",
                                "naive_agent.py:plan",
                                "[DEBUG] forced bypass replan started for anti-deadlock release",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "blocker_id": self._baseline_deadlock_release_blocker_id,
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "destination": [
                                        round(getattr(self.destination_location, "x", 0.0), 3),
                                        round(getattr(self.destination_location, "y", 0.0), 3),
                                    ] if self.destination_location is not None else None,
                                },
                            )
                            # #endregion
                            self.global_planner.begin_forced_bypass(self._baseline_deadlock_release_blocker_id)
                        try:
                            temp_sequence_of_poses = self.global_planner.compute_path(
                                    self.position,
                                    self.destination_location,
                                    self.static_environment,
                                    self.perception_module)
                        finally:
                            if forced_release_bypass:
                                path_found_for_release = temp_sequence_of_poses is not None
                                release_blocker_id = self._baseline_deadlock_release_blocker_id
                                self.global_planner.end_forced_bypass()
                                self._baseline_deadlock_release_blocker_id = None
                                if path_found_for_release:
                                    self._baseline_deadlock_set_commit(
                                        "go",
                                        release_blocker_id,
                                        BASELINE_DEADLOCK_COMMIT_GO_STEPS,
                                    )
                                    self._baseline_deadlock_set_peer_commit(
                                        release_blocker_id,
                                        "yield",
                                        getattr(self, "id", None),
                                        BASELINE_DEADLOCK_COMMIT_YIELD_STEPS,
                                        fail_cooldown_steps=BASELINE_DEADLOCK_RELEASE_FAIL_COOLDOWN_STEPS,
                                    )
                                    self._baseline_deadlock_release_grace_until = getattr(self, "deadlock_commit_until", None)
                                    self._baseline_deadlock_release_fail_cooldown_until = None
                                    self._baseline_deadlock_release_fail_blocker_id = None
                                else:
                                    self._baseline_deadlock_set_commit(
                                        "yield",
                                        release_blocker_id,
                                        BASELINE_DEADLOCK_COMMIT_YIELD_STEPS,
                                    )
                                    current_release_step = self._current_simulator_step()
                                    self._baseline_deadlock_release_fail_blocker_id = release_blocker_id
                                    self._baseline_deadlock_release_fail_cooldown_until = (
                                        current_release_step + BASELINE_DEADLOCK_RELEASE_FAIL_COOLDOWN_STEPS
                                        if current_release_step is not None else None
                                    )
                                # #region debug-point D:forced-release-end
                                self._debug_deadlock_event(
                                    "D",
                                    "naive_agent.py:plan",
                                    "[DEBUG] forced bypass replan finished for anti-deadlock release",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "path_found": path_found_for_release,
                                        "commit_role": getattr(self, "deadlock_commit_role", None),
                                        "commit_until": getattr(self, "deadlock_commit_until", None),
                                        "path_head": [
                                            [round(p.x, 3), round(p.y, 3)]
                                            for p in list(temp_sequence_of_poses)[:4]
                                        ] if temp_sequence_of_poses is not None else None,
                                    },
                                )
                                # #endregion
                        _dbg_global_plan_seconds = round(time.perf_counter() - _dbg_global_plan_start, 4)
                        if replan_reason == "dynamic_replan" and _dbg_global_plan_seconds >= 0.05:
                            self._emit_debug_event(
                                "gui_slowdown",
                                GUI_DEBUG_ENV_PATH,
                                GUI_DEBUG_FALLBACK_URL,
                                GUI_DEBUG_SESSION_ID,
                                "A",
                                "naive_agent.py:plan:global-replan",
                                "[DEBUG] GUI slowdown session measured dynamic global replan",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "global_plan_seconds": _dbg_global_plan_seconds,
                                    "destination": [
                                        round(getattr(self.destination_location, "x", 0.0), 3),
                                        round(getattr(self.destination_location, "y", 0.0), 3),
                                    ] if self.destination_location is not None else None,
                                    "path_preview_before": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                                },
                            )
                        if temp_sequence_of_poses != None:
                            self.sequence_of_poses = temp_sequence_of_poses
                        else:
                            self.sequence_of_poses = deque([goal_pose])
                        if replan_reason == "dynamic_replan" and temp_sequence_of_poses != None:
                            current_step = self._current_simulator_step()
                            new_signature = self._baseline_path_signature(self.sequence_of_poses)
                            self._baseline_last_dynamic_replan_step = current_step
                            self._baseline_last_dynamic_replan_signature = new_signature
                            if new_signature == path_signature_before and len(new_signature) > 0:
                                self._baseline_noop_replan_signature = new_signature
                                self._baseline_noop_replan_block_until = (
                                    current_step + BASELINE_REPLAN_STABILIZATION_STEPS
                                    if current_step is not None else None
                                )
                            else:
                                self._baseline_noop_replan_signature = None
                                self._baseline_noop_replan_block_until = None
                        # #region debug-point B:agent1-dynamic-replan-finish
                        if getattr(self, "id", None) == 1 and replan_reason == "dynamic_replan":
                            path_preview_after = self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else []
                            self._debug_port_speed_event(
                                "B",
                                "naive_agent.py:plan:dynamic-replan-finish",
                                "[DEBUG] agent1 finished a dynamic global replan",
                                {
                                    "agent_id": 1,
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "path_found": temp_sequence_of_poses is not None,
                                    "path_preview_before": path_preview_before,
                                    "path_preview_after": path_preview_after,
                                    "replan_reason": replan_reason,
                                    "global_plan_calls": self.global_plan_calls,
                                    "replan_events": self.replan_events,
                                    "step": self._current_simulator_step(),
                                },
                            )
                        # #endregion
                    elif not skip_global_replan:
                        self.sequence_of_poses = deque([goal_pose])
                    self.goal_changed = False
                    """ Run local local_planner
                    Skip if there is no goal pose
                    """
                if len(self.sequence_of_poses) == 0:
                    # raise Exception(" No next pose while in mobile state ")
                    self.sequence_of_poses = deque([goal_pose])
                    return
                
                if self.position.distance(self.destination_location) < min(self.ray_length_list):
                    self.current_local_planner = self.local_planner[0]
                else:
                    index = randrange(0,2)
                    self.current_local_planner = self.local_planner[0]#[index]
                avoidance_response = None
                avoidance_debug = {
                    "replan_failed": None,
                    "path_still_conflicts": None,
                    "extended_bottom_port_hold": None,
                    "force_stop_after_failed_replan": None,
                    "blocker_distance": None,
                    "stop_distance": None,
                }
                if self.state in {AgentState.CRUISE, AgentState.PREQUEUE, AgentState.QUEUING} and self.global_planner is not None and hasattr(self.global_planner, "compute_avoidance_response"):
                    avoidance_response = self.global_planner.compute_avoidance_response(
                        self.position,
                        self.perception_module,
                        self.sequence_of_poses,
                    )
                    if avoidance_response is not None:
                        # #region debug-point E:planner-hold-applied
                        if avoidance_response.get("command") == (0.0, 0.0):
                            self._debug_multi_stop_event(
                                "E",
                                "naive_agent.py:plan",
                                "[DEBUG] multi-stop planner produced zero-command hold",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "reason": avoidance_response.get("reason"),
                                    "blocker_id": avoidance_response.get("blocker_id"),
                                    "replan": bool(avoidance_response.get("replan")),
                                    "allow_stop_fallback": avoidance_response.get("allow_stop_fallback", True),
                                    "step": self._current_simulator_step(),
                                },
                            )
                        # #endregion
                        if (
                            self._baseline_deadlock_commit_is_active(
                                blocker_id=avoidance_response.get("blocker_id"),
                                role="go",
                            )
                            and avoidance_response.get("reason") in BASELINE_DEADLOCK_PLANNER_HOLD_REASONS
                        ):
                            self._debug_deadlock_event(
                                "E",
                                "naive_agent.py:plan",
                                "[DEBUG] anti-deadlock go-commit ignored planner hold against committed blocker",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "blocker_id": avoidance_response.get("blocker_id"),
                                    "reason": avoidance_response.get("reason"),
                                    "commit_until": getattr(self, "deadlock_commit_until", None),
                                },
                            )
                            avoidance_response = None
                    if self._baseline_deadlock_should_keep_path(avoidance_response):
                        self._debug_deadlock_event(
                            "E",
                            "naive_agent.py:plan",
                            "[DEBUG] anti-deadlock go-commit kept winner on current path and skipped repeated avoidance replan",
                            {
                                "agent_id": getattr(self, "id", None),
                                "blocker_id": avoidance_response.get("blocker_id"),
                                "reason": avoidance_response.get("reason"),
                                "commit_until": getattr(self, "deadlock_commit_until", None),
                                "path_head": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                            },
                        )
                        avoidance_response = {
                            "reason": "baseline_deadlock_commit_keep_path",
                            "blocker_id": avoidance_response.get("blocker_id"),
                        }
                    if avoidance_response is not None:
                        if avoidance_response.get("replan"):
                            _dbg_avoidance_plan_start = time.perf_counter()
                            blocker_id = avoidance_response.get("blocker_id")
                            forced_bypass = False
                            if (
                                (
                                    avoidance_response.get("reason") == "predicted_collision"
                                    or avoidance_response.get("force_blocker_barrier")
                                )
                                and blocker_id is not None
                                and not avoidance_response.get("allow_stop_fallback", True)
                                and hasattr(self.global_planner, "begin_forced_bypass")
                                and hasattr(self.global_planner, "end_forced_bypass")
                            ):
                                # 当前 agent 已经是这组会车里的优先通过侧；
                                # 在这次重规划里显式把 blocker 当成局部障碍，逼出绕行动作。
                                self.global_planner.begin_forced_bypass(blocker_id)
                                forced_bypass = True
                            try:
                                temp_sequence_of_poses = self.global_planner.compute_path(
                                    self.position,
                                    self.destination_location,
                                    self.static_environment,
                                    self.perception_module)
                            finally:
                                if forced_bypass:
                                    self.global_planner.end_forced_bypass()
                            _dbg_avoidance_plan_seconds = round(time.perf_counter() - _dbg_avoidance_plan_start, 4)
                            replan_failed = temp_sequence_of_poses is None
                            if temp_sequence_of_poses != None:
                                self.sequence_of_poses = temp_sequence_of_poses
                            blocker_distance = None
                            stop_distance = avoidance_response.get("stop_distance")
                            allow_stop_fallback = avoidance_response.get("allow_stop_fallback", True)
                            if stop_distance is not None and hasattr(self.global_planner, "_distance_to_blocker"):
                                blocker_distance = self.global_planner._distance_to_blocker(
                                    self.position,
                                    self.perception_module,
                                    blocker_id,
                                )

                            path_still_conflicts = (
                                hasattr(self.global_planner, "path_conflicts_with_blocker") and
                                self.global_planner.path_conflicts_with_blocker(
                                    self.position,
                                    self.sequence_of_poses,
                                    self.perception_module,
                                    blocker_id,
                                )
                            )

                            force_stop_after_failed_replan = (
                                replan_failed and
                                allow_stop_fallback and
                                stop_distance is not None and
                                blocker_distance is not None and
                                blocker_distance <= stop_distance
                            )
                            if (
                                avoidance_response.get("reason") == "exit_queue_owner_avoid"
                                and not path_still_conflicts
                            ):
                                # exiting owner 的当前路径已经不再压向 blocker 时，
                                # 不能仅因为 replan 失败就一直停成双向互锁。
                                force_stop_after_failed_replan = False
                            if (
                                avoidance_response.get("suppress_stop_when_path_clear")
                                and not path_still_conflicts
                            ):
                                force_stop_after_failed_replan = False
                            blocker_state = None
                            if blocker_id is not None and hasattr(self.global_planner, "_find_observed_agent_by_id"):
                                try:
                                    blocker = self.global_planner._find_observed_agent_by_id(self.perception_module, blocker_id)
                                    blocker_state = getattr(blocker, "userData", blocker) if blocker is not None else None
                                except Exception:
                                    blocker_state = None
                            if (
                                avoidance_response.get("reason") == "predicted_collision"
                                and blocker_state is not None
                                and hasattr(self.global_planner, "_should_yield_to")
                                and not self.global_planner._should_yield_to(blocker_state)
                            ):
                                # 当前 agent 在这组 predicted_collision 里是优先通过的一侧；
                                # 不能因为 replan 失败就又把自己也停下来，否则会重新变成双边减速。
                                avoidance_response["allow_stop_fallback"] = False
                                force_stop_after_failed_replan = False
                            extended_bottom_port_hold = False
                            if (
                                avoidance_response.get("reason") == "port_corridor_yield"
                                and not path_still_conflicts
                                and not force_stop_after_failed_replan
                                and hasattr(self.global_planner, "should_extend_bottom_port_corridor_hold")
                            ):
                                extended_bottom_port_hold = self.global_planner.should_extend_bottom_port_corridor_hold(
                                    self.position,
                                    self.perception_module,
                                    self.sequence_of_poses,
                                    blocker_id,
                                )
                                if extended_bottom_port_hold:
                                    path_still_conflicts = True
                            avoidance_debug = {
                                "replan_failed": replan_failed,
                                "path_still_conflicts": path_still_conflicts,
                                "extended_bottom_port_hold": extended_bottom_port_hold,
                                "force_stop_after_failed_replan": force_stop_after_failed_replan,
                                "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                            }
                            if avoidance_response.get("reason") in {"port_corridor_yield", "exit_queue_owner_avoid"}:
                                # #region debug-point E:avoidance-replan-eval
                                self._debug_v4_event(
                                    "E",
                                    "naive_agent.py:plan:avoidance-replan-eval",
                                    "[DEBUG] v4 evaluated avoidance replan",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "reason": avoidance_response.get("reason"),
                                        "blocker_id": blocker_id,
                                        "replan_failed": replan_failed,
                                        "path_still_conflicts": path_still_conflicts,
                                        "extended_bottom_port_hold": extended_bottom_port_hold,
                                        "force_stop_after_failed_replan": force_stop_after_failed_replan,
                                        "allow_stop_fallback": allow_stop_fallback,
                                        "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                        "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                        "path_preview": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                                    },
                                )
                                # #endregion

                            if path_still_conflicts or force_stop_after_failed_replan:
                                allow_stop_fallback = avoidance_response.get("allow_stop_fallback", True)
                                should_stop = allow_stop_fallback
                                if extended_bottom_port_hold:
                                    should_stop = True
                                if allow_stop_fallback and stop_distance is not None and blocker_distance is not None:
                                    should_stop = blocker_distance <= stop_distance
                                if extended_bottom_port_hold:
                                    should_stop = True
                                if should_stop:
                                    baseline_local_hold = type(self.global_planner).__name__ == "LayeredAStarBaselineTrafficAware"
                                    # #region debug-point D:baseline-stop-fallback
                                    if baseline_local_hold:
                                        self._debug_baseline_event(
                                            "D",
                                            "naive_agent.py:plan",
                                            "[DEBUG] baseline runtime escalated to stop fallback",
                                            {
                                                "agent_id": getattr(self, "id", None),
                                                "state": getattr(getattr(self, "state", None), "name", None),
                                                "blocker_id": blocker_id,
                                                "reason": avoidance_response.get("reason"),
                                                "path_still_conflicts": path_still_conflicts,
                                                "replan_failed": replan_failed,
                                                "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                                "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                            },
                                        )
                                        # #region debug-point D:baseline-collision-stop-fallback
                                        self._debug_collision_event(
                                            "D",
                                            "naive_agent.py:plan",
                                            "[DEBUG] baseline runtime executed stop fallback",
                                            {
                                                "agent_id": getattr(self, "id", None),
                                                "state": getattr(getattr(self, "state", None), "name", None),
                                                "blocker_id": blocker_id,
                                                "reason": avoidance_response.get("reason"),
                                                "path_still_conflicts": path_still_conflicts,
                                                "replan_failed": replan_failed,
                                                "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                                "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                                "position": [
                                                    round(getattr(self.position, "x", 0.0), 3),
                                                    round(getattr(self.position, "y", 0.0), 3),
                                                ] if self.position is not None else None,
                                            },
                                        )
                                        # #endregion
                                        # #region debug-point A:global-stop-fallback
                                        self._debug_global_check_event(
                                            "A",
                                            "naive_agent.py:plan",
                                            "[DEBUG] baseline agent entered stop fallback",
                                            {
                                                "agent_id": getattr(self, "id", None),
                                                "state": getattr(getattr(self, "state", None), "name", None),
                                                "blocker_id": blocker_id,
                                                "reason": avoidance_response.get("reason"),
                                                "path_still_conflicts": path_still_conflicts,
                                                "replan_failed": replan_failed,
                                                "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                                "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                                "position": [
                                                    round(getattr(self.position, "x", 0.0), 3),
                                                    round(getattr(self.position, "y", 0.0), 3),
                                                ] if self.position is not None else None,
                                            },
                                        )
                                        # #endregion
                                    # #endregion
                                    # #region debug-point C:stop-fallback-cascade
                                    self._emit_debug_event(
                                        "slowdown",
                                        SLOWDOWN_DEBUG_ENV_PATH,
                                        SLOWDOWN_DEBUG_FALLBACK_URL,
                                        SLOWDOWN_DEBUG_SESSION_ID,
                                        "C",
                                        "naive_agent.py:plan",
                                        "[DEBUG] agent entered stop fallback",
                                        {
                                            "agent_id": getattr(self, "id", None),
                                            "agent_state": getattr(getattr(self, "state", None), "name", None),
                                            "blocker_id": blocker_id,
                                            "reason": avoidance_response.get("reason"),
                                            "target_port_id": getattr(getattr(getattr(self, "task", None), "port", None), "id", None),
                                            "path_still_conflicts": path_still_conflicts,
                                            "force_stop_after_failed_replan": force_stop_after_failed_replan,
                                            "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                            "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                        },
                                    )
                                    # #endregion
                                    if self._should_debug_midmap_cruise_pair():
                                        # #region debug-point G:midmap-stop-fallback
                                        self._debug_v4_event(
                                            "G",
                                            "naive_agent.py:plan:midmap-stop-fallback",
                                            "[DEBUG] focused mid-map pair engaged avoidance stop fallback",
                                            {
                                                "agent_id": getattr(self, "id", None),
                                                "step": self._current_simulator_step(),
                                                "state": getattr(getattr(self, "state", None), "name", None),
                                                "position": [
                                                    round(getattr(self.position, "x", 0.0), 3),
                                                    round(getattr(self.position, "y", 0.0), 3),
                                                ] if self.position is not None else None,
                                                "reason": avoidance_response.get("reason"),
                                                "blocker_id": blocker_id,
                                                "overlay_before": previous_overlay,
                                                "avoidance_debug": avoidance_debug,
                                                "path_head": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                                            },
                                        )
                                        # #endregion
                                    if avoidance_response.get("reason") in {"port_corridor_yield", "exit_queue_owner_avoid"}:
                                        # #region debug-point E:stop-fallback-engaged
                                        self._debug_v4_event(
                                            "E",
                                            "naive_agent.py:plan:stop-fallback-engaged",
                                            "[DEBUG] v4 engaged stop fallback after avoidance replan",
                                            {
                                                "agent_id": getattr(self, "id", None),
                                                "state": getattr(getattr(self, "state", None), "name", None),
                                                "reason": avoidance_response.get("reason"),
                                                "blocker_id": blocker_id,
                                                "path_still_conflicts": path_still_conflicts,
                                                "extended_bottom_port_hold": extended_bottom_port_hold,
                                                "force_stop_after_failed_replan": force_stop_after_failed_replan,
                                                "blocker_distance": round(blocker_distance, 3) if blocker_distance is not None else None,
                                                "stop_distance": round(stop_distance, 3) if stop_distance is not None else None,
                                                "path_preview": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                                            },
                                        )
                                        # #endregion
                                    self._set_stopping_overlay(
                                        not baseline_local_hold,
                                        blocker_id,
                                        avoidance_response.get("reason"),
                                    )
                                    self.apply_local_plan(avoidance_response.get("fallback_command", (0.0, 0.0)))
                                    return
                        command = avoidance_response.get("command")
                        if command is not None:
                            bypass_without_overlay = avoidance_response.get("reason") in {
                                "baseline_direct_bypass",
                                "baseline_exit_same_source_bypass",
                                "baseline_exit_same_source_backoff",
                                "baseline_non_port_corridor_escape",
                                "baseline_non_port_corridor_backoff",
                                "baseline_port_admission_backoff",
                            }
                            # #region debug-point E:baseline-direct-command-apply
                            if type(self.global_planner).__name__ == "LayeredAStarBaselineTrafficAware":
                                self._debug_baseline_event(
                                    "E",
                                    "naive_agent.py:plan",
                                    "[DEBUG] baseline runtime applied direct bypass command",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "blocker_id": avoidance_response.get("blocker_id"),
                                        "reason": avoidance_response.get("reason"),
                                        "bypass_without_overlay": bypass_without_overlay,
                                        "command": [round(command[0], 3), round(command[1], 3)],
                                    },
                                )
                                # #region debug-point D:baseline-collision-direct-command
                                self._debug_collision_event(
                                    "D",
                                    "naive_agent.py:plan",
                                    "[DEBUG] baseline runtime executed direct avoidance command",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "blocker_id": avoidance_response.get("blocker_id"),
                                        "reason": avoidance_response.get("reason"),
                                        "bypass_without_overlay": bypass_without_overlay,
                                        "command": [round(command[0], 3), round(command[1], 3)],
                                        "position": [
                                            round(getattr(self.position, "x", 0.0), 3),
                                            round(getattr(self.position, "y", 0.0), 3),
                                        ] if self.position is not None else None,
                                    },
                                )
                                # #endregion
                                # #region debug-point B:global-direct-command
                                self._debug_global_check_event(
                                    "B",
                                    "naive_agent.py:plan",
                                    "[DEBUG] baseline agent applied direct avoidance command",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "blocker_id": avoidance_response.get("blocker_id"),
                                        "reason": avoidance_response.get("reason"),
                                        "command": [round(command[0], 3), round(command[1], 3)],
                                        "position": [
                                            round(getattr(self.position, "x", 0.0), 3),
                                            round(getattr(self.position, "y", 0.0), 3),
                                        ] if self.position is not None else None,
                                    },
                                )
                                # #endregion
                            # #endregion
                            # #region debug-point C:direct-avoidance-command
                            self._emit_debug_event(
                                "slowdown",
                                SLOWDOWN_DEBUG_ENV_PATH,
                                SLOWDOWN_DEBUG_FALLBACK_URL,
                                SLOWDOWN_DEBUG_SESSION_ID,
                                "C",
                                "naive_agent.py:plan",
                                "[DEBUG] agent applied direct avoidance command",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "agent_state": getattr(getattr(self, "state", None), "name", None),
                                    "blocker_id": avoidance_response.get("blocker_id"),
                                    "reason": avoidance_response.get("reason"),
                                    "target_port_id": getattr(getattr(getattr(self, "task", None), "port", None), "id", None),
                                    "command": [round(command[0], 3), round(command[1], 3)],
                                },
                            )
                            # #endregion
                            if self._should_debug_midmap_cruise_pair():
                                # #region debug-point G:midmap-direct-command
                                self._debug_v4_event(
                                    "G",
                                    "naive_agent.py:plan:midmap-direct-command",
                                    "[DEBUG] focused mid-map pair applied direct avoidance command",
                                    {
                                        "agent_id": getattr(self, "id", None),
                                        "step": self._current_simulator_step(),
                                        "state": getattr(getattr(self, "state", None), "name", None),
                                        "position": [
                                            round(getattr(self.position, "x", 0.0), 3),
                                            round(getattr(self.position, "y", 0.0), 3),
                                        ] if self.position is not None else None,
                                        "reason": avoidance_response.get("reason"),
                                        "blocker_id": avoidance_response.get("blocker_id"),
                                        "command": [round(command[0], 3), round(command[1], 3)],
                                        "overlay_before": previous_overlay,
                                    },
                                )
                                # #endregion
                            # baseline 独立分支里的主动侧绕不是“停下来让行”，
                            # 不应该再把自己标记成 STOPPING，否则会把局部绕行动作扩散成额外让行链。
                            safe_command, runtime_hold = self._baseline_runtime_safety_override(
                                command,
                                avoidance_response.get("reason"),
                                avoidance_response.get("blocker_id"),
                            )
                            safe_command = self._baseline_runtime_restore_speed(
                                safe_command,
                                runtime_hold,
                                avoidance_response.get("reason"),
                            )
                            if runtime_hold is None:
                                safe_command, runtime_hold = self._baseline_runtime_safety_override(
                                    safe_command,
                                    avoidance_response.get("reason"),
                                    avoidance_response.get("blocker_id"),
                                )
                            effective_reason = runtime_hold["reason"] if runtime_hold is not None else avoidance_response.get("reason")
                            effective_blocker_id = runtime_hold["blocker_id"] if runtime_hold is not None else avoidance_response.get("blocker_id")
                            self._set_stopping_overlay(
                                runtime_hold is not None or not bypass_without_overlay,
                                effective_blocker_id,
                                effective_reason,
                            )
                            self._debug_collision_event(
                                "F",
                                "naive_agent.py:plan",
                                "[DEBUG] baseline agent applied avoidance command after runtime safety arbitration",
                                {
                                    "agent_id": getattr(self, "id", None),
                                    "state": getattr(getattr(self, "state", None), "name", None),
                                    "avoidance_reason": avoidance_response.get("reason"),
                                    "blocker_id": avoidance_response.get("blocker_id"),
                                    "runtime_hold_reason": runtime_hold["reason"] if runtime_hold is not None else None,
                                    "runtime_hold_blocker_id": runtime_hold["blocker_id"] if runtime_hold is not None else None,
                                    "command": [
                                        round(safe_command[0], 3),
                                        round(safe_command[1], 3),
                                    ] if safe_command is not None else None,
                                    "overlay_reason": effective_reason,
                                },
                            )
                            self.apply_local_plan(safe_command)
                            return
                target_port = getattr(getattr(self, "task", None), "port", None)
                if target_port is not None and getattr(self.state, "name", None) in {"CRUISE", "PREQUEUE", "QUEUING"}:
                    owner_snapshot = None
                    try:
                        for observed_agent in self.perception_module.other_agents_state_in_range_of(2.6, 2 * pi):
                            observed_state = getattr(observed_agent, "userData", observed_agent)
                            observed_port = getattr(getattr(observed_state, "task", None), "port", None)
                            observed_state_name = getattr(getattr(observed_state, "state", None), "name", None)
                            if observed_port == target_port and observed_state_name in {"LOADING", "UNLOADING"}:
                                owner_snapshot = {
                                    "id": getattr(observed_state, "id", None),
                                    "state": observed_state_name,
                                    "distance": round(self.position.distance(getattr(observed_state, "position", self.position)), 3),
                                    "position": [
                                        round(getattr(getattr(observed_state, "position", None), "x", 0.0), 3),
                                        round(getattr(getattr(observed_state, "position", None), "y", 0.0), 3),
                                    ],
                                }
                                break
                    except Exception:
                        owner_snapshot = None
                    if owner_snapshot is not None:
                        # #region debug-point B:path-head-before-local-plan
                        self._debug_v4_event(
                            "B",
                            "naive_agent.py:plan:before-local-plan",
                            "[DEBUG] agent kept current path head before local planner",
                            {
                                "agent_id": getattr(self, "id", None),
                                "state": getattr(getattr(self, "state", None), "name", None),
                                "replan_flag": self.replan,
                                "goal_changed": self.goal_changed,
                                "destination": [
                                    round(getattr(self.destination_location, "x", 0.0), 3),
                                    round(getattr(self.destination_location, "y", 0.0), 3),
                                ] if self.destination_location is not None else None,
                                "path_head": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                                "owner": owner_snapshot,
                            },
                        )
                        # #endregion
                act = self.current_local_planner.compute_plan(
                            self.position,
                            self.linear_velocity,
                            self.static_environment,
                            self.perception_module,
                            self.sequence_of_poses)
                if self._should_debug_midmap_cruise_pair():
                    # #region debug-point G:midmap-local-command
                    self._debug_v4_event(
                        "G",
                        "naive_agent.py:plan:midmap-local-command",
                        "[DEBUG] focused mid-map pair fell through to local planner command",
                        {
                            "agent_id": getattr(self, "id", None),
                            "step": self._current_simulator_step(),
                            "state_before_transition": state_before_transition,
                            "state_after_transition": getattr(getattr(self, "state", None), "name", None),
                            "position": [
                                round(getattr(self.position, "x", 0.0), 3),
                                round(getattr(self.position, "y", 0.0), 3),
                            ] if self.position is not None else None,
                            "overlay_before": previous_overlay,
                            "overlay_live_before_clear": {
                                "active": bool(getattr(self, "stopping_active", False)),
                                "base_state": getattr(getattr(self, "stopping_base_state", None), "name", None),
                                "blocker_id": getattr(self, "stopping_for_agent_id", None),
                                "reason": getattr(self, "stopping_reason", None),
                            },
                            "avoidance_response": {
                                "is_none": avoidance_response is None,
                                "reason": avoidance_response.get("reason") if avoidance_response is not None else None,
                                "replan": avoidance_response.get("replan") if avoidance_response is not None else None,
                                "command": list(avoidance_response.get("command")) if avoidance_response is not None and avoidance_response.get("command") is not None else None,
                                "fallback_command": list(avoidance_response.get("fallback_command")) if avoidance_response is not None and avoidance_response.get("fallback_command") is not None else None,
                                "blocker_id": avoidance_response.get("blocker_id") if avoidance_response is not None else None,
                                "debug": avoidance_debug,
                            },
                            "path_head": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                            "local_command": [
                                round(act[0], 3),
                                round(act[1], 3),
                            ] if act is not None else None,
                        },
                    )
                    # #endregion
                if previous_overlay["active"] and previous_overlay["reason"] == "port_corridor_yield":
                    # #region debug-point F:port-hold-fell-through
                    self._debug_v4_event(
                        "F",
                        "naive_agent.py:plan:port-hold-fell-through",
                        "[DEBUG] previous port corridor hold fell through to normal local planner",
                        {
                            "agent_id": getattr(self, "id", None),
                            "step": self._current_simulator_step(),
                            "state_before_transition": state_before_transition,
                            "state_after_transition": getattr(getattr(self, "state", None), "name", None),
                            "overlay_before": previous_overlay,
                            "overlay_live_before_clear": {
                                "active": bool(getattr(self, "stopping_active", False)),
                                "base_state": getattr(getattr(self, "stopping_base_state", None), "name", None),
                                "blocker_id": getattr(self, "stopping_for_agent_id", None),
                                "reason": getattr(self, "stopping_reason", None),
                            },
                            "position": [
                                round(getattr(self.position, "x", 0.0), 3),
                                round(getattr(self.position, "y", 0.0), 3),
                            ] if self.position is not None else None,
                            "linear_velocity_before": [
                                round(getattr(self.linear_velocity, "__getitem__", lambda *_: 0.0)(0), 3),
                                round(getattr(self.linear_velocity, "__getitem__", lambda *_: 0.0)(1), 3),
                            ] if self.linear_velocity is not None else None,
                            "destination": [
                                round(getattr(self.destination_location, "x", 0.0), 3),
                                round(getattr(self.destination_location, "y", 0.0), 3),
                            ] if self.destination_location is not None else None,
                            "path_head": self.global_planner._point_preview(self.sequence_of_poses) if hasattr(self.global_planner, "_point_preview") else [],
                            "avoidance_response": {
                                "is_none": avoidance_response is None,
                                "reason": avoidance_response.get("reason") if avoidance_response is not None else None,
                                "replan": avoidance_response.get("replan") if avoidance_response is not None else None,
                                "command": list(avoidance_response.get("command")) if avoidance_response is not None and avoidance_response.get("command") is not None else None,
                                "fallback_command": list(avoidance_response.get("fallback_command")) if avoidance_response is not None and avoidance_response.get("fallback_command") is not None else None,
                                "blocker_id": avoidance_response.get("blocker_id") if avoidance_response is not None else None,
                                "stop_distance": avoidance_response.get("stop_distance") if avoidance_response is not None else None,
                            },
                            "local_command": [
                                round(act[0], 3),
                                round(act[1], 3),
                            ] if act is not None else None,
                        },
                    )
                    # #endregion
                safe_act, runtime_hold = self._baseline_runtime_safety_override(
                    act,
                    avoidance_response.get("reason") if avoidance_response is not None else None,
                    avoidance_response.get("blocker_id") if avoidance_response is not None else None,
                )
                safe_act = self._baseline_runtime_restore_speed(
                    safe_act,
                    runtime_hold,
                    avoidance_response.get("reason") if avoidance_response is not None else None,
                )
                if runtime_hold is None:
                    safe_act, runtime_hold = self._baseline_runtime_safety_override(
                        safe_act,
                        avoidance_response.get("reason") if avoidance_response is not None else None,
                        avoidance_response.get("blocker_id") if avoidance_response is not None else None,
                    )
                if runtime_hold is not None:
                    self._set_stopping_overlay(True, runtime_hold["blocker_id"], runtime_hold["reason"])
                else:
                    self._set_stopping_overlay(False)
                self.apply_local_plan(safe_act)
                if type(self.global_planner).__name__ == "LayeredAStarBaselineTrafficAware":
                    effective_act = safe_act if safe_act is not None else act
                    command_speed = sqrt(effective_act[0] ** 2 + effective_act[1] ** 2) if effective_act is not None else 0.0
                    # #region debug-point D:final-low-speed-command
                    if command_speed <= max(0.05, self.cruise_speed * 0.35):
                        self._debug_multi_stop_event(
                            "D",
                            "naive_agent.py:plan",
                            "[DEBUG] multi-stop final command stayed at low speed after full arbitration",
                            {
                                "agent_id": getattr(self, "id", None),
                                "state": getattr(getattr(self, "state", None), "name", None),
                                "avoidance_reason": avoidance_response.get("reason") if avoidance_response is not None else None,
                                "runtime_hold_reason": runtime_hold["reason"] if runtime_hold is not None else None,
                                "runtime_hold_blocker_id": runtime_hold["blocker_id"] if runtime_hold is not None else None,
                                "overlay_active": bool(getattr(self, "stopping_active", False)),
                                "overlay_reason": getattr(self, "stopping_reason", None),
                                "command": [
                                    round(effective_act[0], 3),
                                    round(effective_act[1], 3),
                                ] if effective_act is not None else None,
                                "command_speed": round(command_speed, 3),
                                "step": self._current_simulator_step(),
                            },
                        )
                    # #endregion
                    if avoidance_response is None or command_speed <= max(0.05, self.cruise_speed * 0.35):
                        # #region debug-point B:global-local-command
                        self._debug_global_check_event(
                            "B",
                            "naive_agent.py:plan",
                            "[DEBUG] baseline agent fell through to local planner command",
                            {
                                "agent_id": getattr(self, "id", None),
                                "state": getattr(getattr(self, "state", None), "name", None),
                                "avoidance_reason": avoidance_response.get("reason") if avoidance_response is not None else None,
                                "command": [
                                    round(effective_act[0], 3),
                                    round(effective_act[1], 3),
                                ] if effective_act is not None else None,
                                "command_speed": round(command_speed, 3),
                                "cruise_speed": round(getattr(self, "cruise_speed", 0.0), 3),
                                "stopping_active": bool(getattr(self, "stopping_active", False)),
                                "stopping_for_agent_id": getattr(self, "stopping_for_agent_id", None),
                                "position": [
                                    round(getattr(self.position, "x", 0.0), 3),
                                    round(getattr(self.position, "y", 0.0), 3),
                                ] if self.position is not None else None,
                            },
                        )
                        # #endregion
                # if act[0] > self.cruise_speed:
                #     act[0] = self.cruise_speed
                # if act[1] > self.max_angular_velocity:
                #     act[1] = self.max_angular_velocity
                # elif act[1] < 0 and act[1] < -self.max_angular_velocity:
                #     act[1] = - self.max_angular_velocity
                # self.speed = act[0]
                # self.angular_velocity = act[1]
            else: # has destination, arrived current destination but still in CRUISE
                if self.state == AgentState.CRUISE:
                    self.go_internal_stations()
                else:
                    self.internal_stations = []
                    self._set_stopping_overlay(False)
                    self.stop()
        else: # do not have destination
            self._set_stopping_overlay(False)
            self.stop()
        
    def go_internal_stations(self):
        if len(self.internal_stations) > 0:
            next_goal = self.internal_stations.pop(0)
            self.task.destination_location = next_goal
            self.destination_location = next_goal
            self.goal_changed = True
