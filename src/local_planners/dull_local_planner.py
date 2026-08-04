"""
@Copyright Dorabot Inc.
@date : 2019-08
@author: cenrong.dai@dorabot.com
@brief : local planner which follows the waypoint of the given global_planner_path strictly
"""
import json
import urllib.request
from geometry import Vector, compute_direction, Point
from .local_planner import LocalPlanner
from math import sqrt

DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env"
DEBUG_FALLBACK_URL = "http://127.0.0.1:7778/event"
DEBUG_SESSION_ID = "v4-port-collision"

class DullPlanner(LocalPlanner):
    _DEBUG_ENDPOINT_DISABLED = object()
    _debug_endpoint_cache = {}
    """

    Keyword arguments:
    position -- agent position
    velocity -- linear velocity of the agent
    sensor_observation -- use perception module to emulate sensor observation
    global_planner_path -- list of poses (way points) obtained from global planner
    Return:
    velcocity -- velocity command [only linear velocity returned in current version]
    local_path -- optional
    """
    def _resolve_debug_endpoint(self):
        cached = self._debug_endpoint_cache.get("v4", None)
        if cached is self._DEBUG_ENDPOINT_DISABLED:
            return None
        if cached is not None:
            return cached

        try:
            debug_url = DEBUG_FALLBACK_URL
            session_id = DEBUG_SESSION_ID
            debug_enabled = False
            with open(DEBUG_ENV_PATH, "r", encoding="utf-8") as env_file:
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

        self._debug_endpoint_cache["v4"] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_DISABLED:
            return None
        return endpoint

    def _debug_v4_event(self, hypothesis_id, location, message, data):
        endpoint = self._resolve_debug_endpoint()
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
            self._debug_endpoint_cache["v4"] = self._DEBUG_ENDPOINT_DISABLED

    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        local_path = []
        
        try:
            if position.arrive(global_planner_path[0]):
                global_planner_path.pop(0)
            goal_pose = global_planner_path[0]
        except:
            goal_pose = self.agent.destination_location

        result_vel = Vector(velocity[0], velocity[1])
        force_goal = self.__goal_attraction(position, goal_pose)
        result_vel = result_vel + force_goal
        ratio = sqrt(result_vel.x**2 + result_vel.y **2)
        try:
            result_vel = result_vel.scale(self.agent.cruise_speed/ratio)
        except: # divide zero
            return (0.0, 0.0)
        target_port = getattr(getattr(self.agent, "task", None), "port", None)
        owner_snapshot = None
        if target_port is not None and getattr(getattr(self.agent, "state", None), "name", None) in {"CRUISE", "PREQUEUE", "QUEUING"}:
            try:
                for observed_agent in sensor_observation.other_agents_state_in_range_of(2.6, 2 * 3.141592653589793):
                    observed_state = getattr(observed_agent, "userData", observed_agent)
                    observed_port = getattr(getattr(observed_state, "task", None), "port", None)
                    observed_state_name = getattr(getattr(observed_state, "state", None), "name", None)
                    if observed_port == target_port and observed_state_name in {"LOADING", "UNLOADING"}:
                        owner_snapshot = {
                            "id": getattr(observed_state, "id", None),
                            "state": observed_state_name,
                            "distance": round(position.distance(getattr(observed_state, "position", position)), 3),
                            "position": [
                                round(getattr(getattr(observed_state, "position", None), "x", 0.0), 3),
                                round(getattr(getattr(observed_state, "position", None), "y", 0.0), 3),
                            ],
                        }
                        break
            except Exception:
                owner_snapshot = None
        if owner_snapshot is not None:
            # #region debug-point A:dullplanner-command
            self._debug_v4_event(
                "A",
                "dull_local_planner.py:compute_plan",
                "[DEBUG] dull planner emitted local velocity near active port owner",
                {
                    "agent_id": getattr(self.agent, "id", None),
                    "agent_state": getattr(getattr(self.agent, "state", None), "name", None),
                    "position": [round(position.x, 3), round(position.y, 3)],
                    "velocity_in": [round(velocity[0], 3), round(velocity[1], 3)],
                    "goal_pose": [round(goal_pose.x, 3), round(goal_pose.y, 3)],
                    "force_goal": [round(force_goal.x, 3), round(force_goal.y, 3)],
                    "result_vel": [round(result_vel.x, 3), round(result_vel.y, 3)],
                    "path_head": [
                        [round(p.x, 3), round(p.y, 3)] for p in list(global_planner_path)[:3]
                    ],
                    "owner": owner_snapshot,
                },
            )
            # #endregion
        return result_vel.to_tuple()
    def __goal_attraction(self, pos_current, loc_destination):
        dist = pos_current.distance(loc_destination)
        vec = compute_direction(pos_current, loc_destination).normalize()
        return vec.scale(100/(dist+0.01))
