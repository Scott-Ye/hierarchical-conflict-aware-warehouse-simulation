
# @author : (xiaoyu.ge, tian.xiao)@dorabot.com
# @brief : partly adopted from the author's github repository: https://github.com/fantastdd/physical-reasoning-2d/blob/master/scenario_generator.py
import pygame
from Box2D import *
from pygame import Rect
from shape import Rectangle
from geometry import Vector, Point
# from framework import * # from local framework.py file
from setup_environment.environment import Environment
from setup_environment.port import Port
from server import Server
from agents.agent_state_machine import AgentState
from agents.agent import Agent
from agents.naive_agent import NaiveAgent
from local_planners.local_planner import LocalPlanner
from local_planners.dull_local_planner import DullPlanner
from local_planners.virtual_force_planner import VirtualForcePlanner
from local_planners.rvo_planner import RVOPlanner
from local_planners.DD_planner import DDPlanner
from local_planners.hrvo_planner import HRVOPlanner
from local_planners.flc_local_planner import FLCPlanner
from global_planners.global_planner import MapType
from global_planners.sample_global_planner import SimpleAStar
from global_planners.layered_astar_planner import LayeredAStar
from global_planners.rrtstar_planner import RRTStar
from global_planners.multiagent_planner_local_entry import MultiAgentPlannerLocalEntry
from multiagent_global_planners.multiagent_planner import MultiAgentPlanner
from multiagent_global_planners.marrtstar_planner import MARRTStar
from multiagent_global_planners.inash_planner import INashRRT
from representation.gridmap_a import GridmapWithNeighbors
from visualisation import Visualisation
from math import *
from enum import Enum
from agents.sensor import Sensor, SensorContactListener, SensorRayCast
import sys, os
import time
import json
import random
import numpy as np
import matplotlib.pyplot as plt
import re
from interaction_handler import *
from experiment_logging import RunMetricsRecorder
import urllib.request

"""entity category of different fixtures in simulator"""
class EntityCategory(Enum):
    """Add new physical body entity here
       The logical of contacts check is 
       Collide =
          (A.maskBits & B.categoryBits) != 0 &&
          (A.categoryBits & B.maskBits) != 0
    """
    wall = 1
    obstacle = 1
    port = 2
    sensor = 4
    agent = 4


"""Multi-Agents Simulator
"""
class Simulator(b2ContactListener):
    step_counter = 0
    sensor_history_length = 3
    _DEBUG_ENDPOINT_DISABLED = object()
    _debug_endpoint_cache = {}
    MIDMAP_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-midmap-collision.env"
    MIDMAP_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
    MIDMAP_DEBUG_SESSION_ID = "baseline-midmap-collision"
    MIDMAP_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-midmap-collision.ndjson"
    def __init__(self, cmd_args):
        super(Simulator, self).__init__()
        # whether run headless
        self.simulation_times = cmd_args.time
        self.TIME_STEP = 1.0/60
        self.server = None
        self.agents = []
        self.loading_ports = []
        self.unloading_ports = []
        """store physical bodies of agents in the simulation"""
        self.b2_objects =  {}
        """store physical bodies of ports and obstacles"""
        self.b2_static_objects = []
        # self.clock = pygame.time.Clock()
        self.timespan = 0
        self.time = 0
        self.task_count = 0
        self.start_time = 0
        self.heatmap_data = []
        self.agent_global_planner = ''
        self.agent_local_planner = ''
        self.task_manager_name = ''
        '''disable gravity, set contact listener for sensor using'''
        self.world = b2World(gravity=(0,0), doSleep=True, contactListener = SensorContactListener()) 
        self.ray_length_list = []
        self.ray_line_list = []
        self.free_control = cmd_args.free
        self.title = cmd_args.title
        self.agent_details = cmd_args.agent_details
        self.receive_q = None # is used to receive the cmd from controller
        self.send_q = None
        self.metrics_recorder = None
        self.latest_run_summary = None
        self.last_reported_minute = -1
        self.enable_stuck_recovery = getattr(cmd_args, "stuck_recovery", False)
        self._dbg_prev_body_pose = {}
        self._dbg_still_steps = {}
        self._dbg_contact_steps = {}
        self._dbg_gui_speed_snapshot = {}
        self.contact_replan_threshold = max(1, int(getattr(cmd_args, "stuck_recovery_threshold", 10) or 10))

    def _resolve_debug_endpoint(self, cache_key, env_path, fallback_url, fallback_session_id):
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
            if session_id == "gui-global-slowdown":
                debug_enabled = True
            if session_id == "baseline-traffic-collision":
                debug_enabled = True
            endpoint = (debug_url, session_id) if debug_enabled else self._DEBUG_ENDPOINT_DISABLED
        except Exception:
            endpoint = self._DEBUG_ENDPOINT_DISABLED
        self._debug_endpoint_cache[cache_key] = endpoint
        if endpoint is self._DEBUG_ENDPOINT_DISABLED:
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
            self._debug_endpoint_cache[cache_key] = self._DEBUG_ENDPOINT_DISABLED

    def _debug_v4_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "sim_v4",
            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env",
            "http://127.0.0.1:7778/event",
            "v4-port-collision",
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_global_check_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "sim_global_check",
            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\global-no-double-stop.env",
            "http://127.0.0.1:7780/event",
            "global-no-double-stop",
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_gui_slowdown_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "sim_gui_global_slowdown",
            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\gui-global-slowdown.env",
            "http://127.0.0.1:7778/event",
            "gui-global-slowdown",
            hypothesis_id,
            location,
            message,
            data,
        )

    def _debug_collision_event(self, hypothesis_id, location, message, data):
        self._emit_debug_event(
            "sim_baseline_collision",
            r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-traffic-collision.env",
            "http://127.0.0.1:7777/event",
            "baseline-traffic-collision",
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
        self._emit_debug_event(
            "midmap_collision",
            self.MIDMAP_DEBUG_ENV_PATH,
            self.MIDMAP_DEBUG_FALLBACK_URL,
            self.MIDMAP_DEBUG_SESSION_ID,
            hypothesis_id,
            location,
            message,
            data,
        )
    # could be loaded from a json file as well.
    def set_environment(self, obstacle_tuples_list = []):
        self.environment = Environment()
        self.environment.create_gridmap()
        self.environment.setup_walls()
        '''Choose ports setup place here'''
        if not self.free_control:
            self.environment.setup_loading_ports_on_row()
            self.environment.setup_unloading_ports_on_bottom()
        self.environment.setup_obstacles(obstacle_tuples_list)
        self.environment.create_static_continuous_space() # this must go after ports and obstacles setup
        return self.environment


    def set_agents(self, agents):
        self.agents = agents
        # create physical objects for agents
        for agent in agents:
            dynamic_body = self.world.CreateDynamicBody(position=[agent.position.x, agent.position.y], userData = agent)
            agent_fixture = dynamic_body.CreatePolygonFixture(box=agent.shape.get_half_dimension(),  userData = agent)
            sensor_fixture = dynamic_body.CreateFixture(shape = agent.sensor.shape, userData = agent.sensor)
            sensor_fixture.sensor = True
            sensor_fixture.filterData.categoryBits = EntityCategory.sensor.value
            sensor_fixture.filterData.maskBits = EntityCategory.sensor.value | EntityCategory.port.value | EntityCategory.wall.value
            agent_fixture.filterData.categoryBits = EntityCategory.agent.value
            agent_fixture.filterData.maskBits = EntityCategory.agent.value | EntityCategory.port.value | EntityCategory.wall.value
            self.b2_objects[agent.id] = dynamic_body

    def set_ports(self, ports):
        for port in list(ports.values()):
            static_port_body = self.world.CreateStaticBody(position = [port.location.x+port.dimension[0]/2.0, port.location.y+port.dimension[1]/2.0], userData = port)
            port_fixture = static_port_body.CreatePolygonFixture(box = (port.dimension[0]/2.0, port.dimension[1]/2.0), userData = port)
            port_fixture.filterData.categoryBits = EntityCategory.port.value
            self.b2_static_objects.append(static_port_body)

    def set_obstacles(self, obstacles):
        for obstacle in list(obstacles.values()):
            static_obstacle_body = self.world.CreateStaticBody(position = [obstacle.location.x+obstacle.dimension[0]/2.0, obstacle.location.y+obstacle.dimension[1]/2.0], userData = obstacle)
            static_obstacle_fixture = static_obstacle_body.CreatePolygonFixture(box = (obstacle.dimension[0]/2.0, obstacle.dimension[1]/2.0), userData = obstacle)
            static_obstacle_fixture.filterData.categoryBits = EntityCategory.obstacle.value
            self.b2_static_objects.append(static_obstacle_body)
         
    def set_walls(self, walls):
        for wall in list(walls.values()):
            static_wall_body = self.world.CreateStaticBody(position = wall.shape.get_box2d_location(), userData = wall)
            static_wall_fixture = static_wall_body.CreatePolygonFixture(box = wall.shape.get_half_dimension(), userData = wall)
            static_wall_fixture.filterData.categoryBits = EntityCategory.wall.value

    def set_local_planner(self, test_agent, local_planner):
        self.agent_local_planner = local_planner.__name__
        test_agent.use_local_planner(local_planner(test_agent))

    def set_global_planner(self, test_agent, global_planner):
        '''All agents share the same (type) of global planner'''
        # deal_with_old_planner
        old_planner = test_agent.global_planner
        if MultiAgentPlanner.__subclasscheck__(type(old_planner)):
            old_planner.remove_agent_under_control(test_agent)
        elif GlobalPlanner.__subclasscheck__(type(old_planner)):
            pass
        else: # None
            pass
        # deal with newly assigned planner
        if not global_planner: # None
            test_agent.global_planner = None
            self.agent_global_planner = 'None'
        elif MultiAgentPlanner.__subclasscheck__(global_planner): # this type of global planner plan paths taking all agents under its control into consideration (collabrative)
            self.agent_global_planner = global_planner.__name__
            # see whether this type of multiagent global planner has been initialized
            server = test_agent.server
            ma_planner = None
            environment_map = self.environment.static_gridmap
            for planner in server.multiagent_global_planners:
                if type(planner) == global_planner: # already exist, not need to create new
                    ma_planner = planner
                    if ma_planner.MAP == MapType.CONTINUOUS:
                        environment_map = self.environment.static_continuous_space
                    break
            if not ma_planner: # not exist: create new, check the required environment type
                if global_planner.MAP == MapType.CONTINUOUS:
                    environment_map = self.environment.static_continuous_space
                ma_planner = global_planner(server, environment_map, self.world, self.TIME_STEP)
                server.add_multiagent_local_planner(ma_planner)
            # ask ma_planner to control the agent
            ma_planner.add_agent_under_control(test_agent)
            # locally installed ma_planner entry on each agent
            test_agent.use_global_planner(MultiAgentPlannerLocalEntry(test_agent))
            test_agent.static_environment = environment_map
        elif GlobalPlanner.__subclasscheck__(global_planner): # this type of global planner only concern the agent installed it and the static environment (non-collabrative)
            self.agent_global_planner = global_planner.__name__
            environment_map = self.environment.static_gridmap
            if global_planner.MAP == MapType.CONTINUOUS:
                environment_map = self.environment.static_continuous_space
            # 本项目的 v1 / v2 / v3 / v4 都属于这里的单 agent global planner。
            # 它们虽然在实验上是一轮轮改进，但在工程实现上保持为独立脚本和独立类，
            # 这样切换命令行参数时，就能直接把不同版本接到同一个 simulator 流程里。
            test_agent.use_global_planner(global_planner(test_agent))
            test_agent.static_environment = environment_map
        else: # Unknown
            print("simulator.py: unknown type of global planner", global_planner)
            test_agent.global_planner = None
        test_agent.goal_changed = True # trigger replan

    def set_multiagent_global_planner(self, server, agents, general_multiagent_global_planner, general_static_environment, time_step):
        if general_multiagent_global_planner:
            ma_class = general_multiagent_global_planner
            multiagent_global_planner = ma_class(general_static_environment, server, self.world, time_step) # create one for all agents
            for agent in agents:
                agent.global_planner.connect_to_multiagent_global_planner(multiagent_global_planner) # overwrite
            self.agent_global_planner = ma_class.__name__

    def ini_perception(self):
        for agent in self.agents:
            agent.ini_perception_module(self.b2_objects[agent.id], list(self.b2_objects.values()), self.b2_static_objects)
    

    def set_agent_velocity(self, agent, agent_body):
        if abs(agent_body.angle) >= pi:
            temp_angle = abs(agent_body.angle)%pi
            if agent_body.angle > 0:
                agent_body.angle = -pi + temp_angle
            elif agent_body.angle < 0:
                agent_body.angle = pi - temp_angle
        speed = agent.speed
        agent_body.angularVelocity = agent.angular_velocity
        agent_body.linearVelocity = (cos(agent_body.angle)*speed, 
                sin(agent_body.angle)*speed)


    def step(self):
        _dbg_step_wall_start = time.perf_counter()
        _dbg_raycast_start = time.perf_counter()
        Port.simulator_step = Simulator.step_counter
        _dbg_focus_before = None
        _dbg_focus_after = None
        for agent in self.agents:
            # obtain observation
            ray_length_list = []
            agent.ray_point_list = []
            agent_body = self.b2_objects[agent.get_id()]
            for line_num in range(0, 512):
                angle = line_num/511.0*pi - pi/2 + agent_body.angle
                length = self.ray_cast_callback(agent_body, angle)
                ray_length_list.append(length)
            agent.ray_length_list = ray_length_list
            
            if len(agent.history_ray_length_list) >= Simulator.sensor_history_length:
                agent.history_ray_length_list = agent.history_ray_length_list[-(Simulator.sensor_history_length-1):]
                agent.history_ray_point_list = agent.history_ray_point_list[-(Simulator.sensor_history_length-1):]
            agent.history_ray_length_list.append(agent.ray_length_list)
            agent.history_ray_point_list.append(agent.ray_point_list)
        _dbg_raycast_seconds = round(time.perf_counter() - _dbg_raycast_start, 4)
        _dbg_update_start = time.perf_counter()
        _dbg_agent_update_timings = []
        for agent in self.agents:
            _dbg_agent_update_start = time.perf_counter()
            self.__update_agent_state(agent, agent.ray_length_list)
            agent_body = self.b2_objects[agent.get_id()]
            if getattr(agent, "motion_output_type", "velocity_vector") == "differential_drive":
                self.set_agent_velocity(agent, agent_body)
            else:
                agent_body.angularVelocity = 0
                agent_body.linearVelocity = agent.linear_velocity
                if abs(agent.linear_velocity[0]) > 1e-9 or abs(agent.linear_velocity[1]) > 1e-9:
                    agent_body.angle = atan2(agent.linear_velocity[1], agent.linear_velocity[0])
            _dbg_agent_update_timings.append(
                {
                    "agent_id": getattr(agent, "id", None),
                    "state": getattr(getattr(agent, "state", None), "name", None),
                    "update_seconds": round(time.perf_counter() - _dbg_agent_update_start, 4),
                    "replan": bool(getattr(agent, "replan", False)),
                    "stopping_active": bool(getattr(agent, "stopping_active", False)),
                }
            )
        _dbg_update_seconds = round(time.perf_counter() - _dbg_update_start, 4)

        if len(self.agents) >= 4:
            _dbg_a = next((agent for agent in self.agents if getattr(agent, "id", None) == 0), None)
            _dbg_b = next((agent for agent in self.agents if getattr(agent, "id", None) == 3), None)
            if _dbg_a is not None and _dbg_b is not None:
                _dbg_state_pair = {
                    getattr(getattr(_dbg_a, "state", None), "name", None),
                    getattr(getattr(_dbg_b, "state", None), "name", None),
                }
                _dbg_distance = _dbg_a.position.distance(_dbg_b.position)
                if _dbg_distance <= 1.6 and "CRUISE" in _dbg_state_pair and len(_dbg_state_pair.intersection({"LOADING", "UNLOADING", "QUEUING"})) > 0:
                    _dbg_body_a = self.b2_objects[_dbg_a.get_id()]
                    _dbg_body_b = self.b2_objects[_dbg_b.get_id()]
                    _dbg_focus_before = (
                        _dbg_a,
                        _dbg_b,
                        {
                            "step": Simulator.step_counter,
                            "distance": round(_dbg_distance, 3),
                            "a": {
                                "id": _dbg_a.id,
                                "state": getattr(getattr(_dbg_a, "state", None), "name", None),
                                "position": [round(_dbg_a.position.x, 3), round(_dbg_a.position.y, 3)],
                                "cmd_vel": [round(_dbg_a.linear_velocity[0], 3), round(_dbg_a.linear_velocity[1], 3)],
                                "body_vel": [round(_dbg_body_a.linearVelocity[0], 3), round(_dbg_body_a.linearVelocity[1], 3)],
                            },
                            "b": {
                                "id": _dbg_b.id,
                                "state": getattr(getattr(_dbg_b, "state", None), "name", None),
                                "position": [round(_dbg_b.position.x, 3), round(_dbg_b.position.y, 3)],
                                "cmd_vel": [round(_dbg_b.linear_velocity[0], 3), round(_dbg_b.linear_velocity[1], 3)],
                                "body_vel": [round(_dbg_body_b.linearVelocity[0], 3), round(_dbg_body_b.linearVelocity[1], 3)],
                            },
                        },
                    )
                    # #region debug-point E:pre-world-step
                    self._debug_v4_event(
                        "E",
                        "simulator.py:step:pre-world-step",
                        "[DEBUG] simulator captured pre-step focused pair state",
                        _dbg_focus_before[2],
                    )
                    # #endregion
            _dbg_midmap_a = next((agent for agent in self.agents if getattr(agent, "id", None) == 1), None)
            _dbg_midmap_b = next((agent for agent in self.agents if getattr(agent, "id", None) == 2), None)
            if _dbg_midmap_a is not None and _dbg_midmap_b is not None and 165 <= Simulator.step_counter <= 180:
                _dbg_midmap_body_a = self.b2_objects[_dbg_midmap_a.get_id()]
                _dbg_midmap_body_b = self.b2_objects[_dbg_midmap_b.get_id()]
                _dbg_midmap_before = {
                    "step": Simulator.step_counter,
                    "distance": round(_dbg_midmap_a.position.distance(_dbg_midmap_b.position), 3),
                    "a": {
                        "id": _dbg_midmap_a.id,
                        "state": getattr(getattr(_dbg_midmap_a, "state", None), "name", None),
                        "stopping": bool(getattr(_dbg_midmap_a, "stopping_active", False)),
                        "task": getattr(getattr(getattr(_dbg_midmap_a, "task", None), "type", None), "name", None),
                        "position": [round(_dbg_midmap_a.position.x, 3), round(_dbg_midmap_a.position.y, 3)],
                        "cmd_vel": [round(_dbg_midmap_a.linear_velocity[0], 3), round(_dbg_midmap_a.linear_velocity[1], 3)],
                        "body_vel": [round(_dbg_midmap_body_a.linearVelocity[0], 3), round(_dbg_midmap_body_a.linearVelocity[1], 3)],
                        "path_head": [[round(p.x, 3), round(p.y, 3)] for p in list(getattr(_dbg_midmap_a, "sequence_of_poses", []) or [])[:3]],
                    },
                    "b": {
                        "id": _dbg_midmap_b.id,
                        "state": getattr(getattr(_dbg_midmap_b, "state", None), "name", None),
                        "stopping": bool(getattr(_dbg_midmap_b, "stopping_active", False)),
                        "task": getattr(getattr(getattr(_dbg_midmap_b, "task", None), "type", None), "name", None),
                        "position": [round(_dbg_midmap_b.position.x, 3), round(_dbg_midmap_b.position.y, 3)],
                        "cmd_vel": [round(_dbg_midmap_b.linear_velocity[0], 3), round(_dbg_midmap_b.linear_velocity[1], 3)],
                        "body_vel": [round(_dbg_midmap_body_b.linearVelocity[0], 3), round(_dbg_midmap_body_b.linearVelocity[1], 3)],
                        "path_head": [[round(p.x, 3), round(p.y, 3)] for p in list(getattr(_dbg_midmap_b, "sequence_of_poses", []) or [])[:3]],
                    },
                }
                # #region debug-point G:midmap-pre-world-step
                self._debug_v4_event(
                    "G",
                    "simulator.py:step:midmap-pre-world-step",
                    "[DEBUG] simulator captured focused mid-map cruise pair before physics step",
                    _dbg_midmap_before,
                )
                # #endregion

        _dbg_physics_start = time.perf_counter()
        self.world.Step(self.TIME_STEP, 10, 10)
        _dbg_physics_seconds = round(time.perf_counter() - _dbg_physics_start, 4)
        _dbg_mobile_agents = []
        _dbg_simultaneous_drop = []
        for _dbg_agent in self.agents:
            _dbg_state = getattr(getattr(_dbg_agent, "state", None), "name", None)
            if _dbg_state not in {"CRUISE", "PREQUEUE", "QUEUING"}:
                continue
            _dbg_body = self.b2_objects.get(_dbg_agent.get_id())
            if _dbg_body is None:
                continue
            _dbg_cruise = float(getattr(_dbg_agent, "cruise_speed", 0.0) or 0.0)
            if _dbg_cruise <= 1e-9:
                continue
            _dbg_command_speed = float(getattr(_dbg_agent, "speed", 0.0) or 0.0)
            _dbg_body_speed = sqrt(float(_dbg_body.linearVelocity[0]) ** 2 + float(_dbg_body.linearVelocity[1]) ** 2)
            _dbg_command_ratio = round(_dbg_command_speed / _dbg_cruise, 3)
            _dbg_body_ratio = round(_dbg_body_speed / _dbg_cruise, 3)
            _dbg_prev = self._dbg_gui_speed_snapshot.get(_dbg_agent.get_id(), {})
            _dbg_entry = {
                "agent_id": getattr(_dbg_agent, "id", None),
                "state": _dbg_state,
                "command_speed": round(_dbg_command_speed, 3),
                "body_speed": round(_dbg_body_speed, 3),
                "cruise_speed": round(_dbg_cruise, 3),
                "command_ratio": _dbg_command_ratio,
                "body_ratio": _dbg_body_ratio,
                "prev_command_ratio": _dbg_prev.get("command_ratio"),
                "prev_body_ratio": _dbg_prev.get("body_ratio"),
            }
            _dbg_mobile_agents.append(_dbg_entry)
            if (
                _dbg_prev.get("body_ratio") is not None
                and _dbg_prev.get("body_ratio") - _dbg_body_ratio >= 0.25
                and _dbg_body_ratio <= 0.75
            ):
                _dbg_simultaneous_drop.append(_dbg_entry)
        _dbg_step_wall_seconds = round(time.perf_counter() - _dbg_step_wall_start, 4)
        if _dbg_mobile_agents and (
            Simulator.step_counter % 5 == 0
            or _dbg_step_wall_seconds >= 0.05
            or len(_dbg_simultaneous_drop) >= 3
        ):
            # #region debug-point A:simulator-speed-snapshot
            self._debug_gui_slowdown_event(
                "A" if _dbg_step_wall_seconds >= 0.05 else ("B" if len(_dbg_simultaneous_drop) >= 3 else "C"),
                "simulator.py:step",
                "[DEBUG] simulator sampled mobile agent speeds",
                {
                    "step": Simulator.step_counter,
                    "sim_time": round(self.time, 3),
                    "step_wall_seconds": _dbg_step_wall_seconds,
                    "raycast_seconds": _dbg_raycast_seconds,
                    "update_seconds": _dbg_update_seconds,
                    "physics_seconds": _dbg_physics_seconds,
                    "simultaneous_drop_count": len(_dbg_simultaneous_drop),
                    "slowest_agents": sorted(_dbg_agent_update_timings, key=lambda item: item["update_seconds"], reverse=True)[:3],
                    "mobile_agents": _dbg_mobile_agents,
                },
            )
            # #endregion
        self._dbg_gui_speed_snapshot = {
            _dbg_entry["agent_id"]: {
                "command_ratio": _dbg_entry["command_ratio"],
                "body_ratio": _dbg_entry["body_ratio"],
            }
            for _dbg_entry in _dbg_mobile_agents
        }
        if _dbg_focus_before is not None:
            _dbg_a, _dbg_b, _dbg_payload = _dbg_focus_before
            _dbg_payload = dict(_dbg_payload)
            _dbg_payload["a"] = dict(_dbg_payload["a"])
            _dbg_payload["b"] = dict(_dbg_payload["b"])
            _dbg_payload["post_distance"] = round(_dbg_a.position.distance(_dbg_b.position), 3)
            _dbg_payload["a"]["post_position"] = [round(_dbg_a.position.x, 3), round(_dbg_a.position.y, 3)]
            _dbg_payload["b"]["post_position"] = [round(_dbg_b.position.x, 3), round(_dbg_b.position.y, 3)]
            _dbg_focus_after = _dbg_payload
            # #region debug-point E:post-world-step
            self._debug_v4_event(
                "E",
                "simulator.py:step:post-world-step",
                "[DEBUG] simulator captured post-step focused pair state",
                _dbg_focus_after,
            )
            # #endregion
        if len(self.agents) >= 4 and 165 <= Simulator.step_counter <= 180:
            _dbg_midmap_a = next((agent for agent in self.agents if getattr(agent, "id", None) == 1), None)
            _dbg_midmap_b = next((agent for agent in self.agents if getattr(agent, "id", None) == 2), None)
            if _dbg_midmap_a is not None and _dbg_midmap_b is not None:
                _dbg_midmap_after = {
                    "step": Simulator.step_counter,
                    "post_distance": round(_dbg_midmap_a.position.distance(_dbg_midmap_b.position), 3),
                    "a": {
                        "id": _dbg_midmap_a.id,
                        "post_position": [round(_dbg_midmap_a.position.x, 3), round(_dbg_midmap_a.position.y, 3)],
                        "post_stopping": bool(getattr(_dbg_midmap_a, "stopping_active", False)),
                    },
                    "b": {
                        "id": _dbg_midmap_b.id,
                        "post_position": [round(_dbg_midmap_b.position.x, 3), round(_dbg_midmap_b.position.y, 3)],
                        "post_stopping": bool(getattr(_dbg_midmap_b, "stopping_active", False)),
                    },
                }
                # #region debug-point G:midmap-post-world-step
                self._debug_v4_event(
                    "G",
                    "simulator.py:step:midmap-post-world-step",
                    "[DEBUG] simulator captured focused mid-map cruise pair after physics step",
                    _dbg_midmap_after,
                )
                # #endregion
        """
        self.time is the simulator world time with unit sec
        """
        # #region debug-point A:contact-and-stuck
        if self.enable_stuck_recovery:
            _dbg_active_contacts = set()
            for _dbg_contact in self.world.contacts:
                if not _dbg_contact.touching:
                    continue
                if _dbg_contact.fixtureA.sensor or _dbg_contact.fixtureB.sensor:
                    continue
                _dbg_a = getattr(_dbg_contact.fixtureA.body, 'userData', None)
                _dbg_b = getattr(_dbg_contact.fixtureB.body, 'userData', None)
                if not (hasattr(_dbg_a, 'id') and hasattr(_dbg_b, 'id')):
                    continue
                _dbg_pair = tuple(sorted((_dbg_a.id, _dbg_b.id)))
                _dbg_active_contacts.add(_dbg_pair)
                _dbg_steps = self._dbg_contact_steps.get(_dbg_pair, 0) + 1
                self._dbg_contact_steps[_dbg_pair] = _dbg_steps
                _dbg_state_a = getattr(getattr(_dbg_a, "state", None), "name", None)
                _dbg_state_b = getattr(getattr(_dbg_b, "state", None), "name", None)
                if _dbg_steps in (self.contact_replan_threshold, self.contact_replan_threshold * 3):
                    _dbg_states = {_dbg_state_a, _dbg_state_b}
                    if "CRUISE" in _dbg_states and len(_dbg_states.intersection({"QUEUING", "PREQUEUE", "LOADING"})) > 0:
                        for _dbg_agent in (_dbg_a, _dbg_b):
                            _dbg_state = getattr(getattr(_dbg_agent, "state", None), "name", None)
                            _dbg_agent.stop()
                            if _dbg_state == "CRUISE":
                                _dbg_agent.goal_changed = True
                                _dbg_agent.replan = True
                                if hasattr(_dbg_agent, "sequence_of_poses"):
                                    _dbg_agent.sequence_of_poses.clear()
            self._dbg_contact_steps = {pair: self._dbg_contact_steps.get(pair, 0) for pair in _dbg_active_contacts}
        # #endregion

        self.time = self.get_simulator_time() -  self.start_time
        self.task_count = sum([port.task_count for port in list(self.environment.unloading_ports.values())])
        Simulator.step_counter += 1
        if Simulator.step_counter % 10 == 0:
            self.heatmap_data.extend([(agent.position.y, agent.position.x) for agent in self.agents])
        if self.metrics_recorder:
            self.metrics_recorder.update()
        baseline_agents = [
            agent for agent in self.agents
            if getattr(getattr(agent, "global_planner", None), "__class__", None) is not None
            and type(agent.global_planner).__name__ == "LayeredAStarBaselineTrafficAware"
        ]
        if baseline_agents:
            collision_pairs = []
            for contact in self.world.contacts:
                if not contact.touching:
                    continue
                if contact.fixtureA.sensor or contact.fixtureB.sensor:
                    continue
                first = getattr(contact.fixtureA.body, "userData", None)
                second = getattr(contact.fixtureB.body, "userData", None)
                if not (hasattr(first, "id") and hasattr(second, "id")):
                    continue
                planners = {
                    type(getattr(first, "global_planner", None)).__name__,
                    type(getattr(second, "global_planner", None)).__name__,
                }
                if "LayeredAStarBaselineTrafficAware" not in planners:
                    continue
                collision_pairs.append(
                    {
                        "pair": sorted((getattr(first, "id", None), getattr(second, "id", None))),
                        "first": {
                            "id": getattr(first, "id", None),
                            "planner": type(getattr(first, "global_planner", None)).__name__,
                            "state": getattr(getattr(first, "state", None), "name", None),
                            "stopping_active": bool(getattr(first, "stopping_active", False)),
                            "stopping_for_agent_id": getattr(first, "stopping_for_agent_id", None),
                            "stopping_reason": getattr(first, "stopping_reason", None),
                            "position": [round(getattr(getattr(first, "position", None), "x", 0.0), 3), round(getattr(getattr(first, "position", None), "y", 0.0), 3)] if getattr(first, "position", None) is not None else None,
                        },
                        "second": {
                            "id": getattr(second, "id", None),
                            "planner": type(getattr(second, "global_planner", None)).__name__,
                            "state": getattr(getattr(second, "state", None), "name", None),
                            "stopping_active": bool(getattr(second, "stopping_active", False)),
                            "stopping_for_agent_id": getattr(second, "stopping_for_agent_id", None),
                            "stopping_reason": getattr(second, "stopping_reason", None),
                            "position": [round(getattr(getattr(second, "position", None), "x", 0.0), 3), round(getattr(getattr(second, "position", None), "y", 0.0), 3)] if getattr(second, "position", None) is not None else None,
                        },
                    }
                )
            if collision_pairs:
                # #region debug-point E:baseline-contact-captured
                self._debug_collision_event(
                    "E",
                    "simulator.py:step",
                    "[DEBUG] simulator captured baseline traffic aware physical contact",
                    {
                        "step": Simulator.step_counter,
                        "sim_time": round(self.time, 3),
                        "collision_pairs": collision_pairs[:4],
                    },
                )
                # #endregion
            active_stop_pairs = set()
            stop_summary = []
            bystander_candidates = []
            for agent in baseline_agents:
                blocker_id = getattr(agent, "stopping_for_agent_id", None)
                if getattr(agent, "stopping_active", False) and blocker_id is not None:
                    active_stop_pairs.add(tuple(sorted((getattr(agent, "id", None), blocker_id))))
                stop_summary.append(
                    {
                        "agent_id": getattr(agent, "id", None),
                        "state": getattr(getattr(agent, "state", None), "name", None),
                        "speed": round(float(getattr(agent, "speed", 0.0) or 0.0), 3),
                        "stopping_active": bool(getattr(agent, "stopping_active", False)),
                        "blocker_id": blocker_id,
                        "reason": getattr(agent, "stopping_reason", None),
                    }
                )
            for pair in sorted(active_stop_pairs):
                first = next((agent for agent in baseline_agents if getattr(agent, "id", None) == pair[0]), None)
                second = next((agent for agent in baseline_agents if getattr(agent, "id", None) == pair[1]), None)
                if first is None or second is None:
                    continue
                if (
                    getattr(first, "stopping_active", False)
                    and getattr(second, "stopping_active", False)
                    and getattr(first, "stopping_for_agent_id", None) == getattr(second, "id", None)
                    and getattr(second, "stopping_for_agent_id", None) == getattr(first, "id", None)
                ):
                    # #region debug-point A:double-stop-pair
                    self._debug_global_check_event(
                        "A",
                        "simulator.py:step",
                        "[DEBUG] detected reciprocal double-stop pair",
                        {
                            "step": Simulator.step_counter,
                            "sim_time": round(self.time, 3),
                            "pair": list(pair),
                            "agents": stop_summary,
                        },
                    )
                    # #endregion
            if active_stop_pairs:
                involved_ids = {agent_id for pair in active_stop_pairs for agent_id in pair}
                for agent in baseline_agents:
                    agent_id = getattr(agent, "id", None)
                    speed = float(getattr(agent, "speed", 0.0) or 0.0)
                    if agent_id in involved_ids or getattr(agent, "stopping_active", False):
                        continue
                    if speed > max(0.05, float(getattr(agent, "cruise_speed", 0.0) or 0.0) * 0.35):
                        continue
                    min_distance = min(
                        (
                            agent.position.distance(other.position)
                            for other in baseline_agents
                            if getattr(other, "id", None) in involved_ids and getattr(other, "position", None) is not None
                        ),
                        default=float("inf"),
                    )
                    if min_distance > 4.0:
                        bystander_candidates.append(
                            {
                                "agent_id": agent_id,
                                "state": getattr(getattr(agent, "state", None), "name", None),
                                "speed": round(speed, 3),
                                "cruise_speed": round(float(getattr(agent, "cruise_speed", 0.0) or 0.0), 3),
                                "min_distance_to_incident": round(min_distance, 3),
                            }
                        )
                if bystander_candidates:
                    # #region debug-point B:bystander-slowdown
                    self._debug_global_check_event(
                        "B",
                        "simulator.py:step",
                        "[DEBUG] detected bystander slowdown candidate",
                        {
                            "step": Simulator.step_counter,
                            "sim_time": round(self.time, 3),
                            "active_stop_pairs": [list(pair) for pair in sorted(active_stop_pairs)],
                            "bystanders": bystander_candidates,
                            "agents": stop_summary,
                        },
                    )
                    # #endregion
            if Simulator.step_counter % 10 == 0:
                # #region debug-point D:baseline-step-snapshot
                self._debug_global_check_event(
                    "D",
                    "simulator.py:step",
                    "[DEBUG] baseline step snapshot",
                    {
                        "step": Simulator.step_counter,
                        "sim_time": round(self.time, 3),
                        "task_count": int(self.task_count),
                        "agents": stop_summary,
                    },
                )
                # #endregion
        if self.time % 60 ==0 and self.task_count >0:
            print("Current PPH:", self.task_count/self.time*3600)

    """RayCast for sensor using"""
    def ray_cast_callback(self, agent_body, angle):
        ray_length = 4.0
        callback = SensorRayCast()
        position = agent_body.position
        cos_angle = cos(angle)
        sin_angle = sin(angle)
        dimension = agent_body.userData.shape.get_half_dimension()
        start_point = ((position[0] + dimension[0]*cos_angle), (position[1] + dimension[0]*sin_angle))
        end_point = ((position[0] + (dimension[0] + ray_length )*cos_angle), (position[1] + (dimension[0] + ray_length )*sin_angle))
        self.world.RayCast(callback, start_point, end_point)
        agent_body.userData.ray_point_list.append((start_point, end_point))
        if callback.hit:
            return self.__get_distance_between_points(callback.point, start_point)
        else:
            return ray_length


    def __get_distance_between_points(self, a, b):
        return sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]))

    def run(self, show_visualisation):
        if show_visualisation:
            # load visualisation config data
            with open('config.json') as file:
                config_data = json.load(file)
            vis = Visualisation(self, config_data)
            vis.run()
        else:
            target_simulation_steps = self.simulation_times * 60 /self.TIME_STEP
            print("---------------------------------------------------------------------")
            print(('The duration will be ' + str(self.simulation_times * 60)+ ' seconds'
                +' or '  + str(self.simulation_times) + ' minutes in simulation world'))
            print("---------------------------------------------------------------------")
            start_realworld_time = time.time()
            while(target_simulation_steps >= Simulator.step_counter):
                # print(target_simulation_steps , Simulator.step_counter)
                current_minute = int(self.get_simulator_time() // 60)
                if self.get_simulator_time() > 0 and current_minute != self.last_reported_minute and abs(self.get_simulator_time() % 60) < self.TIME_STEP:
                    self.last_reported_minute = current_minute
                    print((str(self.get_simulator_time()/60.0)+' minutes passed in simulation world'))
                    end_realworld_time = time.time()
                    print(int(end_realworld_time - start_realworld_time), "seconds passed in real world")
                    if self.get_simulator_time() > 0:
                        print("PPH is", float(self.task_count)/self.get_simulator_time()*3600, "now")
                    print("---------------------------------------------------------------------")
                self.step()
    def get_simulator_time(self):
        return Simulator.step_counter * self.TIME_STEP 

    def realworld_to_simulator_time(self, realworld_time):
        pass
    #################### simulation codes  #####################################
    def __update_agent_state(self, agent, observation):
        agent.observe(observation)
        agent.plan()
        agent.act()

def json_decoder_environment_data():
    with open('config.json') as file:
            data = json.load(file)
    return data

def print_heatmap(simulator, PPH):
    y,x =list(zip(*simulator.heatmap_data))
    heatmap, xedges, yedges = np.histogram2d(x, y, bins=(100,100))
    extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

    # Plot heatmap
    plt.clf()
    plt.title('System Heatmap\n Simulation time ' + str(simulator.get_simulator_time()) 
        + ' sec\nAgents number '+ str(len(simulator.agents)) + "  PPH: "+ str(PPH))
    plt.ylabel('X')
    plt.xlabel('Y')
    plt.text(0.5, -6,
            'Local Planner: '+str(simulator.agent_local_planner)+', Global planner: '+str(simulator.agent_global_planner), verticalalignment='center')
    plt.imshow(heatmap, extent=extent)
    plt.colorbar(orientation='vertical')
    plt.show()


def start_simulator(args, receive_q = None, send_q = None):
    cmd_args = process_cmd(args)
    show_visualisation = True if cmd_args.time == -1 else False # if a simulation time limit is given, do not invoke graphical display 
    if not show_visualisation:
        RRTStar.VISUAL = False
        MARRTStar.VISUAL = False
        INashRRT.VISUAL = False
    config_data = json_decoder_environment_data()
    Simulator.step_counter = 0
    Agent.counter = 0

    # Create a simulator
    simulator = Simulator(cmd_args)
    simulator.metrics_recorder = RunMetricsRecorder(
        simulator,
        cmd_args,
        config_data,
        enabled=(cmd_args.time != -1) or bool(getattr(cmd_args, "record_tag", None)) or bool(getattr(cmd_args, "record_dir", None)),
    )
    if receive_q and send_q:
        simulator.receive_q = receive_q
        simulator.send_q = send_q
        sys.stdout = send_q
    simulator.TIME_STEP=1.0/config_data['simulator']['steps_per_sec']
    h6_2_workspace = simulator.set_environment(cmd_args.obstacle)
    agents_number = config_data['agents']['number']
    agents_speed = config_data['agents']['cruise_speed']
    agent_angular_velocity = pi/config_data['agents']['pi_divide_by_max_angular_velocity']
    agents_dimension = config_data["agents"]["dimension"]
    agents=[]
    workspace_width = config_data['environment']['width_in_meters']
    workspace_height = config_data['environment']['height_in_meters']
    """
    2D workspace array -> 1d array
    ignore four sides because ports spawn on sides
    """
    gridmap = GridmapWithNeighbors(h6_2_workspace.static_gridmap)
    available_index_list = gridmap.available_index_list()
    gridmap.static_obstacle_inflation()

    # Set random agents spawn location seed
    spawn_agents_random_seed = 200
    random.seed(spawn_agents_random_seed)

    # Get random location of agent
    random_agent_spawn_location = random.sample(available_index_list,
        agents_number)

    # assign local and global planner according to cmd input
    general_local_planner = process_local_planner_cmd(cmd_args.local_planner)
    general_global_planner = process_global_planner_cmd(cmd_args.global_planner)
    general_task_manager = process_task_manager_cmd(getattr(cmd_args, "task_manager", None))

    # Create agents
    # The size of agents should be at least one gird
    for num in range(agents_number):
        j = int(random_agent_spawn_location[num] / workspace_width)
        i = random_agent_spawn_location[num] % workspace_width
        test_agent=NaiveAgent(shape=Rectangle(i, j, agents_dimension, agents_dimension),
            position=Point(i, j), speed = agents_speed, angular_velocity = agent_angular_velocity)
        test_agent.angularVelocity = 1
        test_agent.enable_stuck_recovery = getattr(cmd_args, "stuck_recovery", False)
        test_agent.queue_goal_replan_guard_distance = 0.8
        test_agent.sensor = Sensor(radius = 5, angle = pi, location = test_agent.shape.get_box2d_location(), shape = b2PolygonShape)
        agents.append(test_agent)

        if simulator.free_control:
            test_agent.state = AgentState.CRUISE
    
    # Create server
    server = Server(environment=h6_2_workspace, agents=agents, task_manager_class=general_task_manager) # server also connect to agents
    simulator.task_manager_name = type(server.task_manager).__name__
    # Connect agent to server & set planner
    for agent in agents:
        agent.connect_to_central_server(server)
        simulator.set_local_planner(agent, general_local_planner)
        simulator.set_global_planner(agent, general_global_planner)

    simulator.set_agents(agents)
    if not simulator.free_control:
        simulator.set_ports(h6_2_workspace.unloading_ports)
        simulator.set_ports(h6_2_workspace.loading_ports)
        simulator.loading_ports = list(h6_2_workspace.loading_ports.values())
        simulator.unloading_ports = list(h6_2_workspace.unloading_ports.values())
    simulator.set_walls(h6_2_workspace.walls)
    simulator.set_obstacles(h6_2_workspace.obstacles)
    
    simulator.ini_perception()

    for port in list(simulator.environment.loading_ports.values()):
        for i in range(100):
            port.get_random_item(simulator.environment.num_unloading_ports)
    
    simulator.run(show_visualisation)
    if simulator.metrics_recorder:
        simulator.latest_run_summary = simulator.metrics_recorder.finalize()


    print(("Time in seconds:",simulator.time))
    print(("Number of Packages Delivered:", simulator.task_count))
    if simulator.time > 0:
        print(("PPH:", float(simulator.task_count)/simulator.time*3600))
    #PPH = simulator.task_count/simulator.time*3600
    #print("PPH(Packages Per Hour: ", PPH)
    #print_heatmap(simulator, PPH)

if __name__ == "__main__":
    args = create_cmd_parser().parse_args()
    start_simulator(args)
