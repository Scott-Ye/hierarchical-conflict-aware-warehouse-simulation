"""
 @Copyright Dorabot Inc.
 @date : 2018-07
 @author : {xiaoyu.ge, chong.chen2, tian.xiao}@dorabot.com
 @brief : Template implementation of an agent
"""
from enum import Enum
from geometry import Point, compute_direction
from representation.gridmap import GridMap
from task_managers.task_manager import Task, WaitingForOrderTask
from collections import deque
from math import cos, sin, pi, atan2, sqrt
from .agent_state_machine import AgentStateMachine

class SimulatedPerception:
    """ A simulated module emulating sensor and server updates
    Keyword arguments:
        simulated_agent: the simulated agent where this module resides
        all_agents: all the simulated agents
    """
    def __init__(self, simulated_agent, all_simulated_agents, all_ports):
        self.simulated_agent = simulated_agent
        self.all_ports = all_ports
        self.other_simulated_agents = [agent for agent in all_simulated_agents if agent.userData.id != simulated_agent.userData.id]

    """ Localisation module
    add noise if necessary
    """
    def localization(self):
        position = self.simulated_agent.position
        angle = self.simulated_agent.angle
        return [position[0], position[1], angle]
    """ Simulate local observation
    Keyword arguments:
    radius -- simulated radius of the lidar with the active range of 2 * pi
    """
    def other_agents_state_in_range_of(self, radius = 3):
        return [agent for agent in self.other_simulated_agents if self.simulated_agent.userData.position.distance(agent.userData.position) <= radius]
    def ports_in_range_of(self, radius = 2):
        return [port for port in self.all_ports if self.simulated_agent.userData.position.distance(port.userData.location) <= radius]

"""Perception from sensor on agent only"""
class Box2DPerception:
    def __init__(self, simulated_agent):
        self.simulated_agent = simulated_agent
        self.detected_object = self.simulated_agent.userData.sensor.visible_object
        self.ray_length_list = self.simulated_agent.userData.ray_length_list

    def localization(self):
        position = self.simulated_agent.position
        angle = self.simulated_agent.angle
        return [position[0], position[1], angle]
    
    def other_agents_state_in_range_of(self, radius = 3.0, angle = pi):
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        agents = [agent for agent in self.detected_object if agent.userData.type == 'agent']
        agents_in_angle = [agent for agent in agents if abs(atan2(agent.position.y-reference_point.y, agent.position.x-reference_point.x)
            -reference_angle) <= angle]
        return [agent for agent in agents_in_angle if self.simulated_agent.userData.position.distance(agent.position) <= radius]    
            
    def ports_in_range_of(self, radius = 2.0, angle = pi):
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        ports = [port for port in self.detected_object if port.userData.type == 'port']
        ports_in_angle = [port for port in ports if abs(atan2(port.userData.location.y-reference_point.y, port.userData.location.x-reference_point.x)
            -reference_angle) <= angle]
        return [port for port in ports_in_angle if self.simulated_agent.userData.position.distance(port.position) <= radius]
            
    def walls_in_range_of(self, radius = 2.0, angle = pi):
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        walls = [wall for wall in self.detected_object if wall.userData.type == 'wall']
        walls_in_angle = [wall for wall in walls if abs(atan2(wall.userData.location.y-reference_point.y, wall.userData.location.x-reference_point.x)
            -reference_angle) <= angle]
        return [wall for wall in walls_in_angle if self.simulated_agent.userData.position.distance(wall.userData.location) <= radius]

    def other_agents_state_in_collision_of(self, tolerrance = 2e-1, angle = pi):
        '''take both dimensions into consideration, if the distance between the minimum distance of two is smaller than a tolerance '''
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        agents = [agent for agent in self.detected_object if agent.userData.type == 'agent']
        agents_in_angle = [agent for agent in agents if abs(atan2(agent.position.y-reference_point.y, agent.position.x-reference_point.x)
            -reference_angle) <= angle]
        return [agent for agent in agents_in_angle if self.simulated_agent.userData.position.distance(agent.position) <= tolerrance + self.simulated_agent.userData.shape.get_radius() + agents_in_angle.userData.shape.get_radius()]    
            
    def ports_in_collision_of(self, tolerrance = 2e-1, angle = pi):
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        ports = [port for port in self.detected_object if port.userData.type == 'port']
        ports_in_angle = [port for port in ports if abs(atan2(port.userData.location.y-reference_point.y, port.userData.location.x-reference_point.x)
            -reference_angle) <= angle]
        return [port for port in ports_in_angle if self.simulated_agent.userData.position.distance(port.position) <= tolerrance + self.simulated_agent.userData.shape.get_radius() + port.userData.get_radius()]
            
    def walls_in_collision_of(self, tolerrance = 2e-1, angle = pi):
        reference_point = self.simulated_agent.userData.position
        reference_angle = self.simulated_agent.userData.angle
        walls = [wall for wall in self.detected_object if wall.userData.type == 'wall']
        walls_in_angle = [wall for wall in walls if abs(atan2(wall.userData.location.y-reference_point.y, wall.userData.location.x-reference_point.x)
            -reference_angle) <= angle]
        for wall in walls_in_angle:
            print("test:", pow(min(wall.userData.dimension)**2.0, 0.5), self.simulated_agent.userData.shape.get_radius())
        return [wall for wall in walls_in_angle if self.simulated_agent.userData.position.distance(wall.userData.location) <= tolerrance + self.simulated_agent.userData.shape.get_radius() + pow(min(wall.userData.dimension)**2.0*2.0, 0.5) / 2.0]


# Agent Template
class Agent(object):
    counter = 0
    """
    Keyword arguments:
        static_environment -- environment information, (default to GridMap struct)
    """
    def __init__(self, shape, position, static_environment=None, id=counter, speed = 2, angular_velocity = pi/2):
        self.id = Agent.counter
        self.type = 'agent'
        Agent.counter = Agent.counter + 1
        self.shape = shape
        self.cruise_speed = speed
        self.max_angular_velocity = angular_velocity
        self.linear_velocity = (0, 0)
        self.angular_velocity = 0
        self.speed = 0
        self.angle = 0
        self.position = position
        self.task = WaitingForOrderTask()
        self.potential_collision = False
        self.destination_location= None
        self.state_machine = AgentStateMachine(self)
        self.server_command = None
        self.server = None
        self.perception_module = None
        self.goal_changed = False
        self.local_planner = []
        self.current_local_planner = None
        self.global_planner = None
        self.static_environment = static_environment
        self.sensor = None
        self.ray_length_list = []
        self.ray_point_list = []
        self.history_ray_length_list = []
        self.history_ray_point_list = []
        self.sequence_of_poses = deque()
        self.replan = False
        self.internal_stations = []
        self.motion_output_type = "velocity_vector"
        self.distance_travelled = 0.0
        self.global_plan_calls = 0
        self.replan_events = 0
        self.last_replan_reason = None
        self.stopping_active = False
        self.stopping_base_state = None
        self.stopping_for_agent_id = None
        self.stopping_reason = None


    def ini_perception_module(self, simulated_agent, all_simulated_agents, all_ports):
        """ Initialise a simulated perception module to the agent
        Keyword arguments:
            simulated_agent -- the simulated physical body of the agent
            all_simulated_agents -- the simulated physical bodies of all the agents in simulator
            all_ports -- the simulated physical bodies of all the ports(include unloading and loading)
        """
        #self.perception_module = SimulatedPerception(simulated_agent, all_simulated_agents, all_ports)
        self.perception_module = Box2DPerception(simulated_agent)
    def connect_to_central_server(self, server):
        """
        Keyword arguments:
            server -- simulate an interaction with the server
        """
        self.server = server
    def use_local_planner(self, local_planner):
        # self.local_planner.append(local_planner)
        self.local_planner= [local_planner]
    def use_global_planner(self, global_planner):
        self.global_planner = global_planner
    def assign_task(self, task):
        self.task = task

    def observe(self, ray_length_list):
        """ Observation
        - Obtain updated map (if necessary)
        - Obtain sensor data
        - Obtain server commands (high-priority command such as emergent halt)
        """
        raise NotImplementedError

    def plan(self):
        """
        - call global planner first
        - self.global_planner.input(self.static_environment,
        - start_pose, goal_pose)
        - sequence_of_poses = self.global_planner.result()
        - point = way_points[0]
        - self.local_planner.input(self.static_environment,
        - self.senor_data, sequence_of_poses)
        - velocity, local_path = self.local_planner.result()
        """
        raise NotImplementedError

    def act(self):
        raise NotImplementedError

    def in_position(self):
        return self.position.distance(self.destination_location)<1e-1
    def stop(self):
        self.linear_velocity = (0,0)
        self.speed = 0
        self.angular_velocity = 0
        self.motion_output_type = "velocity_vector"

    def snap_to_position(self, target_position):
        if target_position is None:
            return
        self.stop()
        self.position = target_position.copy() if hasattr(target_position, "copy") else Point(target_position.x, target_position.y)
        simulated_agent = getattr(getattr(self, "perception_module", None), "simulated_agent", None)
        if simulated_agent is None:
            return
        try:
            simulated_agent.position = self.position.to_tuple()
            simulated_agent.linearVelocity = (0.0, 0.0)
            simulated_agent.angularVelocity = 0.0
        except Exception:
            pass

    def apply_local_plan(self, command):
        output_type = getattr(self.current_local_planner, "OUTPUT_TYPE", "velocity_vector")
        if output_type == "differential_drive":
            try:
                speed, angular_velocity = command
            except (TypeError, ValueError):
                speed, angular_velocity = 0.0, 0.0
            self.speed = float(speed)
            self.angular_velocity = float(angular_velocity)
            self.linear_velocity = (
                cos(self.angle) * self.speed,
                sin(self.angle) * self.speed,
            )
            self.motion_output_type = output_type
            return

        try:
            vx, vy = command
        except (TypeError, ValueError):
            vx, vy = 0.0, 0.0
        self.linear_velocity = (float(vx), float(vy))
        self.speed = sqrt(self.linear_velocity[0] ** 2 + self.linear_velocity[1] ** 2)
        self.angular_velocity = 0.0
        self.motion_output_type = output_type
        
    """================= accessors ========================================="""
    def get_id(self):
        return self.id
    def get_linear_velocity(self):
        return self.linear_velocity
    def has_destination(self):
        return self.destination_location != None
    """================= helper methods ====================================="""
    def set_next_way_point(self):
        if len(self.way_points) > 0:
            self.next_way_point = self.way_points.popleft()
            return True
        else:
            return False
    #========================== whether in position ============================
    """Only for simulator using
       Check the agent get clicked
    """
    def is_clicked(self, pos):
        '''agent update its position after localization in step(); in pause mode, no step() is executed, thus percepted position is returned instead of agent self-recorded position'''
        dimension = self.shape.get_half_dimension()
        percepted_position = self.perception_module.simulated_agent.position
        # return   self.position.x - dimension[0]<= pos[0] <= self.position.x + dimension[0] and self.position.y - dimension[1] <= pos[1] <= self.position.y + dimension[1] 
        return   percepted_position.x - dimension[0]<= pos[0] <= percepted_position.x + dimension[0] and percepted_position.y - dimension[1] <= pos[1] <= percepted_position.y + dimension[1] 


    def is_in_range(self, pos_start, pos_end):
        low_pos = (min(pos_start[0], pos_end[0]) , min(pos_start[1], pos_end[1]))
        up_pos = (max(pos_start[0], pos_end[0]) , max(pos_start[1], pos_end[1]))
        # return  low_pos[0] <= self.position.x <= up_pos[0] and low_pos[1] <= self.position.y <= up_pos[1]
        percepted_position = self.perception_module.simulated_agent.position
        return  low_pos[0] <= percepted_position.x <= up_pos[0] and low_pos[1] <= percepted_position.y <= up_pos[1]
