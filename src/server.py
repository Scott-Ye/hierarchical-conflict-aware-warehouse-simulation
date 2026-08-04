"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : emulates a central server that deploys a task manager and a multiple
path finding algorithm
the design philosophy is to minimise the computation on the server side: we
let agent compute as much as it can.
"""
import copy
from task_managers.task_manager import TaskMananger
from task_managers.naive_task_manager import NaiveTaskManager
from agents.agent_state_machine import AgentState
from setup_environment.port import Port
class Server:
    def __init__(self, environment, agents, task_manager_class=None):
        # server 负责接入当前实验使用的 task manager。
        # 这样 task manager 可以像 global planner 一样，按版本独立切换和展示。
        self.agents = agents # server connect to agent
        task_manager_class = task_manager_class or NaiveTaskManager
        self.task_manager = task_manager_class(environment=environment, agents=agents)
        self.agents_state = {agent.id: dict() for agent in agents}
        self.loading_ports = list(environment.loading_ports.values())
        self.unloading_ports = list(environment.unloading_ports.values())
        self.longest_squared_distance = environment.width_in_meters * environment.width_in_meters + environment.height_in_meters * environment.height_in_meters
        
        self.multiagent_global_planners = []

    def get_loading_task(self, agent):
        # 与原始实现不同，这里把“选哪个 loading port”显式委托给 task manager。
        # 因此 CongestionAwareTaskManager 这类补充分支，只需要重写 choose_loading_port()
        # 就能接到完整仿真流程里，而不用重写整个 server。
        port = self.task_manager.choose_loading_port(
            agent,
            self.agents_state,
            self.longest_squared_distance,
        )
        task = self.task_manager.next_loading_task(agent, port)
        self.update_data(agent)
        return task
    
    def get_unloading_task(self, agent, item):
        task = self.task_manager.get_unloading_task(agent, item)
        self.update_data(agent)
        return task
    
    def update_data(self, agent):
        self.agents_state[agent.id]['task'] = agent.task
        self.agents_state[agent.id]['state'] = agent.state
    
    # calls for multiagent_global_planner
    def add_multiagent_local_planner(self, ma_planner):
        self.multiagent_global_planners.append(ma_planner)
    
    def remove_multiagent_local_planner(self, ma_planner):
        for i in range(len(self.multiagent_global_planners)):
            if ma_planner == self.multiagent_global_planners[i]:
                self.multiagent_global_planners.pop(i)

    def request_multiagent_global_planner_compute_path(self, agent):
        '''Agent sends a request to server, asking for multiagent_global_planner to compute path for itself'''
        for ma_planner in self.multiagent_global_planners:
            if agent.id in list(ma_planner.agents.keys()): # find the ma_planner which in charge of current agent
                current_step = getattr(Port, "simulator_step", None)
                if ma_planner.cached_solution_step != current_step or ma_planner.cached_solution_paths is None:
                    solution_paths_dict = ma_planner.compute_path() or {}
                    ma_planner.cached_solution_paths = solution_paths_dict
                    ma_planner.cached_solution_step = current_step
                solution_paths_dict = ma_planner.cached_solution_paths
                return solution_paths_dict.get(agent.id, [])
        print("Cannot find a multiagent global planner which is in charge of agent {}".format(agent.id))
        return []
            
    def collect_agents_info(self, agents_dict):
        """return dictionary of {agent.id: AgentAbstraction} for multiagent planner"""
        agents_abstraction = {}
        for agent in list(agents_dict.values()):
            if agent.has_destination():
                agent.observe(agent.ray_length_list) # refresh localization
                agents_abstraction[agent.id] = AgentAbstraction(agent)
        return agents_abstraction

    def refresh_agents_path(self, solution_paths_dict):
        """a work around for pass the paths to all agents"""
        for agent_id in list(solution_paths_dict.keys()):
            agent = self.agents[agent_id]
            if agent.has_destination():
                agent.sequence_of_poses = solution_paths_dict[agent_id]
    
class AgentAbstraction(object):
    """get necessary information of an agent for multiagent planner"""
    def __init__(self, agent):
        self.id = agent.id
        self.position = agent.position.copy()
        self.radius = agent.shape.get_radius()
        self.destination_location = agent.destination_location
        self.sequence_of_poses = copy.deepcopy(agent.sequence_of_poses)
        if agent.current_local_planner:
            self.current_local_planner = agent.current_local_planner
        else:
            self.current_local_planner = agent.local_planner[0]
        self.root_linear_velocity = agent.linear_velocity # tuple
        self.root_angular_velocity = agent.angular_velocity # scala
        self.perception_module = agent.perception_module
        self.root_history_ray_length_list = agent.history_ray_length_list[:]
        self.root_history_ray_point_list = agent.history_ray_point_list[:]

    def __str__(self):
        return "agent {}; position {}; destination {};\nsequence_of_poses {};\nlocal planner {}; linear velocity {}; angular velocity {}"\
            .format(self.id, self.position, self.destination_location, 
            [str(pose) for pose in self.sequence_of_poses],
            self.current_local_planner, self.root_linear_velocity, self.root_angular_velocity)
