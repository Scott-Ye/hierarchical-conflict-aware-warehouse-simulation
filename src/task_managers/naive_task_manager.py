"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : Implementation of a task manager using naive strategy
"""
from task_managers.task_manager import *
class NaiveTaskManager(TaskMananger):

    def __init__(self, environment, agents):
        super(NaiveTaskManager, self).__init__(environment, agents)
        self.longest_squared_distance = environment.width_in_meters * environment.width_in_meters + environment.height_in_meters * environment.height_in_meters
        self.num_agents = len(self.agents)
        self.num_loading_ports = len(self.loading_ports)
    # Local computation
    def get_unloading_task(self, agent, item):
        port = self.unloading_ports[item.destination_port_id]
        #self.record[agent.id]["task"] = (port.id, "u")
        return UnLoadingTask(agent.position, port.entry_point, port)

    def choose_loading_port(self, agent, agents_state, longest_squared_distance):
        def evaluate_port(loading_port):
            counter1 = 0
            counter2 = 0
            for agent_id in agents_state:
                state_info = agents_state[agent_id]
                task = state_info.get("task")
                if getattr(task, "port", None) == loading_port:
                    counter1 += 1
                    if getattr(state_info.get("state"), "name", None) in {"QUEUING", "PREQUEUE"}:
                        counter2 += 1
            # baseline task manager 的逻辑比较直接：
            # 尽量少去已经有人目标指向、或者已经开始排队的 loading port，
            # 同时轻微考虑与该 port 的距离。
            return -(counter1 + 3 * counter2) - 0.3 * loading_port.location.squared_distance(agent.position) / self.longest_squared_distance

        return max(self.loading_ports, key=evaluate_port)
    
    def next_loading_task(self, agent, port):
        # find the next IDEL and closest port.
        # each port gets a score
        #def evaluate_port(loading_port):
            # used weighted formula to measure the score
            #shortage=len(loading_port.items)-loading_port.queue.num_agents()
            #if loading_port.queue.num_agents()==len(loading_port.queue.slots):
            #    return -500
            #if shortage < 0:
            #    return -1000
            #else:
            #return -loading_port.queue.num_agents()/self.num_agents - 0.3*loading_port.location.squared_distance(agent.position)/self.longest_squared_distance
            
        #    return -self.record.values().count((loading_port.id, "l"))/self.num_agents*self.num_loading_ports\
        #            -0.1*loading_port.location.squared_distance(agent.position)/self.longest_squared_distance
            
            
        #port = max(self.loading_ports, key=evaluate_port)
        #self.record[agent.id]["task"] = (port.id, "l")
        
        return LoadingTask(agent.position, port.entry_point, port)
