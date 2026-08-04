"""
@date : 2018-07
@author : {xiaoyu.ge, xiao.tian}@dorabot.com
@brief : Vechile's queuing system
"""
from pygame import Rect
from geometry import Point
from collections import deque
class SimpleQueue:
    """ This class implements a simple queuing system with a single lane
    Attributes:
    agent_slot_map -- a hash map: agent_id ---> assigned_slot
    """
    def __init__(self, region):
        # Create rectangle region
        self.region = region
        # Discretise the space into grids
        self.length = 0
        self.agents= []
        self.agent_slot_map = {}
        self.__setup_slots()

    def num_agents(self):
        return len(self.agents)

    def enter(self,agent):

        self.length = self.length + 1
        self.agents.append(agent)
        self.__arrange_slots()
        return self.length-1

    def exits(self, agent=None):
        if len(self.agents) == 0:
            self.length = 0
            self.agent_slot_map = {}
            return

        exiting_agent = agent
        if exiting_agent is None:
            exiting_agent = self.agents[0]

        self.agents = [queued_agent for queued_agent in self.agents if queued_agent.id != exiting_agent.id]
        self.length = len(self.agents)
        self.agent_slot_map.pop(exiting_agent.id, None)
        self.__arrange_slots()
        return

    def get_slot(self, agent):
        """ All Agents will be sorted each time an agent enters or exists from the queue
        """
        return self.agent_slot_map[agent.id]
    
    def __setup_slots(self):
        slot_side_length = 1
        num_of_slots = int(self.region.width/slot_side_length) + 1
        self.min_dist_to_port = 1
        y = self.region.y + self.region.height/2
        x = self.region.x 
        self.slots = [Point(x + i * slot_side_length, y) for i in range(num_of_slots)]
        
    def __arrange_slots(self, slots_reverse = False):
        # 队列采用 FIFO：
        # 一旦 agent 确认进入 queue，就保持进入顺序分配槽位，
        # 避免后进入者因为“当前更靠近前槽”而被重新排到前面，导致抢位和交叉。
        if slots_reverse:
            ordered_agents = list(reversed(self.agents))
        else:
            ordered_agents = list(self.agents)
        assert len(self.agents) <= len(self.slots)
        self.agent_slot_map = {}
        for i, agent in enumerate(ordered_agents):
            self.agent_slot_map[agent.id] = self.slots[i]
