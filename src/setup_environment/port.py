"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {tian.xiao, xiaoyu.ge}@dorabot.com
@brief : Implementation of ports
"""
from setup_environment.obstacles import Obstacles
import json
from math import pi

""" Ports are created by their location, dimension and identifier.
    The default identifier should be the counter from 0.
"""
class Port(Obstacles):
    simulator_step = 0
    """Ports location is the Top Left corner of the port
       Ports dimension should be a tuple with (width,height)
    """
    def __init__(self, location, dimension, port_process_time, identifier):
        super(Port, self).__init__(location, dimension, identifier)
        self.operation_time_in_secs = port_process_time
        self.type = 'port'
        self.items = []
        self.control_range = 2
        self.operation_range = 0.75
        self.operation_entry_tolerance = 0.2
        """ Queuing Related """
        self.queue = None
        self.has_queue_area = False
        """ Operation Related """
        self.operation_start_time = None
        self.in_operation = False
        self.operation_zone = None
        """ Logging Related """
        self.task_count = 0
        self.simulator_time_step = 1.0/60
        """future using""" 
        self.entry_point = location
        self.__get_time_step()

    def __get_time_step(self):
        try:
            with open('config.json') as file:
                config_data = json.load(file)
            self.simulator_time_step = 1.0/config_data['simulator']['steps_per_sec']
        except:
            pass

    def enforce_queuing_mechanism(self, queuing_mechanism):
        self.queue = queuing_mechanism
        self.operation_zone = self.queue.slots[0]
        self.entry_point = self.queue.slots[-1]

    # ================ External Communication =================================
    def query_num_existing_agents(self):
        return self.queue.num_agents()

    def request_enter_permit(self):
        # An agent entered into the region
      if len(self.queue.agents) < len(self.queue.slots):
          return True, self.queue.num_agents()
      else:
          return False, self.queue.num_agents()

    def confirm_enter(self, agent):
        self.queue.enter(agent)

    def confirm_exit(self, agent=None):
        self.queue.exits(agent)
    def get_slot(self, agent):
        return self.queue.get_slot(agent)
    def operate(self):
        is_done = False
        if self.in_operation:
            if (Port.simulator_step  * self.simulator_time_step) - self.operation_start_time > self.operation_time_in_secs:
                self.in_operation = False
                is_done = True
                self.task_count += 1
        else:
            self.in_operation = True
            self.operation_start_time = Port.simulator_step  * self.simulator_time_step
        return is_done

    # ================ Local Computation =================================
    def in_control_range(self, agent_position):
        if self.queue:
            dist = min(
                [slot.distance(agent_position) for slot in self.queue.slots])
            #dist=abs(self.center.y-agent_position.y)+min([abs(slot.x-agent_position.x) for slot in self.queue.slots])
        else:
            dist = self.location.distance(agent_position)
        return dist < self.control_range

    def in_operation_zone(self, agent_position):
        return self.operation_zone.distance(agent_position) < self.operation_range

    def is_queue_head(self, agent):
        queue_agents = getattr(getattr(self, "queue", None), "agents", None) or []
        if len(queue_agents) == 0 or agent is None:
            return False
        return getattr(queue_agents[0], "id", None) == getattr(agent, "id", None)

    def is_assigned_to_operation_zone(self, agent, tolerance=1e-3):
        if agent is None or self.operation_zone is None:
            return False
        try:
            slot = self.get_slot(agent)
        except Exception:
            return False
        return slot is not None and slot.distance(self.operation_zone) <= tolerance

    def is_at_operation_point(self, agent_position, tolerance=None):
        if self.operation_zone is None or agent_position is None:
            return False
        if tolerance is None:
            tolerance = self.operation_entry_tolerance
        return self.operation_zone.distance(agent_position) <= tolerance

    def _operation_zone_is_clear_for(self, agent, clearance=None):
        perception = getattr(agent, "perception_module", None)
        if perception is None or self.operation_zone is None:
            return True
        if clearance is None:
            own_radius = float(getattr(getattr(agent, "shape", None), "get_radius", lambda: 0.5)() or 0.5)
            clearance = max(0.9, own_radius * 2.0 + 0.12)
        try:
            observed_agents = perception.other_agents_state_in_range_of(clearance, 2 * pi)
        except Exception:
            return True
        for observed_agent in observed_agents:
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) == getattr(agent, "id", None):
                continue
            observed_position = getattr(observed_state, "position", None)
            if observed_position is None:
                continue
            if observed_position.distance(self.operation_zone) <= clearance:
                return False
        return True

    def can_agent_start_operation(self, agent):
        if agent is None:
            return False
        agent_position = getattr(agent, "position", None)
        return (
            self.is_queue_head(agent)
            and self.is_assigned_to_operation_zone(agent)
            and self.is_at_operation_point(agent_position)
            and self._operation_zone_is_clear_for(agent)
        )

    def has_active_operation_owner(self, agent=None):
        queue_agents = getattr(getattr(self, "queue", None), "agents", None) or []
        for queued_agent in queue_agents:
            if agent is not None and getattr(queued_agent, "id", None) == getattr(agent, "id", None):
                continue
            state_name = getattr(getattr(queued_agent, "state", None), "name", None)
            position = getattr(queued_agent, "position", None)
            if state_name in {"LOADING", "UNLOADING"} and self.is_at_operation_point(position):
                return True
        return False


    def is_clicked(self, pos):
        return   self.location.x <= pos[0] <= self.location.x + self.dimension[0] and self.location.y <= pos[1] <= self.location.y + self.dimension[1]
