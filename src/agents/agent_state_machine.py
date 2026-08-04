"""
@Copyright Dorabot Inc.
@date : 2018-07
@author : xiaoyu.ge@dorabot.com
@brief : Quick Impelmentation of agent state machine for python environment
States and State transitions are hardcoded for a specific logic
@TODO Gary: Not urgent, Load configuration from json.
"""
from enum import Enum
from task_managers.task_manager import TaskType

class AgentState(Enum):
    IDLE = 0 # stopped and no task assigned
    LOADING = 1 # do not distinguish loading/unloading state
    QUEUING = 2
    HALT = 3 # stopped during exeucting a task
    CRUISE = 4
    STOPPING = 5 # temporarily stopped to avoid a predicted collision
    PREQUEUE= 6

class AgentStateMachine:
    """ A sample implementation of agent state machine """
    def __init__(self, agent):
        self.states = [
                    AgentState.IDLE,
                    AgentState.LOADING,
                    AgentState.QUEUING,
                    AgentState.HALT,
                    AgentState.CRUISE,
                    AgentState.STOPPING,
                    AgentState.PREQUEUE]
        agent.state = AgentState.IDLE
        self.mobile_states = [
                AgentState.CRUISE,
                AgentState.STOPPING]
    def next_state(self, agent, server_command=None):
        """ State transition function
        delta: Q X Sigma -> Q
        Sigma = {perception, agent_information, server_command}

        Return:
            a function type and a function to be called in q \in Q
        """
        if self.is_mobile_state(agent.state):
            if agent.task.type == TaskType.GO_TO_LOADING_PORT or agent.task.type == TaskType.GO_TO_UNLOADING_PORT:
                if agent.task.port.in_control_range(agent.position):
                    return approaching
            """ As long as it is in a mobile state, it must has a next goal destination """
            if self.arrive_at_destination(agent.position, agent.destination_location):
                agent.stop()
                return go_for_next_pose
        else:
            if agent.state == AgentState.PREQUEUE:
                return approaching
            if agent.state == AgentState.QUEUING:
                if (
                    hasattr(agent.task.port, "can_agent_start_operation")
                    and agent.task.port.can_agent_start_operation(agent)
                ):
                    return start_loading
                else:
                    return move_if_next_slot_available
            if agent.state == AgentState.LOADING:
                return operate
            if agent.state == AgentState.IDLE:
                return go_for_next_loading_task
        return stay_in_current_state
    def is_mobile_state(self, state):
        return state in self.mobile_states
    def arrive_at_destination(self, position, destination):
        if destination == None:
            return True
        if position.distance(destination) < 2e-1:
            return True
        return False

""" Functions
    Helper function an agent should call when tranisting to a new state
    Function will peform certain jobs and complete state transition in the end
"""
""" ================== Server-required (remote) functions ================== """
def go_for_next_loading_task(agent, server):
    task = server.get_loading_task(agent)
    agent.assign_task(task)
    agent.destination_location = task.destination_location
    agent.state = AgentState.CRUISE
    server.update_data(agent)
    agent.goal_changed = True


def _debug_same_port_exit_event(agent, hypothesis_id, location, message, data):
    return


def approaching(agent, server):
    """ Port could be one type of server that is in reminiscent to a control tower in an airport
    We should further decompose the state
    """
    # print agent.position, agent.task.port.location
    allowed, num_agents_ahead = agent.task.port.request_enter_permit()
    if allowed:
        agent.task.port.confirm_enter(agent)
        agent.state = AgentState.QUEUING
        move_if_next_slot_available(agent, server)
        if agent.destination_location is None:
            agent.destination_location = agent.task.port.get_slot(agent)
        # agent.goal_changed = True
        server.update_data(agent)    
    else:
        """ Not enough slots, enter the state of PREQUEUE """
        agent.state = AgentState.PREQUEUE
        server.update_data(agent)
        # agent.destination_location = None

def operate(agent, server):
    operation_is_done, item = agent.task.port.operate()
    if operation_is_done:
        agent.task.port.confirm_exit(agent)
        if agent.task.type == TaskType.GO_TO_LOADING_PORT:
            agent.assign_task(server.get_unloading_task(agent, item))
        elif agent.task.type == TaskType.GO_TO_UNLOADING_PORT:
            agent.assign_task(server.get_loading_task(agent))
        else:
            raise Exception(" Unclassifed operation types ")
        agent.state = AgentState.CRUISE
        server.update_data(agent)
        agent.goal_changed = True

def move_if_next_slot_available(agent, server):
    """ The agent is supposed to query the port that could be either deployed at the
    central server or a local server near the port
    """
    slot = agent.task.port.get_slot(agent)
    operation_point = getattr(agent.task.port, "operation_zone", None)
    operation_zone_clear = True
    if (
        slot is not None
        and operation_point is not None
        and slot.distance(operation_point) <= 1.05
        and hasattr(agent.task.port, "_operation_zone_is_clear_for")
    ):
        operation_zone_clear = agent.task.port._operation_zone_is_clear_for(agent)
    front_queue_hold = (
        slot is not None
        and operation_point is not None
        and slot.distance(operation_point) <= 1.05
        and (
            (
                hasattr(agent.task.port, "has_active_operation_owner")
                and agent.task.port.has_active_operation_owner(agent)
            )
            or not operation_zone_clear
        )
    )
    # #region debug-point D:front-queue-slot-check
    if getattr(agent, "id", None) == 5 and slot is not None and operation_point is not None:
        nearby_front_agents = []
        perception = getattr(agent, "perception_module", None)
        if perception is not None:
            try:
                for observed_agent in perception.other_agents_state_in_range_of(3.5, 6.283185307179586):
                    observed_state = getattr(observed_agent, "userData", observed_agent)
                    if getattr(observed_state, "id", None) == getattr(agent, "id", None):
                        continue
                    observed_position = getattr(observed_state, "position", None)
                    if observed_position is None or observed_position.distance(operation_point) > 1.6:
                        continue
                    nearby_front_agents.append({
                        "id": getattr(observed_state, "id", None),
                        "state": getattr(getattr(observed_state, "state", None), "name", None),
                        "task": getattr(getattr(getattr(observed_state, "task", None), "type", None), "name", None),
                        "target_port_id": getattr(getattr(getattr(observed_state, "task", None), "port", None), "id", None),
                        "distance_to_operation": round(observed_position.distance(operation_point), 3),
                        "position": [round(getattr(observed_position, "x", 0.0), 3), round(getattr(observed_position, "y", 0.0), 3)],
                    })
            except Exception:
                nearby_front_agents = [{"id": "sensor-error"}]
        _debug_same_port_exit_event(
            agent,
            "D",
            "agent_state_machine.py:move_if_next_slot_available",
            "[DEBUG] front queue slot check evaluated same-port exit occupancy",
            {
                "agent_id": getattr(agent, "id", None),
                "state": getattr(getattr(agent, "state", None), "name", None),
                "port_id": getattr(getattr(agent, "task", None).port, "id", None) if getattr(agent, "task", None) is not None else None,
                "slot": [round(getattr(slot, "x", 0.0), 3), round(getattr(slot, "y", 0.0), 3)],
                "operation_point": [round(getattr(operation_point, "x", 0.0), 3), round(getattr(operation_point, "y", 0.0), 3)],
                "distance_slot_to_operation": round(slot.distance(operation_point), 3),
                "front_queue_hold": front_queue_hold,
                "operation_zone_clear": operation_zone_clear,
                "has_active_operation_owner": bool(
                    hasattr(agent.task.port, "has_active_operation_owner")
                    and agent.task.port.has_active_operation_owner(agent)
                ),
                "nearby_front_agents": nearby_front_agents,
            },
        )
    # #endregion
    if front_queue_hold:
        # #region debug-point C:front-queue-hold
        debug_event = getattr(agent, "_debug_multi_stop_event", None)
        if callable(debug_event):
            debug_event(
                "C",
                "agent_state_machine.py:move_if_next_slot_available",
                "[DEBUG] multi-stop front queue hold kept agent at current position",
                {
                    "agent_id": getattr(agent, "id", None),
                    "state": getattr(getattr(agent, "state", None), "name", None),
                    "port_id": getattr(getattr(agent, "task", None).port, "id", None) if getattr(agent, "task", None) is not None else None,
                    "slot": [
                        round(getattr(slot, "x", 0.0), 3),
                        round(getattr(slot, "y", 0.0), 3),
                    ] if slot is not None else None,
                    "operation_point": [
                        round(getattr(operation_point, "x", 0.0), 3),
                        round(getattr(operation_point, "y", 0.0), 3),
                    ] if operation_point is not None else None,
                    "distance_slot_to_operation": round(slot.distance(operation_point), 3) if slot is not None and operation_point is not None else None,
                    "step": getattr(getattr(getattr(agent, "task", None), "port", None).__class__, "simulator_step", None) if getattr(getattr(agent, "task", None), "port", None) is not None else None,
                },
            )
        # #endregion
        hold_position = agent.position.copy() if hasattr(agent.position, "copy") else agent.position
        queue_slots = list(getattr(getattr(agent.task.port, "queue", None), "slots", []) or [])
        for slot_index, slot_point in enumerate(queue_slots):
            if slot_point.distance(slot) > 1e-3:
                continue
            if slot_index + 1 < len(queue_slots):
                hold_position = queue_slots[slot_index + 1]
            break
        if agent.task.destination_location != hold_position:
            agent.goal_changed = True
            agent.task.destination_location = hold_position
        agent.destination_location = hold_position
        return
    if (
        slot is not None
        and hasattr(agent.task.port, "is_at_operation_point")
        and hasattr(agent, "snap_to_position")
        and agent.position is not None
    ):
        is_operation_slot = agent.task.port.is_at_operation_point(slot, tolerance=1e-3)
        slot_tolerance = max(
            0.12,
            float(getattr(agent.task.port, "operation_entry_tolerance", 0.2) or 0.2),
        )
        if not is_operation_slot and agent.position.distance(slot) <= slot_tolerance:
            agent.snap_to_position(slot)
            agent.destination_location = slot.copy() if hasattr(slot, "copy") else slot
            if getattr(agent, "task", None) is not None:
                agent.task.destination_location = agent.destination_location
    if slot != agent.task.destination_location:
        agent.goal_changed = True
        agent.task.destination_location = slot

""" ================= Agent-side functions =================================="""
def go_for_next_pose(agent, server=None):
    if len(agent.sequence_of_poses) > 0:
        pass
    else:
        # agent.state = AgentState.HALT
        agent.stop()
def start_loading(agent, server=None):
    operation_point = getattr(agent.task.port, "operation_zone", None)
    current_slot = agent.task.port.get_slot(agent)
    if operation_point is not None and hasattr(agent, "snap_to_position"):
        agent.snap_to_position(operation_point)
        agent.destination_location = operation_point.copy() if hasattr(operation_point, "copy") else operation_point
        if getattr(agent, "task", None) is not None:
            agent.task.destination_location = agent.destination_location
    debug_event = getattr(agent, "_debug_v4_event", None)
    if callable(debug_event):
        debug_event(
            "F",
            "agent_state_machine.py:start_loading",
            "[DEBUG] agent entered loading state",
            {
                "agent_id": getattr(agent, "id", None),
                "position": [
                    round(getattr(agent.position, "x", 0.0), 3),
                    round(getattr(agent.position, "y", 0.0), 3),
                ] if getattr(agent, "position", None) is not None else None,
                "operation_point": [
                    round(getattr(operation_point, "x", 0.0), 3),
                    round(getattr(operation_point, "y", 0.0), 3),
                ] if operation_point is not None else None,
                "slot": [
                    round(getattr(current_slot, "x", 0.0), 3),
                    round(getattr(current_slot, "y", 0.0), 3),
                ] if current_slot is not None else None,
                "distance_to_slot": round(agent.position.distance(current_slot), 3)
                if getattr(agent, "position", None) is not None and current_slot is not None else None,
                "distance_to_operation_point": round(agent.position.distance(operation_point), 3)
                if getattr(agent, "position", None) is not None and operation_point is not None else None,
                "task": getattr(getattr(agent, "task", None), "type", None).name if getattr(getattr(agent, "task", None), "type", None) is not None else None,
                "sim_position": [
                    round(getattr(getattr(getattr(agent, "perception_module", None), "simulated_agent", None), "position", [0.0, 0.0])[0], 3),
                    round(getattr(getattr(getattr(agent, "perception_module", None), "simulated_agent", None), "position", [0.0, 0.0])[1], 3),
                ] if getattr(getattr(agent, "perception_module", None), "simulated_agent", None) is not None else None,
            },
        )
    agent.state = AgentState.LOADING
    server.update_data(agent)

def stay_in_current_state(agent, server=None):
    return
