"""
@Copyright Dorabot Inc.
@date : 2018-10
@author: {xiaoyu.ge, chen.chong2}@dorabot.com
@brief : local planner based on virtual forces
"""
from geometry import Vector, compute_direction, Point
from .local_planner import LocalPlanner
from math import sqrt
class VirtualForcePlanner(LocalPlanner):
    LOOKAHEAD_STEPS = 2
    FINAL_GOAL_RADIUS = 1.2
    FINAL_STOP_RADIUS = 0.2
    SLOWDOWN_RADIUS = 1.0
    DOCKING_RADIUS = 1.0
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
    def compute_plan(self, position, velocity, gridmap, sensor_observation, global_planner_path):
        local_path = []
        start_pose = position
        
        goal_pose, final_goal = self._select_goal_pose(position, global_planner_path)

        final_distance = position.distance(final_goal)
        if final_distance <= self.FINAL_STOP_RADIUS:
            return (0.0, 0.0)

        if self._should_enter_docking_mode(final_distance):
            return self._dock_to_goal(position, final_goal, final_distance)

        direction = compute_direction(start_pose, goal_pose)

        result_vel = Vector(velocity[0], velocity[1])
        neighbor_count = 0
        for agent in sensor_observation.other_agents_state_in_range_of(5):
            vel_A_B_dir = compute_direction(position, agent.position).normalize()
            # TODO Gary: @Gary do not modify agent status in hidden functions
            self.agent.potential_collision = True
            force_B_A = self.__combined_force(position, agent.position)
            result_vel = result_vel + force_B_A
            neighbor_count += 1
        target_port = getattr(getattr(self.agent, "task", None), "port", None)
        port_count = 0
        for body in sensor_observation.ports_in_range_of(4):
            self.agent.potential_collision = True
            obstacle = body.userData
            if obstacle == target_port:
                # 目标 port 不应被当成需要持续排斥的障碍，
                # 否则 agent 会在 entry/queue/operation zone 前被自己要去的口顶住。
                continue
            pB = Point(obstacle.location.x + obstacle.dimension[0] * 0.5, obstacle.location.y + obstacle.dimension[1] * 0.5)
            force_B_A = self.__combined_force(position, pB)
            result_vel = result_vel + force_B_A
            port_count += 1
            
        force_goal = self.__goal_attraction(position, goal_pose)
        result_vel = result_vel + force_goal
        ratio = sqrt(result_vel.x**2 + result_vel.y **2)
        if ratio < 1e-9:
            return (0.0, 0.0)

        commanded_speed = self.agent.cruise_speed
        if final_distance < self.SLOWDOWN_RADIUS:
            commanded_speed = min(commanded_speed, max(0.0, final_distance * 2.0))
        result_vel = result_vel.scale(commanded_speed/ratio)
        return result_vel.to_tuple()

    def _should_enter_docking_mode(self, final_distance):
        state_name = getattr(getattr(self.agent, "state", None), "name", None)
        if state_name == "QUEUING":
            return True
        if state_name not in {"PREQUEUE", "LOADING"}:
            return False
        return final_distance <= self.DOCKING_RADIUS

    def _dock_to_goal(self, position, final_goal, final_distance):
        direction = compute_direction(position, final_goal)
        ratio = sqrt(direction.x ** 2 + direction.y ** 2)
        if ratio < 1e-9:
            return (0.0, 0.0)
        commanded_speed = min(self.agent.cruise_speed, max(0.0, final_distance * 2.0))
        return direction.scale(commanded_speed / ratio).to_tuple()

    def _select_goal_pose(self, position, global_planner_path):
        if len(global_planner_path) == 0:
            return position, position

        final_goal = global_planner_path[-1]
        if position.distance(final_goal) <= self.FINAL_GOAL_RADIUS:
            return final_goal, final_goal

        nearest_index = min(
            range(len(global_planner_path)),
            key=lambda idx: global_planner_path[idx].distance(position),
        )
        goal_index = min(len(global_planner_path) - 1, nearest_index + self.LOOKAHEAD_STEPS)
        return global_planner_path[goal_index], final_goal

    def __repel_force(self, pos_a, pos_b):
        dist = pos_a.distance(pos_b)
        vec = compute_direction(pos_a, pos_b).normalize()
        force= -20/(dist**4)
        return vec.scale(force)
    def __combined_force(self, pos_a, pos_b):
        return self.__repel_force(pos_a, pos_b)
    def __goal_attraction(self, pos_current, loc_destination):
        dist = pos_current.distance(loc_destination)
        vec = compute_direction(pos_current, loc_destination).normalize()
        return vec.scale(100/(dist+0.01))
