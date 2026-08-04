from math import pi, sqrt

from geometry import Point, edge_edge_shortest_square_distance
from global_planners.user.layered_a_star_reservation_aware_planner import LayeredAStarReservationAware
from representation.float_to_grid import agent_to_gridmap
from task_managers.task_manager import TaskType


class LayeredAStarPortPriorityAware(LayeredAStarReservationAware):
    """Reservation-aware planner with port exit-before-entry priority."""

    SENSOR_RANGE = 7.0
    SENSOR_ANGLE = pi
    WINDOW_DISTANCE = 7.5
    PRIORITY_NODE_INFLATION = 10.0
    PRIORITY_EDGE_INFLATION = 8.0
    PORT_CORRIDOR_INFLATION = 12.0
    YIELD_DISTANCE = 1.7
    YIELD_SEGMENT_DISTANCE = 1.1
    STOPPING_BUFFER_RADIUS = 1.35
    STOPPING_NODE_INFLATION = 48.0
    STOPPING_EDGE_INFLATION = 36.0
    EXIT_ZONE_RADIUS = 3.0
    ENTER_ZONE_RADIUS = 3.0
    LOOKAHEAD_POINTS = 4
    QUEUE_SLOT_EPSILON = 1e-3

    def get_dynamic_layer(self, gridmap, sensor_observation, inflation=None):
        # priority 分支是在 v3 的 reservation-aware 之上，再叠一层
        # “港口进出优先级”代价约束：
        # 1. 先保留 v3 已有的 reservation window；
        # 2. 再识别感知范围内谁的通行等级比自己高；
        # 3. 把高优先级 agent 的未来节点和边进一步抬价，
        #    让低优先级 agent 在全局规划阶段就主动绕开。
        dynamic_layer = super(LayeredAStarPortPriorityAware, self).get_dynamic_layer(
            gridmap, sensor_observation, inflation
        )

        own_priority = self._priority_tuple(self.agent)
        own_target_port = self._target_port(self.agent)
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            observed_priority = self._priority_tuple(observed_state)
            if self._observed_state_name(observed_state) == "STOPPING":
                if not self._stopping_should_not_block_self(observed_state):
                    self._apply_stopping_barrier(dynamic_layer, gridmap, observed_state)
            if observed_priority <= own_priority:
                continue

            # 当别人优先级更高时，我们不是简单地“看到就停”，
            # 而是把对方未来一小段路径做成更重的软障碍。
            # 这样低优先级 agent 会优先考虑改路，而不是所有情况都退化成双停。
            reservation_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
            if not reservation_nodes:
                reservation_nodes = [agent_to_gridmap(observed_state.position, gridmap)]
            self._apply_priority_window(dynamic_layer, gridmap, reservation_nodes)

            source_port = self._source_port_for_current_task(observed_state)
            if own_target_port is not None and source_port == own_target_port:
                # 这是 priority 相比 v3 最“港口特化”的一层：
                # 如果对方正从我要进入的这个 port 离开，就把整个 port corridor
                # 都临时抬价，含义是“先让出港，再让入港靠近”。
                self._apply_port_corridor_priority(dynamic_layer, gridmap, source_port)

        return dynamic_layer

    def observe_path(self, gridmap, current_position, sensor_observation, sequence_of_poses, threshold=1):
        # 这里先复用 v3 的一般性冲突判断；
        # 如果 v3 已经能判断出 reservation / queue / stopping 冲突，
        # priority 不重复造一套，只补“高优先级 agent 即将占用我前方路径”的检测。
        if super(LayeredAStarPortPriorityAware, self).observe_path(
            gridmap, current_position, sensor_observation, sequence_of_poses, threshold
        ):
            return True

        own_priority = self._priority_tuple(self.agent)
        own_nodes = [agent_to_gridmap(current_position, gridmap)]
        for pose in list(sequence_of_poses)[: self.LOOKAHEAD_POINTS]:
            own_nodes.append(agent_to_gridmap(pose, gridmap))

        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._observed_state_name(observed_state) == "STOPPING":
                if (not self._stopping_should_not_block_self(observed_state)) and \
                    self.path_conflicts_with_stopping_agents(current_position, sequence_of_poses, sensor_observation):
                    return True
            if self._priority_tuple(observed_state) <= own_priority:
                continue

            reservation_nodes = self._collect_reservation_nodes(observed_agent, gridmap)
            if not reservation_nodes:
                reservation_nodes = [agent_to_gridmap(observed_state.position, gridmap)]

            # 这里只看双方最近几步节点是否已经重叠，
            # 含义是：我的当前规划马上会闯入更高优先级 agent 的时间窗，
            # 那就应该提前触发 replan，而不是等到贴脸再处理。
            if set(own_nodes[:3]).intersection(reservation_nodes[:3]):
                return True

        return False

    def compute_avoidance_response(self, position, sensor_observation, sequence_of_poses):
        # 这一层负责近场兜底：
        # 如果全局层还没来得及把双方完全分开，但低优先级 agent 已经逼近
        # 高优先级 agent，就在执行前再做一次“未来几步是否会撞”的判断，
        # 命中后立刻要求 replan，必要时交给上层 fallback stop。
        own_priority = self._priority_tuple(self.agent)
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            observed_state_name = self._observed_state_name(observed_state)
            if observed_state_name == "STOPPING":
                if self._stopping_should_not_block_self(observed_state):
                    continue
                if self._future_path_hits_stopping_agent(own_future, observed_state.position):
                    return {
                        "replan": True,
                        "reason": "stopping_agent",
                        "blocker_id": getattr(observed_state, "id", None),
                        "fallback_command": (0.0, 0.0),
                    }
                continue

            if self._priority_tuple(observed_state) <= own_priority:
                continue
            if position.distance(observed_state.position) > self.YIELD_DISTANCE:
                continue

            # 这里不再只看“当前距离近不近”，而是比较双方未来短时轨迹。
            # 这样可以把“出港车和进港车即将对冲”这种冲突提前识别出来。
            observed_future = self._future_points(
                observed_state.position,
                getattr(observed_state, "sequence_of_poses", []),
                getattr(observed_state, "linear_velocity", None) or getattr(observed_agent, "linearVelocity", None),
            )
            if self._future_paths_conflict(own_future, observed_future):
                return {
                    "replan": True,
                    "fallback_command": (0.0, 0.0),
                    "reason": "yield_to_higher_priority",
                    "blocker_id": getattr(observed_state, "id", None),
                }
        return None

    def path_conflicts_with_stopping_agents(self, position, sequence_of_poses, sensor_observation):
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if self._observed_state_name(observed_state) != "STOPPING":
                continue
            if self._stopping_should_not_block_self(observed_state):
                continue
            if self._future_path_hits_stopping_agent(own_future, observed_state.position):
                return True
        return False

    def path_conflicts_with_blocker(self, position, sequence_of_poses, sensor_observation, blocker_id):
        if blocker_id is None:
            return False
        own_future = self._future_points(position, sequence_of_poses, self.agent.linear_velocity)
        for observed_agent in sensor_observation.other_agents_state_in_range_of(self.SENSOR_RANGE, self.SENSOR_ANGLE):
            observed_state = getattr(observed_agent, "userData", observed_agent)
            if getattr(observed_state, "id", None) != blocker_id:
                continue
            if self._observed_state_name(observed_state) == "STOPPING":
                return self._future_path_hits_stopping_agent(own_future, observed_state.position)
            observed_future = self._future_points(
                observed_state.position,
                getattr(observed_state, "sequence_of_poses", []),
                getattr(observed_state, "linear_velocity", None) or getattr(observed_agent, "linearVelocity", None),
            )
            return self._future_paths_conflict(own_future, observed_future)
        return False

    def _apply_priority_window(self, dynamic_layer, gridmap, reservation_nodes):
        # priority window 的作用，是把“高优先级 agent 接下来几步可能经过的格子”
        # 逐步衰减地抬价。越靠前的节点越重要，越远的节点影响越弱，
        # 避免全图都被硬挡住。
        for step_idx, node in enumerate(reservation_nodes[: self.RESERVATION_WAYPOINTS + 1]):
            decay = self.RESERVATION_DECAY ** step_idx
            self._reserve_node_entries(
                dynamic_layer,
                gridmap,
                node,
                self.PRIORITY_NODE_INFLATION * decay,
            )
            if step_idx > 0:
                previous = reservation_nodes[step_idx - 1]
                self._reserve_transition(
                    dynamic_layer,
                    previous,
                    node,
                    self.PRIORITY_EDGE_INFLATION * decay,
                )

    def _apply_port_corridor_priority(self, dynamic_layer, gridmap, port):
        # 这里把 port operation zone + queue slots 当成一个 corridor。
        # 一旦判定“对方正在从这个 port 离开，而我正准备进入它”，
        # 就把这条 corridor 整体抬价，含义是低优先级一方先不要顶进去。
        corridor_nodes = self._queue_nodes_for_port(port, gridmap)
        if not corridor_nodes:
            return
        for step_idx, node in enumerate(corridor_nodes):
            decay = max(0.5, 1.0 - 0.1 * step_idx)
            self._reserve_node_entries(
                dynamic_layer,
                gridmap,
                node,
                self.PORT_CORRIDOR_INFLATION * decay,
            )

    def _queue_nodes_for_port(self, port, gridmap):
        queue = getattr(port, "queue", None)
        if queue is None:
            return []
        nodes = []
        operation_zone = getattr(port, "operation_zone", None)
        if operation_zone is not None:
            nodes.append(agent_to_gridmap(operation_zone, gridmap))
        for slot in getattr(queue, "slots", []):
            nodes.append(agent_to_gridmap(slot, gridmap))
        deduped = []
        for node in nodes:
            if not deduped or deduped[-1] != node:
                deduped.append(node)
        return deduped

    def _apply_stopping_barrier(self, dynamic_layer, gridmap, observed_state):
        origin = agent_to_gridmap(observed_state.position, gridmap)
        self._inflate_transition_ring(dynamic_layer, gridmap, origin, self.STOPPING_NODE_INFLATION)
        for first in gridmap.neighbors(origin):
            self._inflate_transition_ring(dynamic_layer, gridmap, first, self.STOPPING_EDGE_INFLATION)
            for second in gridmap.neighbors(first):
                if second == origin:
                    continue
                self._inflate_transition_ring(dynamic_layer, gridmap, second, self.STOPPING_EDGE_INFLATION * 0.45)

    def _priority_tuple(self, agent_state):
        # 排序规则分三层：
        # 1. 先比宏观通行等级（出港 > loading > queueing ...）；
        # 2. queueing 内部再比谁更靠近队首；
        # 3. 最后用 id 做稳定 tie-break，避免每帧随机换边。
        return (
            self._priority_score(agent_state),
            self._queue_progress_score(agent_state),
            -getattr(agent_state, "id", 0),
        )

    def _priority_score(self, agent_state):
        # 这是 priority 分支最核心的“语义优先级”定义：
        # - 正在出港的 CRUISE 最高，因为它代表“作业完成，先让它离开前区”；
        # - LOADING / QUEUING 次之，表示已经占据或即将占据该 port 流程；
        # - 普通巡航最低，进入目标 port 的巡航还要再低一些，表示应让路。
        state_name = self._observed_state_name(agent_state)
        if self._is_exiting_port(agent_state):
            return 8
        if state_name == "STOPPING":
            return 1
        if state_name == "LOADING":
            return 6
        if state_name == "QUEUING":
            return 5
        if state_name == "PREQUEUE":
            return 4
        if state_name == "CRUISE" and self._is_entering_target_port(agent_state):
            return 2
        if state_name == "CRUISE":
            return 3
        return 1

    def _queue_progress_score(self, agent_state):
        # 同样都是 QUEUING 时，越接近 operation zone，分数越高。
        # 这保证了队首不会被队尾“反超”，也符合排队系统的 FIFO 直觉。
        state_name = self._base_state_name(agent_state)
        if state_name != "QUEUING":
            return 0
        port = self._target_port(agent_state)
        if port is None:
            return 0
        queue = getattr(port, "queue", None)
        if queue is None:
            return 0
        try:
            slot = port.get_slot(agent_state)
        except Exception:
            return 0
        for index, slot_point in enumerate(getattr(queue, "slots", [])):
            if slot_point.distance(slot) <= self.QUEUE_SLOT_EPSILON:
                return len(queue.slots) - index
        return 0

    def _is_entering_target_port(self, agent_state):
        # 判断“正在进港”的标准，不是只看 task type，
        # 还要看当前位置是否已经靠近目标 port 的控制区/入口。
        # 这样可以把远处普通巡航和真正要进前区的 agent 区分开。
        if self._base_state_name(agent_state) not in {"CRUISE", "PREQUEUE"}:
            return False
        target_port = self._target_port(agent_state)
        position = getattr(agent_state, "position", None)
        if target_port is None or position is None:
            return False
        try:
            return target_port.in_control_range(position) or target_port.entry_point.distance(position) <= self.ENTER_ZONE_RADIUS
        except Exception:
            return False

    def _is_exiting_port(self, agent_state):
        # 判断“正在出港”的标准是：
        # 当前处于 CRUISE，但它离当前任务来源 port 的作业区仍很近。
        # 这意味着它刚完成 port 内动作，正从前区退出，应获得最高优先级。
        state_name = self._base_state_name(agent_state)
        if state_name != "CRUISE":
            return False
        source_port = self._source_port_for_current_task(agent_state)
        position = getattr(agent_state, "position", None)
        if source_port is None or position is None:
            return False
        anchor = getattr(source_port, "operation_zone", None) or getattr(source_port, "location", None)
        return anchor is not None and anchor.distance(position) <= self.EXIT_ZONE_RADIUS

    def _target_port(self, agent_state):
        return getattr(getattr(agent_state, "task", None), "port", None)

    def _source_port_for_current_task(self, agent_state):
        task_type = getattr(getattr(agent_state, "task", None), "type", None)
        if task_type == TaskType.GO_TO_UNLOADING_PORT:
            return self._nearest_port_by_type(agent_state, "loading")
        if task_type == TaskType.GO_TO_LOADING_PORT:
            return self._nearest_port_by_type(agent_state, "unloading")
        return None

    def _nearest_port_by_type(self, agent_state, port_type):
        position = getattr(agent_state, "position", None)
        if position is None:
            return None
        candidate_ports = [
            port for port in self._all_ports()
            if getattr(port, "port_type", None) == port_type
        ]
        if not candidate_ports:
            return None
        nearest = min(candidate_ports, key=lambda port: position.distance(getattr(port, "operation_zone", None) or port.location))
        anchor = getattr(nearest, "operation_zone", None) or nearest.location
        if anchor.distance(position) <= self.EXIT_ZONE_RADIUS:
            return nearest
        return None

    def _all_ports(self):
        server = getattr(self.agent, "server", None)
        if server is None:
            return []
        return list(getattr(server, "loading_ports", [])) + list(getattr(server, "unloading_ports", []))

    def _future_points(self, position, sequence_of_poses, velocity):
        # priority 近场让行判断依赖一小段未来轨迹。
        # 如果 planner 已给出 sequence_of_poses，就直接用；
        # 如果暂时拿不到，就按当前速度方向补一个短预测，
        # 避免“因为没有离散路径就完全不会判断让行”。
        points = [Point(position.x, position.y)]
        for pose in list(sequence_of_poses)[: self.LOOKAHEAD_POINTS]:
            points.append(Point(pose.x, pose.y))

        if len(points) > 1:
            return points

        vx = float(getattr(velocity, "x", 0.0) if velocity is not None else 0.0)
        vy = float(getattr(velocity, "y", 0.0) if velocity is not None else 0.0)
        if velocity is not None and not hasattr(velocity, "x"):
            try:
                vx = float(velocity[0])
                vy = float(velocity[1])
            except Exception:
                vx, vy = 0.0, 0.0
        speed = sqrt(vx * vx + vy * vy)
        if speed < 1e-3:
            return points

        step_distance = max(0.8, min(1.4, speed))
        dx = vx / speed
        dy = vy / speed
        for step_idx in range(1, self.LOOKAHEAD_POINTS + 1):
            points.append(
                Point(
                    position.x + dx * step_distance * step_idx,
                    position.y + dy * step_distance * step_idx,
                )
            )
        return points

    def _future_paths_conflict(self, own_future, observed_future):
        # 同时检查“线段是否擦撞”和“近几步节点是否贴近”：
        # 前者处理对向穿越，后者处理低速贴靠或几乎停住的近接触。
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            for other_index in range(len(observed_future) - 1):
                other_start = observed_future[other_index]
                other_end = observed_future[other_index + 1]
                square_distance, _ = edge_edge_shortest_square_distance(
                    own_start, own_end, other_start, other_end
                )
                if square_distance <= self.YIELD_SEGMENT_DISTANCE ** 2:
                    return True

        for own_point in own_future[:3]:
            for other_point in observed_future[:3]:
                if own_point.distance(other_point) <= self.YIELD_DISTANCE:
                    return True
        return False

    def _future_path_hits_stopping_agent(self, own_future, stopping_position):
        # STOPPING agent 在前区里等价于一个短时静态障碍。
        # 这里用“未来路径打到停止缓冲圈”来判断是否必须让开。
        stop_point = Point(stopping_position.x, stopping_position.y)
        for own_index in range(len(own_future) - 1):
            own_start = own_future[own_index]
            own_end = own_future[own_index + 1]
            square_distance, _ = edge_edge_shortest_square_distance(
                own_start, own_end, stop_point, stop_point
            )
            if square_distance <= self.STOPPING_BUFFER_RADIUS ** 2:
                return True
        for own_point in own_future[:3]:
            if own_point.distance(stop_point) <= self.STOPPING_BUFFER_RADIUS:
                return True
        return False

    def _stopping_should_not_block_self(self, observed_state):
        # 两种情况不应把 STOPPING agent 当成我的硬障碍：
        # 1. 它本来就是为了给我让路而停下；
        # 2. 我自己正在执行出港优先，不应被前区让行车反过来挡住。
        if getattr(observed_state, "stopping_for_agent_id", None) == getattr(self.agent, "id", None):
            return True
        if self._is_exiting_port(self.agent):
            return True
        return False

    def _observed_state_name(self, agent_state):
        if getattr(agent_state, "stopping_active", False):
            return "STOPPING"
        return getattr(getattr(agent_state, "state", None), "name", None)

    def _base_state_name(self, agent_state):
        if getattr(agent_state, "stopping_active", False):
            return getattr(getattr(agent_state, "stopping_base_state", None), "name", None)
        return getattr(getattr(agent_state, "state", None), "name", None)
