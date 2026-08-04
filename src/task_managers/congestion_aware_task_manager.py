"""
@brief : Task manager that spreads agents across loading ports using queue and congestion signals
"""
from task_managers.naive_task_manager import NaiveTaskManager


class CongestionAwareTaskManager(NaiveTaskManager):
    """Prefer loading ports with lower short-term congestion and fewer incoming agents."""

    def choose_loading_port(self, agent, agents_state, longest_squared_distance):
        def evaluate_port(loading_port):
            queue_agents = 0
            queue_capacity = 1
            if getattr(loading_port, "queue", None) is not None:
                queue_agents = loading_port.queue.num_agents()
                queue_capacity = max(1, len(getattr(loading_port.queue, "slots", [])))

            incoming_agents = 0
            committed_agents = 0
            for state_info in list(agents_state.values()):
                task = state_info.get("task")
                task_port = getattr(task, "port", None)
                if task_port != loading_port:
                    continue
                incoming_agents += 1
                agent_state = getattr(state_info.get("state"), "name", None)
                if agent_state in {"PREQUEUE", "QUEUING", "LOADING"}:
                    committed_agents += 1

            # supplement 分支的核心改动在这里：
            # 选 loading port 时不只看距离，还同时看
            # 1. 当前 queue 是否已经拥挤；
            # 2. 有没有很多机器人已经在往这个口走；
            # 3. 这些机器人是否已经进入排队 / 装货状态。
            queue_ratio = float(queue_agents) / queue_capacity
            distance_ratio = 0.0
            if longest_squared_distance > 0:
                distance_ratio = loading_port.entry_point.squared_distance(agent.position) / float(longest_squared_distance)

            backlog_signal = min(1.0, float(len(getattr(loading_port, "items", []))) / 20.0)
            available_slots = queue_capacity - queue_agents

            return (
                2.2 * available_slots
                + 0.3 * backlog_signal
                - 2.6 * queue_ratio
                - 1.25 * incoming_agents
                - 1.75 * committed_agents
                - 0.55 * distance_ratio
            )

        return max(self.loading_ports, key=evaluate_port)
