"""
用户自定义的全局规划器版本集中在这个目录下。

本项目为了方便实验对比和会议展示，将每一轮主路径规划改进都拆成单独脚本：
- layered_a_star_collision_aware_planner.py  -> v1
- layered_a_star_reservation_aware_planner.py -> v2
- layered_a_star_queue_aware_planner.py      -> v3
- layered_a_star_baseline_traffic_aware_planner.py -> baseline 独立分支

这些模块会在命令行解析阶段被导入，再通过 simulator / interaction_handler
接到实际运行流程中。这样做的好处是：
1. 每轮改进对应一个独立文件，方便展示“这一轮具体改了什么”；
2. 不会污染 baseline；
3. 便于在实验脚本里直接切换不同版本。
"""

# 显式导入各轮改进版本，方便命令行识别和代码展示。
import global_planners.user.layered_a_star_collision_aware_planner
import global_planners.user.layered_a_star_reservation_aware_planner
import global_planners.user.layered_a_star_queue_aware_planner
import global_planners.user.layered_a_star_baseline_traffic_aware_planner
