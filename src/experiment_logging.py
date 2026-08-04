import csv
import json
import os
import urllib.request
import time
from collections import defaultdict

DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\v4-port-collision.env"
DEBUG_FALLBACK_URL = "http://127.0.0.1:7778/event"
DEBUG_SESSION_ID = "v4-port-collision"
BASELINE_COLLISION_DEBUG_ENV_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\baseline-traffic-collision.env"
BASELINE_COLLISION_DEBUG_FALLBACK_URL = "http://127.0.0.1:7777/event"
BASELINE_COLLISION_DEBUG_SESSION_ID = "baseline-traffic-collision"
BASELINE_COLLISION_DEBUG_LOG_PATH = r"c:\Users\INT\Desktop\Summer IP\dorabot_minions-master\.dbg\trae-debug-log-baseline-traffic-collision.ndjson"
from datetime import datetime
from math import sqrt
from uuid import uuid4


class RunMetricsRecorder(object):
    SNAPSHOT_INTERVAL_SECONDS = 10.0
    COLLISION_SAMPLE_LIMIT = 24

    def __init__(self, simulator, cmd_args, config_data, enabled):
        self.simulator = simulator
        self.cmd_args = cmd_args
        self.config_data = config_data
        self.enabled = enabled
        self.run_id = "{}_{}".format(
            datetime.utcnow().strftime("%Y%m%dT%H%M%SZ"),
            uuid4().hex[:8],
        )
        self.run_started_at = time.time()
        self.last_positions = {}
        self.state_step_counts = defaultdict(int)
        self.contact_frame_counts = defaultdict(int)
        self.collision_event_counts = defaultdict(int)
        self.active_collision_pairs = set()
        self.collision_samples = []
        self.max_queue_lengths = {}
        self.timeline = []
        self.next_snapshot_time = self.SNAPSHOT_INTERVAL_SECONDS
        self.results_dir = self._resolve_results_dir()
        if self.enabled:
            os.makedirs(self.results_dir, exist_ok=True)

    def _resolve_results_dir(self):
        if getattr(self.cmd_args, "record_dir", None):
            return os.path.abspath(self.cmd_args.record_dir)
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        return os.path.join(repo_root, "experiments", "results")

    def update(self):
        if not self.enabled:
            return
        self._update_distance_metrics()
        self._update_state_metrics()
        self._update_contact_metrics()
        self._update_queue_metrics()
        self._maybe_capture_timeline_snapshot()

    def _update_distance_metrics(self):
        for agent in self.simulator.agents:
            position = (agent.position.x, agent.position.y)
            previous_position = self.last_positions.get(agent.id)
            if previous_position is not None:
                dx = position[0] - previous_position[0]
                dy = position[1] - previous_position[1]
                agent.distance_travelled += sqrt(dx * dx + dy * dy)
            self.last_positions[agent.id] = position

    def _update_state_metrics(self):
        for agent in self.simulator.agents:
            if getattr(agent, "state", None) is not None:
                self.state_step_counts[agent.state.name] += 1

    def _update_contact_metrics(self):
        contact_types = defaultdict(int)
        active_pairs_this_step = set()
        for contact in self.simulator.world.contacts:
            if not contact.touching:
                continue
            if contact.fixtureA.sensor or contact.fixtureB.sensor:
                continue
            body_a = contact.fixtureA.body
            body_b = contact.fixtureB.body
            type_a = getattr(contact.fixtureA.body.userData, "type", "unknown")
            type_b = getattr(contact.fixtureB.body.userData, "type", "unknown")
            pair = tuple(sorted((type_a, type_b)))
            key = "{}__{}".format(pair[0], pair[1])
            contact_types[key] += 1
            pair_identity = (key, tuple(sorted((id(body_a), id(body_b)))))
            active_pairs_this_step.add(pair_identity)
            if pair_identity not in self.active_collision_pairs:
                self.collision_event_counts[key] += 1
                if len(self.collision_samples) < self.COLLISION_SAMPLE_LIMIT:
                    self.collision_samples.append(
                        self._build_collision_sample(key, body_a, body_b)
                    )
        for key, count in list(contact_types.items()):
            self.contact_frame_counts[key] += count
        self.active_collision_pairs = active_pairs_this_step

    def _build_collision_sample(self, key, body_a, body_b):
        entity_a = getattr(body_a, "userData", None)
        entity_b = getattr(body_b, "userData", None)
        planner_a = getattr(getattr(entity_a, "global_planner", None), "__class__", type("", (), {})).__name__
        planner_b = getattr(getattr(entity_b, "global_planner", None), "__class__", type("", (), {})).__name__
        collision_data = {
            "sim_time_seconds": round(self.simulator.get_simulator_time(), 3),
            "entity_a": {
                "id": getattr(entity_a, "id", None),
                "state": getattr(getattr(entity_a, "state", None), "name", None),
                "task": getattr(getattr(getattr(entity_a, "task", None), "type", None), "name", None),
                "planner": planner_a,
                "stopping": bool(getattr(entity_a, "stopping_active", False)),
                "position": [round(getattr(getattr(entity_a, 'position', None), 'x', 0.0), 2), round(getattr(getattr(entity_a, 'position', None), 'y', 0.0), 2)] if getattr(entity_a, 'position', None) is not None else None,
            },
            "entity_b": {
                "id": getattr(entity_b, "id", None),
                "state": getattr(getattr(entity_b, "state", None), "name", None),
                "task": getattr(getattr(getattr(entity_b, "task", None), "type", None), "name", None),
                "planner": planner_b,
                "stopping": bool(getattr(entity_b, "stopping_active", False)),
                "position": [round(getattr(getattr(entity_b, 'position', None), 'x', 0.0), 2), round(getattr(getattr(entity_b, 'position', None), 'y', 0.0), 2)] if getattr(entity_b, 'position', None) is not None else None,
            },
        }
        if key == "agent__agent" and ("LayeredAStarQueueAware" in {planner_a, planner_b}):
            # #region debug-point C:collision-sample
            try:
                _u,_s=DEBUG_FALLBACK_URL,DEBUG_SESSION_ID
                with open(DEBUG_ENV_PATH,"r",encoding="utf-8") as _f:
                    _c=_f.read()
                for _l in _c.splitlines():
                    if _l.startswith("DEBUG_SERVER_URL="): _u=_l.split("=",1)[1]
                    elif _l.startswith("DEBUG_SESSION_ID="): _s=_l.split("=",1)[1]
                urllib.request.urlopen(urllib.request.Request(_u,data=json.dumps({"sessionId":_s,"runId":"pre-fix","hypothesisId":"C","location":"experiment_logging.py:_build_collision_sample","msg":"[DEBUG] agent-agent collision sample captured","data":collision_data,"ts":0}).encode(),headers={"Content-Type":"application/json"}),timeout=0.2).read()
            except Exception:
                pass
            # #endregion
        if key == "agent__agent" and ("LayeredAStarBaselineTrafficAware" in {planner_a, planner_b}):
            # #region debug-point C:baseline-collision-sample
            try:
                with open(BASELINE_COLLISION_DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
                    debug_file.write(json.dumps({
                        "sessionId": BASELINE_COLLISION_DEBUG_SESSION_ID,
                        "runId": "pre-fix",
                        "hypothesisId": "C",
                        "location": "experiment_logging.py:_build_collision_sample",
                        "msg": "[DEBUG] baseline agent-agent collision sample captured",
                        "data": collision_data,
                        "ts": 0,
                    }, ensure_ascii=False) + "\n")
            except Exception:
                pass
            try:
                _u, _s = BASELINE_COLLISION_DEBUG_FALLBACK_URL, BASELINE_COLLISION_DEBUG_SESSION_ID
                with open(BASELINE_COLLISION_DEBUG_ENV_PATH, "r", encoding="utf-8") as _f:
                    _c = _f.read()
                for _l in _c.splitlines():
                    if _l.startswith("DEBUG_SERVER_URL="):
                        _u = _l.split("=", 1)[1]
                    elif _l.startswith("DEBUG_SESSION_ID="):
                        _s = _l.split("=", 1)[1]
                urllib.request.urlopen(urllib.request.Request(_u, data=json.dumps({
                    "sessionId": _s,
                    "runId": "pre-fix",
                    "hypothesisId": "C",
                    "location": "experiment_logging.py:_build_collision_sample",
                    "msg": "[DEBUG] baseline agent-agent collision sample captured",
                    "data": collision_data,
                    "ts": 0,
                }).encode(), headers={"Content-Type": "application/json"}), timeout=0.2).read()
            except Exception:
                pass
            # #endregion
        return {
            "sim_time_seconds": round(self.simulator.get_simulator_time(), 3),
            "contact_type": key,
            "entity_a": self._serialize_entity(entity_a),
            "entity_b": self._serialize_entity(entity_b),
        }

    def _serialize_entity(self, entity):
        if entity is None:
            return {}
        position = getattr(entity, "position", None)
        task = getattr(getattr(entity, "task", None), "type", None)
        state = getattr(getattr(entity, "state", None), "name", None)
        return {
            "id": getattr(entity, "id", None),
            "type": getattr(entity, "type", None),
            "state": state,
            "stopping": bool(getattr(entity, "stopping_active", False)),
            "task": getattr(task, "name", None),
            "position": [
                round(getattr(position, "x", 0.0), 2),
                round(getattr(position, "y", 0.0), 2),
            ] if position is not None else None,
        }

    def _update_queue_metrics(self):
        all_ports = {}
        all_ports.update(getattr(self.simulator.environment, "loading_ports", {}))
        all_ports.update(getattr(self.simulator.environment, "unloading_ports", {}))
        for port_id, port in list(all_ports.items()):
            queue_length = len(getattr(port.queue, "queue", []))
            self.max_queue_lengths[port_id] = max(self.max_queue_lengths.get(port_id, 0), queue_length)

    def _maybe_capture_timeline_snapshot(self):
        current_time = self.simulator.get_simulator_time()
        if current_time + 1e-9 < self.next_snapshot_time:
            return
        snapshot = {
            "sim_time_seconds": round(current_time, 3),
            "packages_delivered": self.simulator.task_count,
            "pph": self._safe_pph(current_time),
            "current_states": self._current_state_counts(),
        }
        self.timeline.append(snapshot)
        self.next_snapshot_time += self.SNAPSHOT_INTERVAL_SECONDS

    def _safe_pph(self, sim_time_seconds):
        if sim_time_seconds <= 0:
            return 0.0
        return float(self.simulator.task_count) / sim_time_seconds * 3600.0

    def _current_state_counts(self):
        counts = defaultdict(int)
        for agent in self.simulator.agents:
            if getattr(agent, "state", None) is not None:
                counts[agent.state.name] += 1
        return dict(counts)

    def finalize(self):
        if not self.enabled:
            return None
        sim_time_seconds = self.simulator.time
        total_distance = sum(agent.distance_travelled for agent in self.simulator.agents)
        max_distance = max([agent.distance_travelled for agent in self.simulator.agents] + [0.0])
        global_plan_calls = sum(agent.global_plan_calls for agent in self.simulator.agents)
        replan_events = sum(agent.replan_events for agent in self.simulator.agents)
        agent_count = max(len(self.simulator.agents), 1)
        total_state_steps = max(sum(self.state_step_counts.values()), 1)
        total_contact_frames = sum(self.contact_frame_counts.values())
        total_collision_events = sum(self.collision_event_counts.values())
        summary = {
            "run_id": self.run_id,
            "record_tag": getattr(self.cmd_args, "record_tag", None),
            "timestamp_utc": datetime.utcnow().isoformat() + "Z",
            "title": self.simulator.title,
            "headless": self.simulator.simulation_times != -1,
            "task_manager": getattr(self.simulator, "task_manager_name", ""),
            "global_planner": self.simulator.agent_global_planner,
            "local_planner": self.simulator.agent_local_planner,
            "simulation_minutes_target": self.simulator.simulation_times,
            "simulation_seconds": sim_time_seconds,
            "packages_delivered": self.simulator.task_count,
            "pph": self._safe_pph(sim_time_seconds),
            "real_runtime_seconds": round(time.time() - self.run_started_at, 3),
            "agent_count": len(self.simulator.agents),
            "loading_port_count": len(getattr(self.simulator.environment, "loading_ports", {})),
            "unloading_port_count": len(getattr(self.simulator.environment, "unloading_ports", {})),
            "map_width": self.config_data["environment"]["width_in_meters"],
            "map_height": self.config_data["environment"]["height_in_meters"],
            "resolution": self.config_data["environment"]["resolution"],
            "steps_per_second": self.config_data["simulator"]["steps_per_sec"],
            "total_distance_travelled": round(total_distance, 3),
            "avg_distance_travelled": round(total_distance / agent_count, 3),
            "max_distance_travelled": round(max_distance, 3),
            "global_plan_calls": global_plan_calls,
            "avg_global_plan_calls": round(float(global_plan_calls) / agent_count, 3),
            "replan_events": replan_events,
            "avg_replan_events": round(float(replan_events) / agent_count, 3),
            "total_contact_frames": total_contact_frames,
            "contact_frame_counts": dict(self.contact_frame_counts),
            "total_collision_events": total_collision_events,
            "collision_event_counts": dict(self.collision_event_counts),
            "agent_agent_collision_events": self.collision_event_counts.get("agent__agent", 0),
            "agent_wall_collision_events": self.collision_event_counts.get("agent__wall", 0),
            "collision_samples": self.collision_samples,
            "max_queue_lengths": self.max_queue_lengths,
            "state_step_counts": dict(self.state_step_counts),
            "state_time_ratios": {
                state: round(float(count) / total_state_steps, 4)
                for state, count in list(self.state_step_counts.items())
            },
            "timeline": self.timeline,
            "cmd_args": self._serialize_cmd_args(),
        }
        self._write_summary(summary)
        return summary

    def _serialize_cmd_args(self):
        return {
            key: value
            for key, value in vars(self.cmd_args).items()
            if key not in {"obstacle"}
        }

    def _write_summary(self, summary):
        tag = summary["record_tag"] or "run"
        json_filename = "{}.json".format(tag) if summary["record_tag"] else "{}_{}.json".format(tag, summary["run_id"])
        json_path = os.path.join(self.results_dir, json_filename)
        with open(json_path, "w") as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)

        jsonl_path = os.path.join(self.results_dir, "run_summaries.jsonl")
        summary_key = summary["record_tag"] or summary["run_id"]
        existing_jsonl = []
        if os.path.exists(jsonl_path):
            with open(jsonl_path, "r") as handle:
                for line in handle:
                    line = line.strip()
                    if not line:
                        continue
                    row = json.loads(line)
                    row_key = row.get("record_tag") or row.get("run_id")
                    if row_key != summary_key:
                        existing_jsonl.append(row)
        existing_jsonl.append(summary)
        with open(jsonl_path, "w") as handle:
            for row in existing_jsonl:
                handle.write(json.dumps(row, sort_keys=True) + "\n")

        csv_path = os.path.join(self.results_dir, "run_summaries.csv")
        row = {
            "run_id": summary["run_id"],
            "record_tag": summary["record_tag"],
            "timestamp_utc": summary["timestamp_utc"],
            "task_manager": summary["task_manager"],
            "global_planner": summary["global_planner"],
            "local_planner": summary["local_planner"],
            "packages_delivered": summary["packages_delivered"],
            "pph": summary["pph"],
            "simulation_seconds": summary["simulation_seconds"],
            "real_runtime_seconds": summary["real_runtime_seconds"],
            "agent_count": summary["agent_count"],
            "loading_port_count": summary["loading_port_count"],
            "unloading_port_count": summary["unloading_port_count"],
            "map_width": summary["map_width"],
            "map_height": summary["map_height"],
            "resolution": summary["resolution"],
            "steps_per_second": summary["steps_per_second"],
            "total_distance_travelled": summary["total_distance_travelled"],
            "avg_distance_travelled": summary["avg_distance_travelled"],
            "max_distance_travelled": summary["max_distance_travelled"],
            "global_plan_calls": summary["global_plan_calls"],
            "replan_events": summary["replan_events"],
            "total_contact_frames": summary["total_contact_frames"],
            "total_collision_events": summary["total_collision_events"],
            "agent_agent_collision_events": summary["agent_agent_collision_events"],
            "agent_wall_collision_events": summary["agent_wall_collision_events"],
            "state_time_ratios": json.dumps(summary["state_time_ratios"], sort_keys=True),
            "contact_frame_counts": json.dumps(summary["contact_frame_counts"], sort_keys=True),
            "collision_event_counts": json.dumps(summary["collision_event_counts"], sort_keys=True),
            "max_queue_lengths": json.dumps(summary["max_queue_lengths"], sort_keys=True),
        }
        rows = []
        if os.path.exists(csv_path):
            with open(csv_path, "r", newline="") as handle:
                reader = csv.DictReader(handle)
                for existing_row in reader:
                    existing_key = existing_row.get("record_tag") or existing_row.get("run_id")
                    if existing_key != summary_key:
                        rows.append(existing_row)
        rows.append(row)
        with open(csv_path, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
            writer.writeheader()
            writer.writerows(rows)
