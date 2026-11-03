"""Opt-in real Habitat regression; uses installed assets, isolated ROS domain 94.

Run in the Jazzy/Habitat environment from the repository root. This starts and
cleans up only its own planner/simulator process groups. No models/downloads.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=int, default=550)
    parser.add_argument("--steps", type=int, default=1500)
    parser.add_argument("--timeout", type=float, default=140)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    data = root.parent / "ApexNav-Lite/data"
    episodes = data / "datasets/objectnav/hm3d/v2/val/val.json.gz"
    assert episodes.is_file(), f"Existing HM3D episodes required: {episodes}"
    os.environ["ROS_DOMAIN_ID"] = "94"
    rclpy.init(domain_id=94)
    node = rclpy.create_node("hm3d_frontier_regression")
    states, positions = [], []
    node.create_subscription(String, "/topology/state", lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(Odometry, "/habitat/odom", lambda m: positions.append(
        (m.pose.pose.position.x, m.pose.pose.position.y)), qos_profile_sensor_data)
    logs = Path(tempfile.mkdtemp(prefix="hm3d_frontier_"))
    print(f"LOG_DIR={logs}", flush=True)
    processes, files = [], []
    try:
        commands = [
            ("planner", ["ros2", "launch", "exploration_manager", "simulation.launch.py",
                         "planner:=TOPOLOGY", "mode:=continuous"]),
            ("simulator", ["python", "habitat_evaluation.py", "--mode", "continuous", "--planner",
                           "TOPOLOGY", "--dataset", "hm3dv2", f"test_epi_num={args.episode}",
                           "llm.llm_client.llm_client=none", f"habitat.dataset.data_path={episodes}",
                           f"habitat.dataset.scenes_dir={data / 'scene_datasets'}",
                           "habitat.simulator.habitat_sim_v0.allow_sliding=true",
                           f"habitat.environment.max_episode_steps={args.steps}"]),
        ]
        for name, command in commands:
            output = (logs / f"{name}.log").open("w")
            files.append(output)
            processes.append(subprocess.Popen(command, cwd=root, stdout=output,
                                              stderr=subprocess.STDOUT, start_new_session=True,
                                              env=dict(os.environ, APEXNAV_TOPOLOGY_SNAPSHOT_DIR=str(logs / "topology_snapshot"))))
            if name == "planner":
                deadline = time.monotonic() + 10
                while not states and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=.1)
                assert states and processes[0].poll() is None, "Planner failed to start"
        deadline, previous = time.monotonic() + args.timeout, None
        while time.monotonic() < deadline and processes[-1].poll() is None:
            rclpy.spin_once(node, timeout_sec=.03)
            if states:
                s = states[-1]
                key = (s["state"], s["waypoints"], s["frontiers"], s["edges"], s["converted"])
                if key != previous:
                    print("STATE", key, s["reason"], flush=True)
                    previous = key
            assert processes[0].poll() is None, "Planner exited unexpectedly"
        (logs / "states.json").write_text(json.dumps(states))
        (logs / "positions.json").write_text(json.dumps(positions))
        assert processes[-1].poll() == 0, "Simulator timed out or failed; inspect simulator.log"
        distance = sum(math.dist(a, b) for a, b in zip(positions, positions[1:]))
        displacement = max((math.dist(positions[0], p) for p in positions), default=0.)
        active = [s for s in states if s["waypoints"]]
        assert active, "No graph was built"
        duplicates = max(sum(math.hypot(a["x"] - b["x"], a["y"] - b["y"]) < .35
                             for i, a in enumerate(s["nodes"]) for b in s["nodes"][i+1:])
                         for s in active)
        result = dict(episode=args.episode, traveled=distance, max_displacement=displacement,
                      converted=max(s["converted"] for s in active),
                      peak_waypoints=max(s["waypoints"] for s in active),
                      peak_frontiers=max(s["frontiers"] for s in active),
                      peak_edges=max(s["edges"] for s in active),
                      created_total=max(s["created_total"] for s in active),
                      detours_total=max(s["detours_total"] for s in active),
                      actual_duplicate_pairs=duplicates)
        print(json.dumps(result, indent=2), flush=True)
        (logs / "result.json").write_text(json.dumps(result, indent=2))
        assert displacement > 1. and result["converted"] >= 1, "Rotation never progressed to exploration"
        assert result["peak_frontiers"] > 0 and result["peak_edges"] > 0
        assert duplicates == 0, "Graph contains duplicate nodes within arrival radius"
        print("PASS real HM3D rotation -> exploration -> frontier conversion", flush=True)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=6)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
        for output in files:
            output.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
