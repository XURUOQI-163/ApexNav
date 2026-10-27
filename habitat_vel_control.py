"""Continuous Habitat simulation driven by the trajectory controller's /cmd_vel.

Use habitat_evaluation.py --mode continuous for the shared dataset/override CLI.
The perception bridge runs in a child process so HTTP inference cannot block the
velocity integration loop. YOLOE and CLIP model servers remain external.
"""
import json
import math
import multiprocessing
import time

import habitat
import habitat_sim
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from habitat.config.default import patch_config
from habitat.config.default_structured_configs import CollisionsMeasurementConfig
from habitat.sims.habitat_simulator.actions import HabitatSimActions
from habitat_sim.utils import common as utils
from omegaconf import OmegaConf
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

from basic_utils.velocity_command import VelocityCommand
from habitat2ros.habitat_publisher import ROSPublisherNonNode
from params import HABITAT_STATE
from vlm.label_utils import normalize_objectnav_label


class HabitatVelControlNode(Node):
    def __init__(self, command_timeout=.5):
        super().__init__('habitat_velocity_simulator')
        self.command = VelocityCommand(command_timeout)
        self.stop_requested = False
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 1)
        self.create_subscription(Int32, '/habitat/plan_action', self.action_callback, 10)
        self.label_pub = self.create_publisher(String, '/detector/label', 1)
        self.trigger_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.stop_pub = self.create_publisher(Empty, '/traj_server/stop', 10)
        self.state_pub = self.create_publisher(Int32, '/habitat/state', 10)
        self.ros_pub = ROSPublisherNonNode(self)

    def cmd_vel_callback(self, msg):
        self.command.update(msg.linear.x, msg.angular.z)

    def action_callback(self, msg):
        # The trajectory FSM only uses this topic to request task completion.
        if msg.data == 0:
            self.stop_requested = True
            self.command.stop()

    def wait_for_controller(self, timeout=10.):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if self.count_publishers('/cmd_vel'):
                return
        raise RuntimeError(
            'No /cmd_vel publisher. Start the planner with '
            'ros2 launch exploration_manager simulation.launch.py mode:=continuous '
            'in the same ROS_DOMAIN_ID, and stop the discrete planner first.')


def _run_perception(config, stop_event):
    """Spawn entry point: a fresh ROS context, without creating a Habitat scene."""
    from real_world_test_example.real_world_test_habitat import RealWorldNode
    rclpy.init()
    node = RealWorldNode(OmegaConf.create(config))
    try:
        while rclpy.ok() and not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_simulation(cfg, node, perception_process):
    cfg = patch_config(cfg)
    fps = float(cfg.continuous.control_hz)
    scan_seconds = float(cfg.continuous.initial_scan_seconds)
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError('continuous.control_hz must be positive and finite')
    if not math.isfinite(scan_seconds) or scan_seconds < 0:
        raise ValueError('continuous.initial_scan_seconds must be nonnegative and finite')
    if cfg.habitat.simulator.forward_step_size != 0:
        raise ValueError('Continuous mode requires habitat.simulator.forward_step_size=0 to avoid double motion')
    if cfg.need_video:
        raise ValueError('Continuous mode currently publishes live RGB/RViz only; use need_video=false')
    time_step = 1. / fps
    episode_index = int(cfg.test_epi_num)
    if episode_index < -1:
        raise ValueError('test_epi_num must be -1 or a nonnegative episode index')
    # Continuous mode is a single-episode runner; -1 selects the first episode.
    episode_index = max(0, episode_index)
    with habitat.config.read_write(cfg):
        cfg.habitat.task.measurements.collisions = CollisionsMeasurementConfig()

    with habitat.Env(cfg) as env:
        if episode_index >= len(env.episodes):
            raise ValueError(f'Episode index {episode_index} is outside [0, {len(env.episodes) - 1}]')
        env.current_episode = env.episodes[episode_index]
        observations = env.reset()
        episode = env.current_episode
        label = normalize_objectnav_label(episode.object_category, cfg.habitat.dataset.data_path)
        print(f'Continuous mode: index={episode_index}, episode_id={episode.episode_id}, '
              f'scene={episode.scene_id}, target={label}', flush=True)

        sim = env.sim
        agent = sim.agents[0]
        control = habitat_sim.physics.VelocityControl()
        control.controlling_lin_vel = control.controlling_ang_vel = True
        control.lin_vel_is_local = control.ang_vel_is_local = True
        elapsed_sim_time = 0.
        last_trigger_time = -math.inf
        last_label_time = -math.inf
        frames = 0

        while rclpy.ok() and not env.episode_over:
            loop_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.)
            if not perception_process.is_alive():
                raise RuntimeError(f'Visual processing node exited ({perception_process.exitcode}); see its traceback')
            if node.stop_requested:
                env.step(HabitatSimActions.stop)
                break

            linear, angular = node.command.sample()
            if elapsed_sim_time < scan_seconds:
                # Preserve the existing initial panoramic scan before triggering
                # trajectory planning. No translation while collecting the map.
                linear, angular = 0., math.pi / 2.
            control.linear_velocity = np.array([0., 0., -linear])
            control.angular_velocity = np.array([0., angular, 0.])
            state = agent.state
            previous = habitat_sim.RigidState(utils.quat_to_magnum(state.rotation), state.position)
            target = control.integrate_transform(time_step, previous)
            end_position = sim.step_filter(previous.translation, target.translation)
            actual_speed = float(np.linalg.norm(np.asarray(end_position) - np.asarray(previous.translation))) / time_step
            actual_speed = math.copysign(actual_speed, linear) if linear else 0.
            state.position = end_position
            state.rotation = utils.quat_from_magnum(target.rotation)
            agent.set_state(state)

            # A zero-distance action advances task bookkeeping/sensors only;
            # translation has already been integrated and NavMesh-filtered above.
            observations = env.step(HabitatSimActions.move_forward)
            observations['camera_pitch'] = 0.
            observations['linear_velocity'] = actual_speed
            observations['angular_velocity'] = angular
            node.ros_pub.habitat_publish_ros_topic(observations)
            if loop_start - last_label_time >= .5:
                node.label_pub.publish(String(data=label))
                last_label_time = loop_start
            if elapsed_sim_time >= scan_seconds and loop_start - last_trigger_time >= .5:
                node.trigger_pub.publish(PoseStamped())
                last_trigger_time = loop_start
            frames += 1
            elapsed_sim_time += time_step
            if frames % max(1, round(fps * 5)) == 0:
                node.get_logger().info(f'Finding {label}: v={actual_speed:.3f} m/s, w={angular:.3f} rad/s, tick={frames}')
            time.sleep(max(0., time_step - (time.monotonic() - loop_start)))

        metrics = env.get_metrics()
        summary = {key: metrics[key] for key in ['success','spl','soft_spl','distance_to_goal','collisions'] if key in metrics}
        print('Continuous episode metrics: ' + json.dumps(summary, default=float), flush=True)
        print('Continuous ticks and time budgets differ from discrete action evaluation.', flush=True)


def run_continuous(cfg):
    """Own the simulator ROS node and the automatically managed visual bridge."""
    rclpy.init()
    node = None
    process = None
    stop_event = None
    try:
        node = HabitatVelControlNode(command_timeout=float(cfg.continuous.command_timeout))
        node.wait_for_controller()
        # Resolve the selected dataset configuration before passing it to the
        # visual bridge, including its matching LLM cache and camera parameters.
        cfg = patch_config(cfg)
        context = multiprocessing.get_context('spawn')
        stop_event = context.Event()
        process = context.Process(
            target=_run_perception, args=(OmegaConf.to_container(cfg, resolve=True), stop_event),
            name='apexnav_continuous_perception')
        process.start()
        run_simulation(cfg, node, process)
    except KeyboardInterrupt:
        print('Continuous simulation interrupted.', flush=True)
    finally:
        if node is not None:
            node.command.stop()
            if rclpy.ok():
                node.stop_pub.publish(Empty())
                node.state_pub.publish(Int32(data=HABITAT_STATE.EPISODE_FINISH))
                rclpy.spin_once(node, timeout_sec=.1)
        if process is not None:
            stop_event.set()
            # Let an in-flight HTTP inference finish before shutting down ROS.
            process.join(timeout=2.)
            if process.is_alive():
                process.terminate()
            process.join(timeout=5.)
            if process.is_alive():
                process.kill()
                process.join(timeout=2.)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import sys
    from habitat_evaluation import run_cli
    run_cli(['--mode', 'continuous', *sys.argv[1:]])
