"""Independent ROS collector; bounded worker performs conversion, JPEG and disk I/O."""
from collections import deque
import fcntl
import json
from pathlib import Path
import threading

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from plan_env.msg import FrontierScan, FrontierMemoryState, SkeletonGraph, TopologyObservation

from .recorder import MemoryRecorder
from .store import UnifiedTopologyMemory, identity


class UnifiedTopologyMemoryNode(Node):
    def __init__(self):
        super().__init__('unified_topology_memory')
        defaults = dict(directory=str(Path(__file__).resolve().parents[1] / 'data/topology'),
                        write_hz=1., frame_window_size=20, observation_queue_size=20,
                        node_match_distance=.5, max_observations_per_node=8,
                        keyframe_angle_degrees=45., keyframe_distance=.75,
                        jpeg_quality=90, model_version='')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda key: self.get_parameter(key).value
        if not 0 < p('write_hz') <= 20 or not 1 <= p('frame_window_size') <= 100 or not 1 <= p('observation_queue_size') <= 100:
            raise ValueError('Invalid UTM worker rate/window/queue capacity')
        if not 0 < p('keyframe_angle_degrees') <= 180 or p('keyframe_distance') <= 0 or not 1 <= p('jpeg_quality') <= 100:
            raise ValueError('Invalid UTM keyframe parameters')
        directory = Path(p('directory')).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        self.writer_lock = (directory / '.writer.lock').open('a')
        try:
            fcntl.flock(self.writer_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'Another UTM recorder already writes {directory}')
        self.memory = UnifiedTopologyMemory(directory, p('node_match_distance'), p('max_observations_per_node'))
        self.recorder = MemoryRecorder(self.memory, CvBridge(), p('frame_window_size'),
                                       p('keyframe_angle_degrees'), p('keyframe_distance'), p('jpeg_quality'), p('model_version'))
        self.condition = threading.Condition()
        self.closed = False
        self.session_context = None
        self.graph = self.state = None
        self.frames = deque(maxlen=p('frame_window_size'))
        self.observations = deque(maxlen=p('observation_queue_size'))
        self.dropped = 0
        self.period = 1. / p('write_hz')
        durable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/topology_memory/context', self._receive_context, durable)
        self.create_subscription(SkeletonGraph, '/skeleton_graph/graph', lambda msg: self._latest('graph', msg), durable)
        self.create_subscription(FrontierMemoryState, '/topology_memory/frontier_state', lambda msg: self._latest('state', msg), durable)
        self.create_subscription(TopologyObservation, '/topology_memory/scored_observation', self._observation, 2)
        self.rgb = message_filters.Subscriber(self, Image, '/habitat/camera_rgb', qos_profile=qos_profile_sensor_data)
        self.scan = message_filters.Subscriber(self, FrontierScan, '/frontier_semantics/scan', qos_profile=20)
        self.odom = message_filters.Subscriber(self, Odometry, '/habitat/odom', qos_profile=20)
        self.sync = message_filters.TimeSynchronizer([self.rgb, self.scan, self.odom], 30)
        self.sync.registerCallback(self._frame)
        self.status_pub = self.create_publisher(String, '/topology_memory/status', durable)
        self.worker = threading.Thread(target=self._work, name='utm-disk-writer', daemon=False)
        self.worker.start()
        self.get_logger().info(f'UTM directory: {directory}; waiting for scene context and fresh graph/frontiers')

    def _latest(self, key, msg):
        with self.condition:
            setattr(self, key, msg)

    def _receive_context(self, msg):
        try:
            data = json.loads(msg.data)
            identity(data)
            if not all(k in data for k in ('session_id', 'started_at_ns', 'goal_text', 'model_name')):
                raise ValueError('Incomplete session metadata')
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Ignoring invalid UTM context: {exc}')
            return
        with self.condition:
            if self.session_context and self.session_context['session_id'] != data['session_id']:
                self.frames.clear()
                self.observations.clear()
            self.session_context = data

    def _frame(self, rgb, scan, odom):
        with self.condition:
            self.frames.append((rgb, scan, odom))

    def _observation(self, msg):
        with self.condition:
            if len(self.observations) == self.observations.maxlen:
                self.dropped += 1
            self.observations.append(msg)

    def _work(self):
        try:
            issues = self.memory.load_memory()
            for issue in issues:
                self.get_logger().warning('UTM load: ' + issue)
        except Exception as exc:
            self.get_logger().error(f'UTM load failed; persistence disabled to preserve existing files: {exc}')
            return
        last_report = None
        while True:
            with self.condition:
                if not self.closed:
                    self.condition.wait(timeout=self.period)
                closed = self.closed
                context, graph, state = self.session_context, self.graph, self.state
                frames, observations = list(self.frames), list(self.observations)
                # Clear only after a usable context/graph exists, otherwise a
                # delayed first Skeleton build would lose scored originals.
            try:
                saved = self.recorder.process(context, graph, state, frames, observations)
                if saved:
                    with self.condition:
                        consumed = {id(o) for o in observations}
                        self.observations = deque((o for o in self.observations if id(o) not in consumed),
                                                  maxlen=self.observations.maxlen)
                    status = dict(directory=str(self.memory.root), session_id=self.memory.manifest['session_id'],
                                  junctions=sum(n['status'] == 'ACTIVE' for n in self.memory.skeleton['nodes']),
                                  skeleton_edges=sum(e['status'] == 'ACTIVE' for e in self.memory.skeleton['edges']),
                                  active_frontiers=len(self.memory.get_active_frontiers()),
                                  observations=sum(len(n['observations']) for n in self.memory.skeleton['nodes'] + self.memory.frontiers),
                                  dropped_observation_packets=self.dropped)
                    report = json.dumps(status)
                    if report != last_report:
                        if rclpy.ok():
                            self.status_pub.publish(String(data=report))
                        self.get_logger().info('UTM saved: ' + report)
                        last_report = report
            except Exception as exc:
                self.get_logger().error(f'UTM snapshot failed (navigation unaffected): {exc}')
            if closed:
                break

    def close(self):
        with self.condition:
            self.closed = True
            self.condition.notify_all()
        self.worker.join()
        self.writer_lock.close()


def main():
    rclpy.init()
    node = None
    try:
        node = UnifiedTopologyMemoryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
