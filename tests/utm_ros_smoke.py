"""Small synthetic ROS/recorder checks, run inside the sourced Jazzy container.

No planner, Habitat episode, model server or project data directory is touched.
"""
import json
import tempfile
import time
import unittest
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from plan_env.msg import (FrontierMemoryState, FrontierMemoryNode, FrontierMemoryCell,
                          FrontierScan, SkeletonGraph, SkeletonNode, SkeletonEdge,
                          SkeletonFrontierLink, TopologyObservation)

from topology_memory.recorder import MemoryRecorder
from topology_memory.store import UnifiedTopologyMemory


def header(msg, ns):
    msg.header.frame_id = 'world'
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)


def fixture(ns=200):
    context = dict(session_id='smoke', scene_id='synthetic', episode_id='1', start_position=[0, 0, 0],
                   start_rotation=[0, 0, 0, 1], frame_id='world', coordinate_convention='test',
                   started_at_ns=ns-100, goal_text='chair', model_name='blip2')
    graph = SkeletonGraph()
    header(graph, ns+50)
    graph.map_info.resolution = 1.
    graph.map_info.width = graph.map_info.height = 10
    graph.map_info.origin.orientation.w = 1.
    for node_id, x, y in ((1, 1.5, 1.5), (2, 1.5, 2.5)):
        node = SkeletonNode(node_id=node_id, node_type=1, active=True, position=Point(x=x, y=y))
        node.visit_state=SkeletonNode.CONFIRMED
        node.visit_radius=.5
        node.confirmed_at_ns=ns
        node.visit_position=Point(x=x,y=y)
        graph.nodes.append(node)
    edge = SkeletonEdge(edge_id=1, start_node_id=1, end_node_id=2, length=1., min_clearance=.5)
    edge.path = [n.position for n in graph.nodes]
    graph.edges.append(edge)
    link = SkeletonFrontierLink(frontier_id=7, active=True, connected=True, junction_node_id=1,
                                frontier_centroid=Point(x=2.5, y=1.5), approach=Point(x=1.5, y=1.5), length=0.)
    link.path = [link.approach]
    link.display_path = link.path
    graph.frontier_links.append(link)
    state = FrontierMemoryState(map_epoch_ns=ns-150, last_depth_stamp_ns=ns, backend='blip2')
    header(state, ns+70)
    node = FrontierMemoryNode(frontier_id=7, active=True, centroid=link.frontier_centroid,
                              fused_score=.21, confidence=.5, scored_cells=1)
    node.cells = [FrontierMemoryCell(token=12, pixel_x=2, pixel_y=1, x=2.5, y=1.5,
                                     fused_score=.21, confidence=.5, raw_similarity=.23, last_frame_stamp_ns=ns)]
    state.nodes.append(node)
    rgb = CvBridge().cv2_to_imgmsg(np.full((8, 8, 3), [255, 0, 0], dtype=np.uint8), 'rgb8')
    header(rgb, ns)
    scan = FrontierScan(resolution=1., max_distance=5.)
    header(scan, ns)
    scan.camera_pose.position.x = .5
    scan.camera_pose.position.y = 1.5
    scan.camera_pose.orientation.y = scan.camera_pose.orientation.w = 2**-.5
    scan.free_x, scan.free_y = [0, 1], [1, 1]
    odom = Odometry()
    header(odom, ns)
    odom.pose.pose.orientation.w = 1.
    odom.pose.pose.position.x=1.4
    odom.pose.pose.position.y=1.5
    observation = TopologyObservation(image=rgb, camera_pose=scan.camera_pose, robot_pose=odom.pose.pose,
                                      camera_yaw=0., goal_text='chair', model_name='blip2',
                                      raw_similarity=.23, computed_at_ns=ns+5, cell_ids=[12], weights=[.5])
    header(observation, ns)
    return context, graph, state, (rgb, scan, odom), observation


class RecorderCheck(unittest.TestCase):
    def test_candidate_remote_frames_and_nearby_replacement(self):
        from copy import deepcopy
        with tempfile.TemporaryDirectory() as temp:
            memory=UnifiedTopologyMemory(temp);recorder=MemoryRecorder(memory,CvBridge())
            context,graph,state,frame,_=fixture()
            graph.nodes[0].visit_state=SkeletonNode.CANDIDATE
            recorder.process(context,graph,state,[frame],[])
            node=next(n for n in memory.skeleton['nodes'] if n['source_id']==1)
            self.assertEqual(node['visit_state'],'CANDIDATE')
            self.assertEqual(node['observations'],[])
            graph.nodes[0].visit_state=SkeletonNode.CONFIRMED
            recorder.process(context,graph,state,[],[])
            node=next(n for n in memory.skeleton['nodes'] if n['source_id']==1)
            self.assertEqual(len(node['observations']),1)
            old_path=Path(temp)/node['observations'][0]['image_path']
            closer=deepcopy(frame)
            for msg in closer:header(msg,201)
            closer[2].pose.pose.position.x=1.5
            recorder.process(context,graph,state,[closer],[])
            node=next(n for n in memory.skeleton['nodes'] if n['source_id']==1)
            self.assertEqual(len(node['observations']),1)
            self.assertEqual(node['observations'][0]['stamp_ns'],201)
            self.assertFalse(old_path.exists())
            remote=deepcopy(frame)
            for msg in remote:header(msg,202)
            remote[2].pose.pose.position.x=0.
            recorder.process(context,graph,state,[remote],[])
            node=next(n for n in memory.skeleton['nodes'] if n['source_id']==1)
            self.assertEqual(len(node['observations']),1)
            self.assertEqual(node['observations'][0]['stamp_ns'],201)

    def test_frontier_images_off_preserves_semantics_and_junction_images(self):
        with tempfile.TemporaryDirectory() as temp:
            memory = UnifiedTopologyMemory(temp)
            recorder = MemoryRecorder(memory, CvBridge())
            context, graph, state, frame, scored = fixture()
            self.assertTrue(recorder.process(context, graph, state, [frame], [scored]))
            frontier = memory.get_active_frontiers()[0]
            self.assertNotIn('cells', frontier)
            self.assertNotIn('observations', frontier)
            self.assertAlmostEqual(frontier['semantic']['fused_score'], .21)
            self.assertAlmostEqual(frontier['semantic']['raw_similarity'], .23)
            self.assertEqual(frontier['semantic']['computed_at'], 205)
            self.assertEqual(len(memory.attachments), 1)
            self.assertFalse(list((Path(temp) / 'observations/frontier').rglob('*.jpg')))
            self.assertTrue(list((Path(temp) / 'observations/junction').rglob('*.jpg')))
            self.assertFalse(hasattr(recorder.scored[200], 'image'))

    def test_original_frame_late_score_and_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            memory = UnifiedTopologyMemory(temp, save_frontier_images=True)
            recorder = MemoryRecorder(memory, CvBridge())
            context, graph, state, frame, scored = fixture()
            self.assertTrue(recorder.process(context, graph, state, [frame], []))
            frontier = memory.get_active_frontiers()[0]
            before = memory.get_node_observations(frontier['node_id'])[0]
            self.assertIsNone(before['evidence'])
            # Inference returns after camera collection. Promote the exact saved
            # original; do not overwrite/relabel some newer camera image.
            recorder.process(context, graph, state, [], [scored])
            observation = memory.get_node_observations(frontier['node_id'])[0]
            self.assertEqual(observation['evidence']['raw_similarity'], .23)
            self.assertEqual(observation['stamp_ns'], 200)
            self.assertEqual(before['image_path'], observation['image_path'])
            image = cv2.imread(str(Path(temp) / observation['image_path']))
            self.assertGreater(int(image[0, 0, 2]), 240)
            self.assertLess(int(image[0, 0, 0]), 10)
            restored = UnifiedTopologyMemory(temp, save_frontier_images=True)
            self.assertEqual(restored.load_memory(), [])
            self.assertEqual(len(restored.get_topology_graph()['edges']), 2)

    def test_pre_session_snapshot_and_wrong_goal_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            memory = UnifiedTopologyMemory(temp, save_frontier_images=True)
            recorder = MemoryRecorder(memory, CvBridge())
            context, graph, state, frame, scored = fixture()
            graph.header.stamp.nanosec = 90
            self.assertFalse(recorder.process(context, graph, state, [frame], [scored]))
            self.assertFalse((Path(temp) / 'manifest.json').exists())
            graph.header.stamp.nanosec = 250
            scored.goal_text = 'table'
            recorder.process(context, graph, state, [frame], [scored])
            obs = memory.get_active_frontiers()[0]['observations'][0]
            self.assertIsNone(obs['evidence'])

    def test_incremental_scored_packets_merge_and_upgrade_nearby_view(self):
        from copy import deepcopy
        with tempfile.TemporaryDirectory() as temp:
            memory = UnifiedTopologyMemory(temp, save_frontier_images=True)
            recorder = MemoryRecorder(memory, CvBridge())
            context, graph, state, frame, scored = fixture()
            cell = deepcopy(state.nodes[0].cells[0])
            cell.token = 13
            state.nodes[0].cells.append(cell)
            recorder.process(context, graph, state, [frame], [])
            # A slightly later original at an equivalent camera pose gets a score.
            header(scored, 201)
            scored.image = deepcopy(scored.image)
            header(scored.image, 201)
            second = deepcopy(scored)
            second.cell_ids, second.weights = [13], [.4]
            recorder.process(context, graph, state, [], [scored, second])
            observations = memory.get_active_frontiers()[0]['observations']
            self.assertEqual(len(observations), 1)
            self.assertEqual(observations[0]['stamp_ns'], 201)
            self.assertEqual({c['token'] for c in observations[0]['evidence']['projected_cells']}, {12, 13})

    def test_ros_worker_receives_and_flushes_without_navigation(self):
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        from topology_memory.ros_node import UnifiedTopologyMemoryNode
        with tempfile.TemporaryDirectory() as temp:
            rclpy.init(args=['--ros-args', '-p', 'directory:=' + temp, '-p', 'write_hz:=5.0'])
            recorder = UnifiedTopologyMemoryNode()
            publisher = Node('utm_synthetic_source')
            executor = SingleThreadedExecutor()
            executor.add_node(recorder)
            executor.add_node(publisher)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            context_pub = publisher.create_publisher(String, '/topology_memory/context', qos)
            graph_pub = publisher.create_publisher(SkeletonGraph, '/skeleton_graph/graph', qos)
            state_pub = publisher.create_publisher(FrontierMemoryState, '/topology_memory/frontier_state', qos)
            context, graph, state, _, _ = fixture(time.time_ns())
            try:
                context_pub.publish(String(data=json.dumps(context)))
                graph_pub.publish(graph)
                state_pub.publish(state)
                deadline = time.monotonic() + 8.
                while time.monotonic() < deadline and not (Path(temp) / 'manifest.json').exists():
                    executor.spin_once(timeout_sec=.05)
                self.assertTrue((Path(temp) / 'manifest.json').exists())
            finally:
                recorder.close()
                executor.shutdown()
                recorder.destroy_node()
                publisher.destroy_node()
                rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
