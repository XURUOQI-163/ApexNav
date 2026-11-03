"""Opt-in real launch/RGB-D-map Skeleton Graph regression (ROS domain 94).

No Habitat, vision service or robot required. Uses existing exact-time depth/pose
input, APEXNAV continuous planner, reset service and MarkerArray subscribers.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Int32
from visualization_msgs.msg import MarkerArray
from plan_env.msg import SkeletonGraph
from tf_transformations import quaternion_from_euler
from habitat2ros.episode_reset import reset_episode, finish_episode


def main():
    os.environ['ROS_DOMAIN_ID'] = '94'
    root = Path(__file__).resolve().parents[1]
    directory = Path(tempfile.mkdtemp(prefix='skeleton-ros-'))
    print('LOG_DIR=' + str(directory), flush=True)
    rclpy.init(domain_id=94)
    node = rclpy.create_node('skeleton_probe')
    node.state_pub = node.create_publisher(Int32, '/habitat/state', 10)
    depth_pub = node.create_publisher(Image, '/habitat/camera_depth', 10)
    pose_pub = node.create_publisher(Odometry, '/habitat/sensor_pose', 10)
    graphs, markers, frontiers, inflated = [], [], [], []
    from rclpy.qos import QoSProfile, DurabilityPolicy
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(SkeletonGraph, '/skeleton_graph/graph', graphs.append, qos),
        node.create_subscription(MarkerArray, '/skeleton_graph/markers', markers.append, qos),
        node.create_subscription(MarkerArray, '/frontier_semantics/markers', frontiers.append, 10),
        node.create_subscription(PointCloud2, '/grid_map/occupied_inflate', inflated.append, 1),
    ]
    bridge = CvBridge()

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)

    def sense(x=0., seconds=4.):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # The existing Habitat bridge publishes normalized depth [0,1];
            # 0.3 corresponds to a wall around 1.5m, below the 4.99m range cap.
            image = bridge.cv2_to_imgmsg(np.full((480, 640), .3, np.float32), encoding='32FC1')
            image.header.stamp = node.get_clock().now().to_msg()
            image.header.frame_id = 'world'
            pose = Odometry(); pose.header = image.header
            pose.pose.pose.position.x = x; pose.pose.pose.position.z = .88
            q = quaternion_from_euler(np.pi/2, np.pi, np.pi/2)
            pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = map(float, q)
            pose_pub.publish(pose); depth_pub.publish(image); spin(.1)

    with (directory/'planner.log').open('w') as output:
        process = subprocess.Popen([
            'ros2', 'launch', 'exploration_manager', 'simulation.launch.py',
            'planner:=APEXNAV', 'mode:=continuous', 'topology:=none', 'skeleton:=true',
        ], cwd=root, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+20
            while not depth_pub.get_subscription_count() and time.monotonic()<deadline:
                if process.poll() is not None:
                    raise AssertionError((directory/'planner.log').read_text()[-5000:])
                spin(.1)
            assert depth_pub.get_subscription_count(), 'Map did not start'
            reset_episode(node, continuous=True)
            sense(); spin(3.)
            assert graphs and graphs[-1].nodes and graphs[-1].frontier_links, (directory/'planner.log').read_text()[-5000:]
            graph = graphs[-1]
            assert graph.header.frame_id == 'world'
            assert graph.map_info.width == 1600 and graph.map_info.height == 1600
            assert abs(graph.map_info.resolution-.05)<1e-6
            assert graph.map_info.origin.position.x == -40.
            assert all(n.node_type != n.ENDPOINT for n in graph.nodes), 'Deadend nodes were retained'
            assert graph.removed_deadends > 0, 'No deadends removed in fixture'
            connected = [c for c in graph.frontier_links if c.connected]
            assert connected, 'No Frontier connected to the retained Junction'
            ids = {n.node_id for n in graph.nodes}
            assert len(ids)==len(graph.nodes)
            for edge in graph.edges:
                assert edge.start_node_id in ids and edge.end_node_id in ids
                assert len(edge.path)==len(edge.pixel_x)==len(edge.pixel_y)>=2
                length=sum(np.hypot(b.x-a.x,b.y-a.y) for a,b in zip(edge.path,edge.path[1:]))
                assert abs(length-edge.length)<1e-8
            assert inflated and inflated[-1].width, 'Inflation fixture missing'
            # This is test-only inspection of the existing visualization output;
            # production SkeletonGraphManager reads the internal getters directly.
            blocked = {(round(float(p[0]), 3), round(float(p[1]), 3)) for p in
                       point_cloud2.read_points(inflated[-1], field_names=('x','y'), skip_nans=True)}
            assert all((round(p.x, 3), round(p.y, 3)) not in blocked
                       for edge in graph.edges for p in edge.path), 'Skeleton crossed live inflated occupancy'
            for c in connected:
                assert c.junction_node_id in ids and c.path
                assert (round(c.display_path[0].x, 3), round(c.display_path[0].y, 3)) == next(
                    (round(n.position.x, 3), round(n.position.y, 3))
                    for n in graph.nodes if n.node_id == c.junction_node_id)
                assert (c.display_path[-1].x,c.display_path[-1].y)==(c.frontier_centroid.x,c.frontier_centroid.y)
                assert all((round(p.x, 3), round(p.y, 3)) not in blocked for p in c.path)
            cyan = next(m for m in markers[-1].markers if m.ns == 'frontier_links')
            assert len(cyan.points) == sum(2*(len(c.display_path)-1) for c in connected)
            original_centres={(p.x,p.y,p.z) for m in frontiers[-1].markers if m.ns.endswith('_centers') for p in m.points}
            offset=0
            for c in connected:
                offset+=2*(len(c.display_path)-1)
                endpoint=cyan.points[offset-1]
                assert (endpoint.x,endpoint.y,endpoint.z) in original_centres, 'Cyan line misses actual Frontier sphere centre'
            assert not any(m.ns=='frontier_approaches' for m in markers[-1].markers), 'Old approach spheres still present'
            assert any(m.points for m in frontiers[-1].markers if 'boundaries' in m.ns), 'Original Frontier disappeared'
            edge_marker = next(m for m in markers[-1].markers if m.ns=='edges')
            assert edge_marker.color.b==1. and edge_marker.color.r==0.
            assert len(edge_marker.points)==sum(2*(len(e.display_path)-1) for e in graph.edges)
            assert not node.get_publishers_info_by_topic('/sparse_topology/markers'), 'Old descriptor topology was enabled'
            assert len(node.get_publishers_info_by_topic('/cmd_vel'))==1, 'Velocity publishers changed'
            spin(2.); assert graphs[-1].revision==graph.revision, 'Unchanged map rebuilt'
            finish_episode(node); spin(1.)
            assert [(n.node_id,n.position.x,n.position.y) for n in graphs[-1].nodes] == [(n.node_id,n.position.x,n.position.y) for n in graph.nodes]
            print(f'PASS APEXNAV launch, original Frontier, independent Skeleton N{len(graph.nodes)}/E{len(graph.edges)}, deadends removed, safe Frontier links, full paths, unchanged-map skip, finish retention; sample={graph.sampling_ms:.1f}ms build={graph.build_ms:.1f}ms', flush=True)
            reset_episode(node, continuous=True); spin(2.)
            assert not graphs[-1].nodes and not graphs[-1].edges and not graphs[-1].frontier_links, 'Reset retained old skeleton or Frontier links'
            assert not any(m.points for m in markers[-1].markers), 'Reset retained old skeleton markers'
            sense(3.); spin(3.)
            assert graphs[-1].nodes and all(n.position.x>2.5 for n in graphs[-1].nodes), 'Old scene leaked into new graph'
            print('PASS reset clears graph/markers and next scene rebuilds independently', flush=True)
        finally:
            process.send_signal(signal.SIGINT)
            try: process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM); process.wait(timeout=10)
            node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
