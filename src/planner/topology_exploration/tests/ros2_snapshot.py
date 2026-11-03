"""Isolated domain-94 persistence test with real ROS messages and JPEG decoding."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String
from tf_transformations import quaternion_from_euler
from habitat2ros.episode_reset import reset_episode, finish_episode


def main():
    os.environ['ROS_DOMAIN_ID'] = '94'
    rclpy.init(domain_id=94)
    node = rclpy.create_node('snapshot_probe')
    node.state_pub = node.create_publisher(Int32, '/habitat/state', 10)
    node.stop_pub = node.create_publisher(Empty, '/traj_server/stop', 10)
    pubs = {name: node.create_publisher(kind, '/habitat/' + name, 10) for name, kind in
            [('odom', Odometry), ('sensor_pose', Odometry), ('camera_rgb', Image), ('camera_depth', Image)]}
    states, commands, frames = [], [], {}
    node.create_subscription(String, '/topology/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    bridge = CvBridge()
    folder = Path(tempfile.mkdtemp(prefix='topology-snapshot-'))
    snapshot = folder / 'topology_snapshot'
    env = dict(os.environ, APEXNAV_TOPOLOGY_SNAPSHOT_DIR=str(snapshot))
    print(f'LOG_DIR={folder}', flush=True)
    process, outputs = None, []

    def launch():
        output = (folder / f'planner_{len(outputs)}.log').open('w')
        outputs.append(output)
        return subprocess.Popen(['ros2', 'launch', 'exploration_manager', 'simulation.launch.py',
                                 'planner:=TOPOLOGY', 'mode:=continuous'], stdout=output,
                                stderr=subprocess.STDOUT, env=env, start_new_session=True)

    def wait(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.03)

    def feed(x, seconds, delayed_rgb=None):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            yaw = .5 * (time.monotonic() - start)
            stamp = node.get_clock().now().to_msg()
            ns = stamp.sec * 10**9 + stamp.nanosec
            odom = Odometry()
            odom.header.frame_id, odom.header.stamp = 'world', stamp
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            pubs['odom'].publish(odom)
            pose = Odometry(); pose.header = odom.header
            pose.pose.pose.position.x, pose.pose.pose.position.z = float(x), .88
            q = quaternion_from_euler(np.pi/2, np.pi, np.pi/2+yaw)
            (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
             pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = map(float, q)
            value = 30 + len(frames) % 180
            frames[ns] = (value, x, list(map(float, q)))
            rgb = bridge.cv2_to_imgmsg(np.full((480, 640, 3), value, np.uint8), 'rgb8')
            depth = bridge.cv2_to_imgmsg(np.ones((480, 640), np.float32), '32FC1')
            rgb.header = depth.header = odom.header
            pubs['sensor_pose'].publish(pose)
            if delayed_rgb is None: pubs['camera_rgb'].publish(rgb)
            else: delayed_rgb.append(rgb)
            pubs['camera_depth'].publish(depth)
            wait(.12)

    def inspect():
        data = json.loads((snapshot / 'graph.json').read_text())
        assert data['nodes'] and data['edges']
        images = 0
        for n in data['nodes']:
            assert len(n['descriptor']['directions']) == 72
            for img in n['images']:
                expected, x, q = frames[img['stamp_ns']]
                decoded = cv2.imread(str(snapshot / img['path']))
                assert decoded is not None and decoded.shape == (240, 320, 3)
                assert abs(float(decoded.mean()) - expected) <= 1
                assert img['camera_position_xyz_m'] == [x, 0, .88]
                np.testing.assert_allclose(img['orientation_xyzw'], q)
                images += 1
        assert images > 0
        return data, images

    def stop_process():
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()

    try:
        process = launch()
        reset_episode(node, continuous=True)
        feed(0., 6.)
        wait(1.1)
        first, count = inspect()
        # A failed previous background job must not suppress the final retry.
        (snapshot / 'graph.json.tmp').mkdir()
        wait(1.2)
        (snapshot / 'graph.json.tmp').rmdir()
        finishing_at = node.get_clock().now().nanoseconds
        finish_episode(node)
        wait(.8)
        after_stop, _ = inspect()
        assert after_stop['captured_at_ns'] >= finishing_at
        assert {n['id'] for n in after_stop['nodes']} == {n['id'] for n in first['nodes']} and states[-1]['nodes']
        assert commands[-1].linear.x == commands[-1].angular.z == 0
        print(f'PASS stop retains graph and {count} JPEGs with exact image stamp/pose/pixels', flush=True)
        old_start = first['started_at_ns']
        reset_episode(node, continuous=True)
        empty = json.loads((snapshot / 'graph.json').read_text())
        assert not empty['nodes'] and not list((snapshot / 'images').iterdir())
        assert empty['started_at_ns'] > old_start
        late = []
        feed(3., 6., delayed_rgb=late)
        finish_episode(node)
        for rgb in late:
            pubs['camera_rgb'].publish(rgb)
            wait(.03)
        wait(1.3)
        late_graph, late_count = inspect()
        assert late_count > 0
        print('PASS late RGB after stop is persisted; failed previous job does not cancel final retry', flush=True)
        interrupted_at = node.get_clock().now().nanoseconds
        stop_process()  # Ctrl+C planner after the last periodic save: explicit exit flush.
        second, _ = inspect()
        assert second['captured_at_ns'] >= interrupted_at, 'Ctrl+C did not flush the final in-memory graph'
        assert all(n['descriptor']['origin_xy_m'] == [3, 0] for n in second['nodes'])
        saved = (snapshot / 'graph.json').read_bytes()
        process = launch()
        wait(2.)
        assert (snapshot / 'graph.json').read_bytes() == saved
        reset_episode(node, continuous=True)
        assert not json.loads((snapshot / 'graph.json').read_text())['nodes']
        print('PASS Ctrl+C retains files; planner restart retains files; next simulation clears all previous nodes/images', flush=True)
    finally:
        stop_process()
        for output in outputs: output.close()
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
