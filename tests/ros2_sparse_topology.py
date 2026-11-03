"""Real ROS2 mapper integration, synthetic forward depth/RGB/poses, domain 94.

Run from the repo with Jazzy + install + Habitat Python venv sourced. No Habitat
scene, models, controller, odometry or exploration trigger is started by this test.
"""
import json
import math
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
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler
from habitat2ros.episode_reset import reset_episode


def main():
    root = Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID'] = '94'
    rclpy.init(domain_id=94)
    node = rclpy.create_node('sparse_topology_probe')
    depth_pub = node.create_publisher(Image, '/habitat/camera_depth', 10)
    rgb_pub = node.create_publisher(Image, '/habitat/camera_rgb', 10)
    pose_pub = node.create_publisher(Odometry, '/habitat/sensor_pose', 10)
    states, markers = [], []
    node.create_subscription(String, '/sparse_topology/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(MarkerArray, '/sparse_topology/markers', markers.append, 10)
    bridge = CvBridge()
    sent = set()

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)

    def frame(x=0., y=0., yaw=0.):
        depth = bridge.cv2_to_imgmsg(np.ones((480, 640), np.float32), '32FC1')
        depth.header.stamp = node.get_clock().now().to_msg()
        depth.header.frame_id = 'world'
        rgb = bridge.cv2_to_imgmsg(np.full((480, 640, 3), 100, np.uint8), 'rgb8')
        rgb.header = depth.header
        pose = Odometry(); pose.header = depth.header
        pose.pose.pose.position.x = float(x); pose.pose.pose.position.y = float(y)
        pose.pose.pose.position.z = .88
        q = quaternion_from_euler(np.pi/2, np.pi, np.pi/2+yaw)
        (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
         pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = map(float, q)
        stamp = depth.header.stamp.sec*1000000000 + depth.header.stamp.nanosec
        sent.add(stamp)
        # Repeating an exact timestamp is harmless at the mapper, and avoids DDS
        # discovery/loss turning the geometry fixture into a flaky timing test.
        for _ in range(2):
            rgb_pub.publish(rgb); pose_pub.publish(pose); depth_pub.publish(depth)
            spin(.12)
        return rgb, pose, depth

    with tempfile.TemporaryDirectory(prefix='sparse-topology-') as tmp:
        wrapper = Path(tmp)/'probe.launch.py'
        wrapper.write_text('''import importlib.util
from launch import LaunchDescription
from launch_ros.actions import SetParameter

def generate_launch_description():
    spec = importlib.util.spec_from_file_location('algorithm', %r)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    ld = module.generate_launch_description()
    return LaunchDescription([
        SetParameter(name='sparse_topology.update_hz', value=10.0),
        *ld.entities])
''' % str(root/'src/planner/exploration_manager/launch/simulation.launch.py'))
        log = Path(tmp)/'planner.log'
        with log.open('w') as output:
            process = subprocess.Popen(['ros2', 'launch', str(wrapper), 'mode:=discrete', 'topology:=sparse'], cwd=root,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic()+15
                while time.monotonic()<deadline and not (depth_pub.get_subscription_count() and rgb_pub.get_subscription_count()):
                    spin(.1)
                assert depth_pub.get_subscription_count() and rgb_pub.get_subscription_count(), log.read_text()[-3000:]
                spin(.5)
                # Full turns progressively reveal directions and clear the original
                # map's initial unknown footprint; each input is a forward view.
                for _ in range(3):
                    for degrees in range(0, 360, 30): frame(yaw=math.radians(degrees))
                for _ in range(8): frame()
                spin(1.2)
                assert states, 'No topology state: '+log.read_text()[-3000:]
                first = states[-1]
                assert first['waypoints'] == 1 and first['frontiers'] > 0, first
                assert first['edges'], first
                first_ids = {n['id'] for n in first['nodes']}
                for _ in range(8): old_frame = frame()
                spin(1.2)
                steady = states[-1]
                assert {n['id'] for n in steady['nodes']} == first_ids, 'Repeated views grew/replaced nodes'
                duplicates = sum(math.hypot(a['x']-b['x'], a['y']-b['y']) < .4
                    for i,a in enumerate(steady['nodes']) for b in steady['nodes'][i+1:])
                assert duplicates == 0, steady
                images = [i for n in steady['nodes'] for i in n['images'] if i['bytes']]
                assert images and all(i['stamp'] in sent for i in images), 'Image stamp mismatch'
                wp = next(n for n in steady['nodes'] if n['type'] == 'WAYPOINT')
                assert all(s['mask'] == 63 for s in wp['sectors']), 'In-place turns did not complete descriptor'
                assert all(s['state'] == 2 for s in wp['sectors']), 'Valid max-depth must represent free observations'
                latest = markers[-1].markers
                green = [m for m in latest if m.ns == 'topology_nodes' and m.color.g == 1.]
                red = [m for m in latest if m.ns == 'topology_nodes' and m.color.r == 1.]
                blue = [m for m in latest if m.ns == 'topology_edges' and m.color.b == 1. and m.points]
                labels = [m for m in latest if m.ns == 'topology_ids' and m.text]
                assert len(green) == 1 and len(red) == steady['frontiers'] and blue and len(labels) == len(first_ids)
                print('PASS ROS steady: waypoints=%d frontiers=%d edges=%d actual_duplicates=%d duplicate_rejections=%d images=%d' %
                      (steady['waypoints'], steady['frontiers'], len(steady['edges']), duplicates, steady['duplicates'], len(images)), flush=True)
                # Synthetic arrival (not a navigation-performance experiment).
                target = next(n for n in steady['nodes'] if n['type'] == 'FRONTIER')
                yaw = math.atan2(target['y'], target['x'])
                for _ in range(6): frame(target['x'], target['y'], yaw)
                spin(1.2)
                arrived = states[-1]
                same = next(n for n in arrived['nodes'] if n['id'] == target['id'])
                assert same['type'] == 'WAYPOINT' and same['measured_at_node'], same
                assert math.hypot(same['descriptor_origin'][0]-target['x'], same['descriptor_origin'][1]-target['y']) < 1e-4
                by_id = {n['id']:n for n in arrived['nodes']}
                for edge in arrived['edges']:
                    a,b=by_id[edge['a']],by_id[edge['b']]
                    assert abs(edge['weight']-math.hypot(a['x']-b['x'], a['y']-b['y'])) < 1e-4
                print('PASS ROS arrival: same frontier ID converted; waypoints=%d frontiers=%d edges=%d' %
                      (arrived['waypoints'], arrived['frontiers'], len(arrived['edges'])), flush=True)
                reset_episode(node)
                spin(.8)
                assert not states[-1]['nodes'] and not states[-1]['edges'], 'Old topology survived reset'
                assert all(m.action == Marker.DELETEALL or not m.points
                           for m in markers[-1].markers), 'Old markers survived reset'
                rgb_pub.publish(old_frame[0]); pose_pub.publish(old_frame[1]); depth_pub.publish(old_frame[2])
                spin(.6)
                assert not states[-1]['nodes'], 'Queued old frame recreated topology'
                print('PASS ROS reset: graph/edges/images cleared; old timestamp rejected', flush=True)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try: process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL); process.wait()
                Path('/tmp/apexnav-sparse-topology.log').write_text(log.read_text())
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
