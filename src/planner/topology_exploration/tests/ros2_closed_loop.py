"""Synthetic ROS closed loop; no Habitat scene, model service or hardware required.

Source Jazzy, workspace and the Python ROS venv. Run from the repository root.
Uses domain 94, starts/stops only its own launch process.
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf_transformations import quaternion_from_euler


def main():
    os.environ['ROS_DOMAIN_ID'] = '94'
    rclpy.init(domain_id=94)
    node = rclpy.create_node('topology_closed_loop_probe')
    publishers = {name: node.create_publisher(kind, '/habitat/'+name, 10)
                  for name, kind in [('odom', Odometry), ('sensor_pose', Odometry),
                                     ('camera_rgb', Image), ('camera_depth', Image)]}
    states, commands, maps = [], [], []
    node.create_subscription(String, '/topology/state', lambda m: states.append(json.loads(m.data)), 10)
    node.create_subscription(Twist, '/cmd_vel', commands.append, 10)
    node.create_subscription(OccupancyGrid, '/topology/local_map', maps.append, 10)
    bridge = CvBridge()
    position = np.zeros(2)
    yaw = 0.
    traveled = 0.
    previous_tick = time.monotonic()
    last_depth = 0.

    def step(send_depth=True, invalid_depth=False, send_odom=True, depth_value=1.):
        nonlocal yaw, previous_tick, last_depth, traveled
        current = time.monotonic()
        dt = min(.1, current-previous_tick)
        previous_tick = current
        v, w = (commands[-1].linear.x, commands[-1].angular.z) if commands else (0., 0.)
        position[:] += v*dt*np.array([math.cos(yaw+w*dt/2), math.sin(yaw+w*dt/2)])
        traveled += abs(v)*dt
        yaw += w*dt
        stamp = node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'world'
        odom.pose.pose.position.x, odom.pose.pose.position.y = map(float, position)
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v, w
        if send_odom:
            publishers['odom'].publish(odom)
        if send_depth and current-last_depth >= .105:
            last_depth = current
            depth = bridge.cv2_to_imgmsg(np.full((480, 640), 0. if invalid_depth else depth_value, np.float32), '32FC1')
            depth.header = odom.header
            rgb = bridge.cv2_to_imgmsg(np.full((480, 640, 3), 100, np.uint8), 'rgb8')
            rgb.header = odom.header
            pose = Odometry(); pose.header = odom.header
            pose.pose.pose.position.x, pose.pose.pose.position.y = map(float, position)
            pose.pose.pose.position.z = .88
            q = quaternion_from_euler(np.pi/2, np.pi, np.pi/2+yaw)
            (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
             pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = map(float, q)
            publishers['sensor_pose'].publish(pose)
            publishers['camera_rgb'].publish(rgb)
            publishers['camera_depth'].publish(depth)
        rclpy.spin_once(node, timeout_sec=.02)

    def run(seconds, **kwargs):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            step(**kwargs)

    with tempfile.TemporaryDirectory(prefix='topology-loop-') as temp:
        log = Path(temp)/'planner.log'
        with log.open('w') as output:
            process = subprocess.Popen(['ros2', 'launch', 'exploration_manager', 'simulation.launch.py',
                'planner:=TOPOLOGY', 'mode:=continuous'], stdout=output, stderr=subprocess.STDOUT,
                start_new_session=True,
                env=dict(os.environ, APEXNAV_TOPOLOGY_SNAPSHOT_DIR=str(Path(temp) / "topology_snapshot")))
            try:
                deadline = time.monotonic()+15
                while time.monotonic()<deadline and not publishers['odom'].get_subscription_count():
                    rclpy.spin_once(node, timeout_sec=.1)
                assert publishers['odom'].get_subscription_count(), log.read_text()[-4000:]
                assert node.count_publishers('/cmd_vel') == 1
                deadline = time.monotonic()+90
                while time.monotonic()<deadline:
                    step()
                    if states and states[-1]['converted'] >= 2 and traveled > 4.:
                        break
                assert states, log.read_text()[-4000:]
                # Minimum-cost exploration can turn back toward another nearby
                # frontier; radial displacement is not its success criterion.
                assert traveled > 4. and states[-1]['converted'] >= 2, (position, states[-1], log.read_text()[-2000:])
                assert maps and all(m.info.width == 120 and len(m.data) == 14400 for m in maps)
                assert any(abs(m.info.origin.position.x+3) > .5 or abs(m.info.origin.position.y+3) > .5 for m in maps)
                assert all(abs(c.linear.x) <= .40001 and abs(c.angular.z) <= 1.00001 for c in commands)
                # Normal increments are bounded; emergency stops intentionally jump to zero.
                for a, b in zip(commands, commands[1:]):
                    if abs(b.linear.x)+abs(b.angular.z) > 1e-8:
                        assert abs(b.linear.x-a.linear.x) <= .02501, (a, b)
                        assert abs(b.angular.z-a.angular.z) <= .05001, (a, b)
                print('PASS ROS closed loop: traveled=%.2f converted=%d waypoints=%d frontiers=%d edges=%d cells=%d' %
                      (traveled, states[-1]['converted'], states[-1]['waypoints'], states[-1]['frontiers'], states[-1]['edges'], len(maps[-1].data)), flush=True)
                run(1.7, send_depth=False)
                assert commands[-1].linear.x == commands[-1].angular.z == 0., states[-1]
                print('PASS depth timeout: zero v,w', flush=True)
                run(1.4, invalid_depth=True)
                assert commands[-1].linear.x == commands[-1].angular.z == 0., states[-1]
                print('PASS invalid depth cannot refresh watchdog', flush=True)
                run(1.0)
                run(1.0, send_odom=False)
                assert commands[-1].linear.x == commands[-1].angular.z == 0., states[-1]
                print('PASS odometry timeout: zero v,w', flush=True)
                run(1.)
                run(.6, depth_value=.035)  # Fresh wall intrudes into the footprint.
                assert commands[-1].linear.x == commands[-1].angular.z == 0., states[-1]
                assert any(v == 100 for v in maps[-1].data)
                print('PASS fresh close obstacle: immediate occupied cells and zero v,w', flush=True)
                client = node.create_client(Trigger, '/exploration/reset')
                assert client.wait_for_service(timeout_sec=2)
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=3)
                assert future.result().success
                end = time.monotonic()+.8
                while time.monotonic()<end:
                    rclpy.spin_once(node, timeout_sec=.05)
                assert states[-1]['nodes'] == [] and states[-1]['edges'] == 0, states[-1]
                assert all(v == -1 for v in maps[-1].data)
                assert commands[-1].linear.x == commands[-1].angular.z == 0.
                print('PASS reset clears graph, rolling map, target and command', flush=True)
            except BaseException:
                print(log.read_text()[-5000:], flush=True)
                raise
            finally:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL); process.wait()
                node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
