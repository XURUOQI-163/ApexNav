"""Opt-in integration: real C++ map node, delayed scores and original poses.

Run inside Jazzy after sourcing ROS and install, using the Habitat Python venv.
Uses isolated ROS domain 94; never starts a robot/controller or model downloads.
"""
import importlib.util
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
from plan_env.msg import SemanticScore
from tf_transformations import quaternion_from_euler


def main():
    root = Path(__file__).resolve().parents[1]
    os.environ['ROS_DOMAIN_ID'] = '94'
    rclpy.init(domain_id=94)
    node = rclpy.create_node('semantic_alignment_probe')
    depth_pub = node.create_publisher(Image, '/habitat/camera_depth', 10)
    pose_pub = node.create_publisher(Odometry, '/habitat/sensor_pose', 10)
    score_pub = node.create_publisher(SemanticScore, '/semantic/score', 10)
    bridge = CvBridge()
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.03)
    def frame(x):
        img = bridge.cv2_to_imgmsg(np.full((480,640),.6,np.float32), encoding='32FC1')
        img.header.stamp = node.get_clock().now().to_msg(); img.header.frame_id = 'world'
        pose = Odometry(); pose.header = img.header
        pose.pose.pose.position.x = float(x); pose.pose.pose.position.z = .88
        q = quaternion_from_euler(np.pi/2,np.pi,np.pi/2)
        pose.pose.pose.orientation.x,pose.pose.pose.orientation.y,pose.pose.pose.orientation.z,pose.pose.pose.orientation.w = map(float,q)
        pose_pub.publish(pose);depth_pub.publish(img)
        spin(.12)
        return img.header
    def score(header,value):
        msg=SemanticScore();msg.header=header;msg.backend='clip';msg.score=value
        score_pub.publish(msg);spin(.3)
    with tempfile.TemporaryDirectory(prefix='semantic-alignment-') as tmp:
        path=Path(tmp)
        wrapper=path/'probe.launch.py'
        wrapper.write_text('''import importlib.util
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution

def generate_launch_description():
    spec=importlib.util.spec_from_file_location('algorithm', %r)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    ld=module.generate_launch_description()
    for action in ld.entities:
        if isinstance(action, Node):
            action.cmd.extend([[TextSubstitution(text=x)] for x in ['--ros-args','--log-level','exploration_node:=debug']])
    return ld
''' % str(root/'src/planner/exploration_manager/launch/algorithm_traj.launch.py'))
        log=path/'planner.log'
        with log.open('w') as output:
            process=subprocess.Popen(['ros2','launch',str(wrapper)],cwd=root,
                stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline=time.monotonic()+12
                while not score_pub.get_subscription_count() and time.monotonic()<deadline:spin(.1)
                assert score_pub.get_subscription_count(), 'Map did not subscribe: '+log.read_text()[-2000:]
                spin(.5)
                for _ in range(8): original=frame(0.)
                for _ in range(8): latest=frame(8.)
                score(original,.7)  # Camera already moved to B, return score for A.
                score(original,.9)  # Duplicate cannot overwrite .7.
                score(latest,.2)
                spin(.5)
                text=log.read_text()
                stamp=lambda h: h.stamp.sec*1000000000+h.stamp.nanosec
                a=[line for line in text.splitlines() if f'fused stamp={stamp(original)} ' in line]
                b=[line for line in text.splitlines() if f'fused stamp={stamp(latest)} ' in line]
                assert len(a)==1 and 'pose=(0.000,0.000) score=0.7000' in a[0], text[-5000:]
                assert len(b)==1 and 'pose=(8.000,0.000) score=0.2000' in b[0], text[-5000:]
                assert 'cells=0' not in a[0] and 'cells=0' not in b[0], (a,b)
                print('PASS: delayed A score fused once at A; B score at B; duplicate ignored')
                print(a[0]);print(b[0])
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                Path('/tmp/apexnav-semantic-alignment.log').write_text(log.read_text())
    node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
