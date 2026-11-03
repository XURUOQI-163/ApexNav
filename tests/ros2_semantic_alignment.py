"""Opt-in integration: real C++ map node, delayed scores and original poses.

Run inside Jazzy after sourcing ROS and install, using the Habitat Python venv.
Uses isolated ROS domain 94; never starts a robot/controller or model downloads.
"""
import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import re
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from plan_env.msg import SemanticScore, FrontierScan
from basic_utils.frontier_frame_window import Frame, project_cells
from visualization_msgs.msg import MarkerArray, Marker
from habitat2ros.episode_reset import reset_episode, finish_episode
from std_msgs.msg import Int32
from tf_transformations import quaternion_from_euler


def main():
    root = Path(__file__).resolve().parents[1]
    continuous = '--continuous' in sys.argv
    os.environ['ROS_DOMAIN_ID'] = '94'
    rclpy.init(domain_id=94)
    node = rclpy.create_node('semantic_alignment_probe')
    node.state_pub = node.create_publisher(Int32, '/habitat/state', 10)
    depth_pub = node.create_publisher(Image, '/habitat/camera_depth', 10)
    pose_pub = node.create_publisher(Odometry, '/habitat/sensor_pose', 10)
    score_pub = node.create_publisher(SemanticScore, '/semantic/score', 10)
    odom_pub = node.create_publisher(Odometry, '/habitat/odom', 10)
    scans={}
    stamp=lambda h: h.stamp.sec*1000000000+h.stamp.nanosec
    node.create_subscription(FrontierScan,'/frontier_semantics/scan',lambda m:scans.update({stamp(m.header):m}),20)
    clouds=[]
    node.create_subscription(PointCloud2,'/frontier_semantics/score',clouds.append,10)
    free_clouds=[]
    labels=[]
    markers=[]
    paths=[]
    frontier_markers=[]
    node.create_subscription(PointCloud2,'/grid_map/free',free_clouds.append,10)
    node.create_subscription(MarkerArray,'/frontier_semantics/labels',labels.append,10)
    node.create_subscription(Marker,'/planning_vis/frontier',markers.append,10)
    node.create_subscription(Marker,'/travel_traj',paths.append,10)
    node.create_subscription(MarkerArray,'/frontier_semantics/markers',frontier_markers.append,10)
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
        # Sensor topics are BEST_EFFORT at the map. Confirm the fixture's exact
        # stamp was acquired before advancing; a lost pair is validly discarded
        # by production and must not make this alignment test nondeterministic.
        marker=f'snapshot stamp={img.header.stamp.sec*1000000000+img.header.stamp.nanosec} '
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            pose_pub.publish(pose);depth_pub.publish(img)
            spin(.15)
            if marker in log.read_text() and stamp(img.header) in scans:
                return img.header
        raise AssertionError('Depth/pose fixture was not acquired: '+log.read_text()[-3000:])
    def score(header,value):
        msg=SemanticScore();msg.header=header;msg.backend='blip2';msg.score=value
        scan=scans[stamp(header)]
        q=scan.camera_pose.orientation
        yaw=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
        f=Frame(stamp(header),None,scan.camera_pose.position.x,scan.camera_pose.position.y,yaw,
                scan.resolution,scan.origin_x,scan.origin_y,scan.max_distance,
                set(zip(scan.free_x,scan.free_y)),(),())
        projected=project_cells(f,list(zip(scan.cell_ids,scan.cell_x,scan.cell_y)))
        assert projected, 'Synthetic frame has no observed frontier support'
        msg.cell_ids=[int(i) for i,w in projected];msg.weights=[float(w) for i,w in projected]
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
    if %r:
        ld.add_action(Node(package='trajectory_manager', executable='traj_server',
            name='traj_server_node', parameters=[%r, {'need_init':False}],
            remappings=[('odometry','/habitat/odom'),('trajectory','/planning/trajectory')]))
    return ld
''' % (str(root/'src/planner/exploration_manager/launch/algorithm_traj.launch.py'),
       continuous, str(root/'src/planner/trajectory_manager/config/control_param.yaml')))
        log=path/'planner.log'
        with log.open('w') as output:
            process=subprocess.Popen(['ros2','launch',str(wrapper),
                'is_real_world_:='+str(continuous).lower()],cwd=root,
                stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline=time.monotonic()+12
                while not score_pub.get_subscription_count() and time.monotonic()<deadline:spin(.1)
                assert score_pub.get_subscription_count(), 'Map did not subscribe: '+log.read_text()[-2000:]
                spin(.5)
                for _ in range(8): original=frame(0.)
                for _ in range(8): latest=frame(8.)
                spin(.3)
                assert frontier_markers, 'No frontier geometry markers (unscored frontiers must also be visible)'
                center_markers=[m for m in frontier_markers[-1].markers if m.ns.endswith('_centers')]
                boundary_markers=[m for m in frontier_markers[-1].markers if m.ns.endswith('_boundaries')]
                assert sum(len(m.points) for m in center_markers)>0, 'Unscored centers hidden'
                assert sum(len(m.points) for m in boundary_markers)>0, 'Unscored boundaries hidden'
                assert all(abs(c.r-c.g)<1e-5 and abs(c.g-c.b)<1e-5 and c.a==1.
                    for m in center_markers for c in m.colors), 'Unscored nodes must be neutral gray'
                score(original,.7)  # Camera already moved to B, return score for A.
                score(original,.9)  # Duplicate cannot overwrite .7.
                score(latest,.2)
                spin(.5)
                text=log.read_text()
                stamp=lambda h: h.stamp.sec*1000000000+h.stamp.nanosec
                a=[line for line in text.splitlines() if f'cells stamp={stamp(original)} ' in line]
                b=[line for line in text.splitlines() if f'cells stamp={stamp(latest)} ' in line]
                assert len(a)==2 and 'score=0.7000' in a[0] and 'updated=0' in a[1], text[-5000:]
                assert len(b)==1 and 'score=0.2000' in b[0], text[-5000:]
                assert all(int(re.search(r'updated=(\d+)',line)[1])>0 for line in (a[0],b[0])), (a,b)
                assert clouds and clouds[-1].width>0, 'No sparse frontier cloud published'
                points=list(point_cloud2.read_points(clouds[-1],field_names=['x','y','intensity']))
                assert any(p[0]<5 and abs(float(p[2])-.7)<1e-5 for p in points), points
                assert any(p[0]>7 and abs(float(p[2])-.2)<1e-5 for p in points), points
                assert len(points)<200, f'Expected frontier-only points, got {len(points)}'
                centers=[(p,c) for m in frontier_markers[-1].markers if m.ns.endswith('_centers')
                         for p,c in zip(m.points,m.colors)]
                boundaries=[(p,c) for m in frontier_markers[-1].markers if m.ns.endswith('_boundaries')
                            for p,c in zip(m.points,m.colors)]
                current_labels=[m for m in labels[-1].markers if m.action==Marker.ADD]
                assert len(centers)==len(current_labels), 'Visible centers must include every current frontier'
                assert len(centers)>=len(points), 'Scored-only cloud cannot outnumber all frontier centers'
                scored_centers=[(p,c) for p,c in centers if c.r>c.g+1e-5]
                assert len(scored_centers)==len(points), 'Scored centers must match scored-only cloud'
                for q in points:
                    assert any(abs(p.x-float(q[0]))<1e-4 and abs(p.y-float(q[1]))<1e-4
                        for p,c in scored_centers), 'A scored frontier center is missing'
                ac=[c for p,c in scored_centers if p.x<5];bc=[c for p,c in scored_centers if p.x>7]
                assert ac and bc and ac[0].g<bc[0].g and ac[0].r<bc[0].r, 'Higher score must be darker'
                assert all(c.a==1. for p,c in centers), 'All current frontier centers must be visible'
                assert all(any(abs(c.r-d.r)<1e-5 and abs(c.g-d.g)<1e-5
                    for q,d in centers) for p,c in boundaries if c.r>c.g+1e-5), 'Scored fixture cells must agree with their cluster max'
                assert any(p.x<5 and abs(c.r-ac[0].r)<1e-5 and abs(c.g-ac[0].g)<1e-5
                    for p,c in boundaries), 'A boundary did not receive A semantic color'
                assert any(p.x>7 and abs(c.r-bc[0].r)<1e-5 and abs(c.g-bc[0].g)<1e-5
                    for p,c in boundaries), 'B boundary did not receive B semantic color'
                print('PASS: delayed BLIP2 A score updates A frontiers once; B score updates B; sparse cloud correct')
                print(a[0]);print(b[0])
                # A new simulator process must reset the still-running planner,
                # even if the previous process never sent EPISODE_FINISH.
                old_odom=Odometry(); old_odom.header=latest
                old_odom.pose.pose.position.x=8.;old_odom.pose.pose.orientation.w=1.
                for _ in range(4):odom_pub.publish(old_odom);spin(.1)
                if continuous:
                    assert any(m.points for m in paths), 'Fixture did not populate traveled path'
                finish_episode(node)
                spin(.6)
                assert clouds[-1].width > 0 and free_clouds[-1].width > 0, 'Episode exit erased the map'
                assert any(m.action == Marker.ADD for m in labels[-1].markers), 'Episode exit erased frontier labels'
                if continuous:
                    assert paths[-1].points, 'Episode exit erased traveled path'
                print('PASS: episode finish retains map, frontier semantics, labels and traveled path')
                reset_episode(node, continuous=continuous)
                spin(.6)
                assert clouds[-1].width==0, 'Previous frontier scores survived reset'
                assert free_clouds and free_clouds[-1].width==0, 'Previous map survived reset'
                assert all(m.action==Marker.DELETEALL for m in labels[-1].markers), 'Old frontier labels survived'
                assert not any(m.points for m in frontier_markers[-1].markers), 'Old frontier geometry survived reset'
                assert any(m.action==Marker.DELETEALL for m in markers), 'RViz frontier markers not cleared'
                if continuous:
                    assert not paths[-1].points, 'Old traveled path survived reset'
                    odom_pub.publish(old_odom);spin(.3)
                    assert not paths[-1].points, 'Queued old odometry repopulated traveled path'
                score(original,.95)  # In-flight old result arrives after reset.
                assert clouds[-1].width==0, 'Late old score recreated frontier nodes'
                for _ in range(8): fresh=frame(20.)
                score(fresh,.4)
                points=list(point_cloud2.read_points(clouds[-1],field_names=['x','y','intensity']))
                assert points and all(p[0]>19 for p in points), 'Old frontier geometry survived'
                assert all(abs(float(p[2])-.4)<1e-5 for p in points), 'Old semantic memory survived'
                print('PASS: reset clears map/frontiers/markers; old scores rejected; fresh scene independent')
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try:process.wait(timeout=5)
                    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                Path('/tmp/apexnav-semantic-alignment.log').write_text(log.read_text())
    node.destroy_node();rclpy.shutdown()


if __name__ == '__main__':
    main()
