"""Independent ROS collector and bounded image-window semantic inference worker."""
import math
import threading
from copy import deepcopy

import message_filters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from plan_env.msg import FrontierScan, SemanticScore, TopologyObservation
from basic_utils.frontier_frame_window import Frame, FrameWindow, evaluate_frame
from vlm.utils.get_semantic_similarity import get_semantic_similarity


def pose_tuple(p):
    return (p.position.x,p.position.y,p.position.z,p.orientation.x,
            p.orientation.y,p.orientation.z,p.orientation.w)

class FrontierSemanticBridge:
    """A slow HTTP inference never blocks sensor collection or robot control.

    The live window has exactly 20 frames. One running batch pins at most another
    20 references while inference completes. There is no growing task queue.
    """
    def __init__(self, backend='clip', inference=get_semantic_similarity):
        self.node=Node('frontier_semantic_window')
        self.backend=backend
        self.inference=inference
        self.cv=CvBridge()
        self.condition=threading.Condition()
        self.window=FrameWindow(20)
        self.cells=[]
        self.target=None
        self.generation=0
        self.revision=0
        self.latest_scan=0
        self.epoch=0
        self.closed=False
        self.pub=self.node.create_publisher(SemanticScore,'/semantic/score',20)
        self.memory_pub=self.node.create_publisher(TopologyObservation,'/topology_memory/scored_observation',2)
        self.rgb_sub=message_filters.Subscriber(self.node,Image,'/habitat/camera_rgb',qos_profile=qos_profile_sensor_data)
        self.scan_sub=message_filters.Subscriber(self.node,FrontierScan,'/frontier_semantics/scan',qos_profile=20)
        self.odom_sub=message_filters.Subscriber(self.node,Odometry,'/habitat/odom',qos_profile=20)
        self.scan_sub.registerCallback(self._current_cells)
        self.sync=message_filters.TimeSynchronizer([self.rgb_sub,self.scan_sub,self.odom_sub],30)
        self.sync.registerCallback(self._frame)
        self.executor=SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.collector=threading.Thread(target=self._collect,name='semantic-frame-collector',daemon=True)
        self.worker=threading.Thread(target=self._work,name='semantic-window-inference',daemon=True)
        self.collector.start();self.worker.start()

    def set_target(self,label,room,force=False):
        with self.condition:
            if not force and self.target==(label,room):return
            self.target=(label,room)
            self.generation+=1
            self.epoch=self.node.get_clock().now().nanoseconds
            self.latest_scan=0
            self.window=FrameWindow(20)
            self.cells=[]
            self.revision+=1
            self.condition.notify_all()

    @staticmethod
    def stamp(header):return header.stamp.sec*1_000_000_000+header.stamp.nanosec

    def _current_cells(self,scan):
        stamp=self.stamp(scan.header)
        if not (len(scan.cell_ids)==len(scan.cell_x)==len(scan.cell_y)):return
        with self.condition:
            if stamp<=self.epoch or stamp<self.latest_scan:return
            self.latest_scan=stamp
            cells=list(zip(scan.cell_ids,scan.cell_x,scan.cell_y))
            if cells!=self.cells:
                self.cells=cells
                self.revision+=1;self.condition.notify_all()

    def _frame(self,rgb,scan,odom):
        stamp=self.stamp(scan.header)
        with self.condition:
            if self.closed or stamp<=self.epoch or stamp in self.window.frames:return
            generation=self.generation
        q=scan.camera_pose.orientation
        yaw=math.atan2(2*(q.y*q.z-q.w*q.x),2*(q.x*q.z+q.w*q.y))
        if len(scan.free_x)!=len(scan.free_y) or scan.resolution<=0:return
        frame=Frame(stamp,self.cv.imgmsg_to_cv2(rgb,'rgb8').copy(),
                    scan.camera_pose.position.x,scan.camera_pose.position.y,yaw,
                    scan.resolution,scan.origin_x,scan.origin_y,scan.max_distance,
                    set(zip(scan.free_x,scan.free_y)),pose_tuple(scan.camera_pose),
                    pose_tuple(odom.pose.pose),deepcopy(rgb.header))
        with self.condition:
            if self.closed or generation!=self.generation:return
            if self.window.add(frame):
                self.revision+=1;self.condition.notify_all()

    def _collect(self):
        try:
            while rclpy.ok() and not self.closed:self.executor.spin_once(timeout_sec=.1)
        except rclpy.executors.ExternalShutdownException:
            pass

    def _work(self):
        processed=-1
        while True:
            with self.condition:
                self.condition.wait_for(lambda:self.closed or
                    (self.target is not None and self.revision!=processed))
                if self.closed:return
                processed=self.revision
                generation=self.generation
                target=self.target
                batch=self.window.snapshot()  # oldest to newest, one bounded batch
            for frame in batch:
                with self.condition:
                    if self.closed:return
                    if generation!=self.generation:break
                    cells=list(self.cells)
                if frame.failures>=2:continue
                try:
                    was_cached=frame.score is not None
                    result=evaluate_frame(frame,cells,
                        lambda image:self.inference(image,*target,self.backend))
                    if result is None:continue
                    score,projected=result
                    if not was_cached:
                        frame.memory_computed_at_ns=self.node.get_clock().now().nanoseconds
                    with self.condition:
                        if self.closed:return
                        if generation!=self.generation:break
                        live={c[0] for c in self.cells}
                        projected=[(i,w) for i,w in projected if i in live]
                        # Frontiers can disappear during HTTP inference. Keep
                        # the cached score, but publish/log only live evidence.
                        if not projected:continue
                        msg=SemanticScore()
                        msg.header=frame.header;msg.backend=self.backend;msg.score=score
                        msg.cell_ids=[int(i) for i,w in projected]
                        msg.weights=[float(w) for i,w in projected]
                        self.pub.publish(msg)
                        frame.sent_ids.update(msg.cell_ids)
                    # Optional read-only tap. Keep the pinned ORIGINAL frame even
                    # when slow inference has outlived the live 20-frame window.
                    # Memory failures must not change inference retry/fusion state.
                    if self.memory_pub.get_subscription_count():
                        try:
                            observation=TopologyObservation()
                            observation.header=frame.header
                            observation.image=self.cv.cv2_to_imgmsg(frame.image,encoding='rgb8')
                            observation.image.header=frame.header
                            for pose, values in ((observation.camera_pose,frame.camera_pose),
                                                 (observation.robot_pose,frame.robot_pose)):
                                pose.position.x,pose.position.y,pose.position.z=values[:3]
                                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=values[3:]
                            observation.camera_yaw=frame.camera_yaw
                            observation.goal_text=target[0];observation.model_name=self.backend
                            observation.raw_similarity=score
                            observation.computed_at_ns=getattr(frame,'memory_computed_at_ns',0)
                            observation.cell_ids=msg.cell_ids;observation.weights=msg.weights
                            self.memory_pub.publish(observation)
                        except Exception as memory_error:
                            self.node.get_logger().warning(f'UTM observation skipped: {memory_error}')
                    self.node.get_logger().info(
                        f'semantic[{self.backend}] frame={frame.stamp} raw={score:.4f} '
                        f'cells={len(projected)} cached={was_cached}')
                except Exception as exc:
                    frame.failures+=1
                    with self.condition:
                        if self.closed:return
                        self.node.get_logger().warning(f'Semantic frame {frame.stamp} failed ({frame.failures}/2): {exc}')

    def close(self):
        with self.condition:
            if self.closed:return
            self.closed=True;self.condition.notify_all()
        self.collector.join(timeout=2.)
        self.worker.join(timeout=2.)
        # An in-flight daemon HTTP call checks closed before publishing anything.
        self.executor.shutdown(timeout_sec=1.)
        self.node.destroy_node()
