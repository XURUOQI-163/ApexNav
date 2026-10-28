"""Opt-in ROS2 window regression, isolated domain 94; no model downloads.

Uses the real collector/worker with a controlled inference function. Tests slow
inference, exact synchronization, bounded original frames and context resets.
"""
import threading
import time
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from plan_env.msg import FrontierScan, SemanticScore
from tf_transformations import quaternion_from_euler
from habitat2ros.frontier_semantics import FrontierSemanticBridge


def main():
    rclpy.init(domain_id=94)
    receiver=rclpy.create_node('semantic_publisher_probe')
    received=[]
    receiver.create_subscription(SemanticScore,'/semantic/score',received.append,100)
    rgb_pub=receiver.create_publisher(Image,'/habitat/camera_rgb',30)
    scan_pub=receiver.create_publisher(FrontierScan,'/frontier_semantics/scan',30)
    odom_pub=receiver.create_publisher(Odometry,'/habitat/odom',30)
    cv=CvBridge()
    def spin(seconds=.05):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(receiver,timeout_sec=.01)
    def until(predicate,timeout=5):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:spin()
        assert predicate(), 'Timed out waiting for ROS window evidence'
    def stamp(h):return h.stamp.sec*1_000_000_000+h.stamp.nanosec
    def frame(value,cells=True):
        rgb=cv.cv2_to_imgmsg(np.full((4,4,3),value,np.uint8),'rgb8')
        rgb.header.stamp=receiver.get_clock().now().to_msg();rgb.header.frame_id='world'
        odom=Odometry();odom.header=rgb.header;odom.pose.pose.orientation.w=1.
        scan=FrontierScan();scan.header=rgb.header;scan.resolution=1.
        scan.origin_x=-.5;scan.origin_y=-.5;scan.max_distance=5.
        q=quaternion_from_euler(np.pi/2,np.pi,np.pi/2)
        scan.camera_pose.orientation.x,scan.camera_pose.orientation.y,scan.camera_pose.orientation.z,scan.camera_pose.orientation.w=map(float,q)
        scan.free_x=[1];scan.free_y=[0]
        if cells:scan.cell_ids=[1];scan.cell_x=[2];scan.cell_y=[0]
        return rgb,scan,odom
    def publish(messages):
        rgb,scan,odom=messages
        rgb_pub.publish(rgb);scan_pub.publish(scan);odom_pub.publish(odom)
    try:
        for backend in ['clip','blip2']:
            entered=threading.Event();release=threading.Event();calls=[]
            def delayed(image,label,room,selected):
                assert selected==backend
                value=int(image[0,0,0]);calls.append(value)
                if value in (1,90):
                    entered.set()
                    assert release.wait(5), 'Test did not release inference'
                return value/100.
            bridge=FrontierSemanticBridge(backend,inference=delayed)
            try:
                bridge.set_target('chair','room')
                until(lambda:rgb_pub.get_subscription_count() and scan_pub.get_subscription_count()
                      and odom_pub.get_subscription_count() and bridge.pub.get_subscription_count())
                # Mismatched odometry must never produce a cached RGB frame.
                original=frame(1);rgb,scan,odom=original
                rgb_pub.publish(rgb);scan_pub.publish(scan)
                wrong=Odometry();wrong.header.stamp=receiver.get_clock().now().to_msg()
                odom_pub.publish(wrong);spin(.15)
                assert not entered.is_set(), 'Mismatched timestamp accepted'
                odom_pub.publish(odom);until(entered.is_set)
                # Collector must keep receiving while HTTP inference is blocked.
                headers=[]
                for value in range(2,26):
                    messages=frame(value);publish(messages);headers.append(messages[0].header)
                    until(lambda:stamp(messages[0].header) in bridge.window.frames)
                with bridge.condition:
                    assert len(bridge.window.frames)==20
                    assert list(bridge.window.frames)==[stamp(h) for h in headers[-20:]]
                    assert bridge.window.snapshot()[-1].robot_pose==(0.,0.,0.,0.,0.,0.,1.)
                release.set()
                until(lambda:any(m.header==original[0].header and m.backend==backend for m in received))
                first=next(m for m in received if m.header==original[0].header and m.backend==backend)
                assert first.score==.01 and list(first.cell_ids)==[1]
                assert abs(first.weights[0]-1/(1+(2/5)**2))<1e-6
                until(lambda:any(m.header==headers[-1] and m.backend==backend for m in received))
                # New cell lifetime reuses cached RGB scores, no second inference.
                before=len(calls)
                update=frame(30)[1];update.cell_ids=[2];scan_pub.publish(update)
                until(lambda:any(list(m.cell_ids)==[2] and m.backend==backend for m in received))
                assert len(calls)==before, 'Cached RGB inferred twice'
                # A context reset must discard a result already in flight.
                entered.clear();release.clear()
                stale=frame(90);publish(stale);until(entered.is_set)
                bridge.set_target('table','kitchen',force=True)
                release.set();spin(.2)
                assert not any(m.header==stale[0].header for m in received), 'Old target score escaped reset'
                fresh=frame(80,cells=False);publish(fresh)
                until(lambda:stamp(fresh[0].header) in bridge.window.frames)
                spin(.2)
                assert 80 not in calls, 'Image without frontiers was sent to model'
                assert not any(m.header==fresh[0].header for m in received), 'Empty frontier score published'
                # The saved original image becomes useful only after a frontier
                # appears in its measured scan support, without publishing RGB again.
                update=frame(30)[1];update.cell_ids=[3];scan_pub.publish(update)
                until(lambda:any(m.header==fresh[0].header for m in received))
                result=next(m for m in received if m.header==fresh[0].header)
                assert result.score==.8 and list(result.cell_ids)==[3]
                assert calls.count(80)==1
                # Having a frontier somewhere in the map is insufficient.
                bridge.set_target('table','kitchen',force=True)
                unseen=frame(70);unseen[1].free_x=[];unseen[1].free_y=[]
                publish(unseen);until(lambda:stamp(unseen[0].header) in bridge.window.frames)
                spin(.2)
                assert 70 not in calls, 'Unrelated image was sent to model'
                # A frontier may disappear while inference is running: keep the
                # computed cache, but never emit a cells=0 update/log.
                bridge.set_target('table','kitchen',force=True)
                entered.clear();release.clear()
                disappearing=frame(90);publish(disappearing);until(entered.is_set)
                scan_pub.publish(frame(30,cells=False)[1]);until(lambda:not bridge.cells)
                release.set();spin(.2)
                assert not any(m.header==disappearing[0].header for m in received)
                before=len(calls)
                update=frame(30)[1];update.cell_ids=[4];scan_pub.publish(update)
                until(lambda:any(m.header==disappearing[0].header for m in received))
                assert len(calls)==before, 'In-flight result was not cached for a later frontier'
                assert all(m.cell_ids for m in received), 'A cells=0 update escaped filtering'
                print(f'PASS: {backend}: exact stamp, live 20-frame window during blocked inference, '
                      'cached reuse, original cell weights, context reset, frontier-only inference/publication')
            finally:
                release.set();bridge.close()
    finally:receiver.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
