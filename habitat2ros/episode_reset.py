"""Acknowledged episode boundary; no sensor/vision publishing until this returns."""
import time

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import Empty, Int32
from std_srvs.srv import Trigger
from params import HABITAT_STATE


def finish_episode(node):
    """Stop planning/control while retaining the final map for inspection."""
    node.state_pub.publish(Int32(data=HABITAT_STATE.EPISODE_FINISH))
    publishers = [node.state_pub]
    if hasattr(node, 'stop_pub'):
        node.stop_pub.publish(Empty())
        publishers.append(node.stop_pub)
    # No reset on exit. Let reliable finish/stop messages reach the planner
    # before destroying the simulator's publishers.
    for publisher in publishers:
        if not publisher.wait_for_all_acked(timeout=Duration(seconds=1.)):
            node.get_logger().warning('Episode stop acknowledgement timed out')


def reset_episode(node, *, continuous=False, timeout=10.0):
    # Stop/reconstruct the planner first so it cannot enqueue another old
    # trajectory while the controller is clearing its command and path history.
    services = ['/exploration/reset']
    if continuous:
        services.append('/traj_server/reset')
    for name in services:
        client = node.create_client(Trigger, name)
        try:
            deadline = time.monotonic() + timeout
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    f'{name} unavailable: rebuild and restart the matching planner '
                    'in the same ROS_DOMAIN_ID. Simulation has not been started.')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=max(0., deadline - time.monotonic()))
            if not future.done():
                future.cancel()
                raise RuntimeError(f'{name} reset timed out; refusing to reuse episode history')
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(f'{name} reset failed: {response}')
        finally:
            node.destroy_client(client)
    node.get_logger().info('[Episode] Previous map, frontier semantics and path cleared')
