"""Publish identity metadata only; never reset maps or change navigation."""
import json
import uuid

from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String


def publish_topology_context(node, episode, goal_text, model_name):
    if not hasattr(node, '_topology_context_pub'):
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node._topology_context_pub = node.create_publisher(String, '/topology_memory/context', qos)
    context = dict(session_id=uuid.uuid4().hex, scene_id=str(episode.scene_id),
                   episode_id=str(episode.episode_id),
                   start_position=[float(v) for v in episode.start_position],
                   start_rotation=[float(v) for v in episode.start_rotation],
                   coordinate_convention='habitat_episodic_gps_ros_x=-z_y=-x_z=y_v1',
                   frame_id='world', started_at_ns=node.get_clock().now().nanoseconds,
                   goal_text=str(goal_text), model_name=str(model_name))
    node._topology_context_pub.publish(String(data=json.dumps(context)))
