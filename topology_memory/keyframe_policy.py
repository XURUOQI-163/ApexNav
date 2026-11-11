"""Bounded view selection; no ROS state, image decoding or additional graph."""
import math


def angle_distance(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def distance_to_node(node, position):
    if position is None or len(position) < 2 or not all(math.isfinite(v) for v in position[:2]):
        return math.inf
    return math.hypot(position[0]-node['x'], position[1]-node['y'])


def junction_capture_allowed(node, robot):
    return (node.get('visit_state') == 'CONFIRMED' and
            distance_to_node(node, robot) <= node.get('visit_radius', .5))


def view_error(node, camera, yaw):
    return angle_distance(yaw, math.atan2(node['y']-camera[1], node['x']-camera[0]))


def select_keyframe(node, acquired, camera, robot, yaw, limit, junction_angle=math.pi/3):
    """Return (save, observation_id_to_replace). Eligibility uses capture poses."""
    old = node.get('observations', [])
    if limit <= 0 or not math.isfinite(yaw) or any(o['stamp_ns'] == acquired for o in old):
        return False, None
    if node['node_type'] == 'JUNCTION':
        if not junction_capture_allowed(node, robot):
            return False, None
        nearby = [o for o in old if angle_distance(yaw, o['camera_yaw']) <= junction_angle+1e-9]
        if not nearby:
            return len(old) < limit, None
        # A view between two saved directions must not replace one and become
        # a duplicate of the other. Keep the existing distinct directions.
        if len(nearby) != 1:
            return False, None
        previous = nearby[0]
        if distance_to_node(node, robot) < distance_to_node(node, previous.get('robot_pose'))-1e-6:
            return True, previous['observation_id']
        return False, None
    if node['node_type'] == 'ENDPOINT':
        if not old:
            return True, None
        previous = old[0]
        new_error = view_error(node, camera, yaw)
        old_error = view_error(node, previous['camera_pose'], previous['camera_yaw'])
        better = new_error < old_error-1e-6
        tied_and_closer = (abs(new_error-old_error) <= 1e-6 and
                          distance_to_node(node, camera) < distance_to_node(node, previous['camera_pose'])-1e-6)
        return (True, previous['observation_id']) if better or tied_and_closer else (False, None)
    raise ValueError('Visit keyframe policy only applies to Junction / Endpoint')
