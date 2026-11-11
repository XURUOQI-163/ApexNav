"""ROS message adapters and bounded keyframe selection, run only on disk worker."""
from collections import OrderedDict
from copy import deepcopy
from types import SimpleNamespace
import math

from basic_utils.frontier_frame_window import Frame, FrameWindow, project_cells
from .keyframe_policy import select_keyframe, junction_capture_allowed, distance_to_node, view_error


def stamp(header):
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def pose(p):
    return [p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]


def xy(point):
    return [point.x, point.y]


def angle_distance(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def skeleton_data(msg):
    types = {0: 'ENDPOINT', 1: 'JUNCTION', 2: 'LOOP_ANCHOR'}
    nodes = [dict(source_id=int(n.node_id), node_type=types[n.node_type],
                  x=n.position.x, y=n.position.y, pixel=[n.pixel_x, n.pixel_y],
                  anchor_pixels=[list(p) for p in zip(n.anchor_pixel_x, n.anchor_pixel_y)],
                  clearance=n.clearance, near_unknown_boundary=n.near_unknown_boundary)
             for n in msg.nodes if n.active]
    by_id = {int(n.node_id): n for n in msg.nodes}
    for node in nodes:
        if node['node_type'] == 'JUNCTION':
            source = by_id[node['source_id']]
            node.update(visit_state='CONFIRMED' if source.visit_state == source.CONFIRMED else 'CANDIDATE',
                        visit_radius=source.visit_radius if source.visit_radius > 0 else .5,
                        confirmed_at_ns=int(source.confirmed_at_ns),
                        visit_position=xy(source.visit_position) if source.confirmed_at_ns > 0 else None)
    edges = [dict(source_id=int(e.edge_id), start_source_id=int(e.start_node_id),
                  end_source_id=int(e.end_node_id), path=[xy(p) for p in e.path],
                  pixel_path=[list(p) for p in zip(e.pixel_x, e.pixel_y)],
                  display_path=[xy(p) for p in e.display_path], length=e.length,
                  min_clearance=e.min_clearance, mean_clearance=None) for e in msg.edges]
    return dict(nodes=nodes, edges=edges,
                map_info=dict(resolution=msg.map_info.resolution, width=msg.map_info.width,
                              height=msg.map_info.height, origin=pose(msg.map_info.origin)))


def frontier_data(state, graph, context, score_metadata, include_cells=True):
    links = {int(link.frontier_id): link for link in graph.frontier_links}
    result = []
    for node in state.nodes:
        winner = max((c for c in node.cells if c.confidence > 0),
                     key=lambda c: c.fused_score, default=None)
        raw_stamp = int(winner.last_frame_stamp_ns) if winner else None
        # Cell geometry is temporary input for opt-in image selection only.
        cells = [dict(token=int(c.token), pixel=[c.pixel_x, c.pixel_y])
                 for c in node.cells] if include_cells else []
        metadata = score_metadata.get(raw_stamp)
        semantic = dict(goal_text=context['goal_text'], model_name=state.backend or context['model_name'],
                        model_version=metadata.model_version or None if metadata else None,
                        raw_similarity=winner.raw_similarity if winner else None,
                        raw_frame_stamp_ns=raw_stamp,
                        computed_at=int(metadata.computed_at_ns) if metadata else None,
                        fused_score=node.fused_score if node.scored_cells else None,
                        confidence=node.confidence, normalized_score=None,
                        aggregation='max_of_fused_frontier_cells', scored_cells=int(node.scored_cells))
        link = links.get(int(node.frontier_id))
        approach, attachment, parent = None, None, None
        # An asynchronous Skeleton job can refer to a previous cluster geometry.
        # Do not attach a moved/recycled Frontier simply because its source ID matches.
        tolerance = max(graph.map_info.resolution * .5, 1e-6)
        if (link and link.connected and link.active == node.active and
                math.hypot(link.frontier_centroid.x-node.centroid.x,
                           link.frontier_centroid.y-node.centroid.y) <= tolerance):
            safe_path = [xy(p) for p in link.display_path]
            support = xy(link.approach)
            # A restarted recorder can see an old latched producer whose display
            # path still included Unknown. Its raster witness stops in FREE.
            if not safe_path or math.dist(safe_path[-1], support) > 1e-6:
                safe_path = [xy(p) for p in link.path]
            if safe_path and math.dist(safe_path[-1], support) <= 1e-6:
                parent = int(link.junction_node_id)
                approach = dict(x=link.approach.x, y=link.approach.y,
                                yaw=math.atan2(node.centroid.y-link.approach.y, node.centroid.x-link.approach.x),
                                source='known_free_attachment_support_not_controller_goal')
                attachment = dict(path=safe_path, path_role='KNOWN_FREE_APPROACH',
                                  raster_path=[xy(p) for p in link.path],
                                  length=sum(math.dist(a, b) for a, b in zip(safe_path, safe_path[1:])),
                                  terminal_may_be_frontier_unknown=False,
                                  association_path=[support, xy(node.centroid)],
                                  association_length=math.dist(support, xy(node.centroid)),
                                  association_traversable=False)
        result.append(dict(source_id=int(node.frontier_id), x=node.centroid.x, y=node.centroid.y,
                           status='ACTIVE' if node.active else 'DORMANT', approach_pose=approach,
                           parent_source_id=parent, semantic=semantic, cells=cells, attachment=attachment))
    return result


class MemoryRecorder:
    def __init__(self, memory, cv_bridge, window_size=20, keyframe_angle_degrees=45.,
                 keyframe_distance=.75, jpeg_quality=90, model_version='', junction_keyframe_angle_degrees=60.):
        self.memory = memory
        self.cv = cv_bridge
        self.capacity = int(window_size)
        self.angle = math.radians(keyframe_angle_degrees)
        self.junction_angle = math.radians(junction_keyframe_angle_degrees)
        self.distance = keyframe_distance
        self.quality = int(jpeg_quality)
        self.model_version = model_version
        self.window = FrameWindow(self.capacity)
        self.scored = OrderedDict()
        self.session = None

    def process(self, context, graph, state, frames, observations):
        # All timestamps are source acquisition times; no nearest-time fallback.
        if not context or state is None or graph is None:
            return False
        cutoff = int(context['started_at_ns'])
        if (state.last_depth_stamp_ns <= cutoff or stamp(graph.header) <= cutoff or
                stamp(graph.header) <= state.map_epoch_ns or
                graph.header.frame_id != context['frame_id'] or state.header.frame_id != context['frame_id']):
            return False
        source_context = dict(context, simulator_session_id=context['session_id'], map_epoch_ns=int(state.map_epoch_ns),
                              session_id=context['session_id'] + '_' + str(state.map_epoch_ns))
        if source_context['session_id'] != self.session:
            self.memory.begin_session(source_context)
            self.session = source_context['session_id']
            self.window = FrameWindow(self.capacity)
            self.scored.clear()
        cutoff = max(cutoff, int(state.map_epoch_ns))
        for rgb, scan, odom in frames:
            acquired = stamp(rgb.header)
            if (acquired <= cutoff or acquired in self.window.frames or
                    acquired != stamp(scan.header) or acquired != stamp(odom.header) or
                    scan.header.frame_id != context['frame_id'] or odom.header.frame_id != context['frame_id'] or scan.resolution <= 0 or
                    len(scan.free_x) != len(scan.free_y)):
                continue
            q = scan.camera_pose.orientation
            yaw = math.atan2(2*(q.y*q.z-q.w*q.x), 2*(q.x*q.z+q.w*q.y))
            # Image stays as a ROS message reference until selected for saving.
            self.window.add(Frame(acquired, rgb, scan.camera_pose.position.x, scan.camera_pose.position.y,
                                  yaw, scan.resolution, scan.origin_x, scan.origin_y, scan.max_distance,
                                  set(zip(scan.free_x, scan.free_y)), pose(scan.camera_pose), pose(odom.pose.pose)))
        for observation in observations:
            acquired = stamp(observation.header)
            if (acquired <= cutoff or observation.header.frame_id != context['frame_id'] or
                    observation.goal_text != context['goal_text'] or
                    observation.model_name != context['model_name'] or
                    stamp(observation.image.header) != acquired or
                    len(observation.cell_ids) != len(observation.weights)):
                continue
            if not self.memory.save_frontier_images:
                # Keep score provenance without retaining its RGB payload.
                self.scored[acquired] = SimpleNamespace(model_version=observation.model_version,
                                                       computed_at_ns=observation.computed_at_ns)
                self.scored.move_to_end(acquired)
                continue
            if acquired in self.scored:
                previous = self.scored[acquired]
                evidence = dict(zip(previous.cell_ids, previous.weights))
                evidence.update(zip(observation.cell_ids, observation.weights))
                observation = deepcopy(observation)
                observation.cell_ids = sorted(evidence)
                observation.weights = [evidence[token] for token in observation.cell_ids]
            self.scored[acquired] = observation
            self.scored.move_to_end(acquired)
        while len(self.scored) > self.capacity:
            self.scored.popitem(last=False)
        self.memory.save_skeleton_graph(skeleton_data(graph))
        current = frontier_data(state, graph, context, self.scored, self.memory.save_frontier_images)
        if self.model_version:
            for node in current:
                node['semantic']['model_version'] = self.model_version
        self.memory.save_frontier_state(current)
        self._select_observations({n['source_id']: n['cells'] for n in current})
        self.memory.flush()
        return True

    def _distinct(self, node, acquired, camera, yaw):
        existing = node.get('observations', [])
        if any(o['stamp_ns'] == acquired for o in existing):
            return False
        if len(existing) >= self.memory.observation_limit(node):
            return False
        # Require meaningful new viewpoint/heading relative to EVERY saved view.
        return all(angle_distance(yaw, o['camera_yaw']) >= self.angle or
                   math.hypot(camera[0]-o['camera_pose'][0], camera[1]-o['camera_pose'][1]) >= self.distance
                   for o in existing)

    def _save(self, node, acquired, image, camera, robot, yaw, **extra):
        if self.memory.observation_limit(node) == 0:
            return
        evidence = extra.get('evidence')
        if evidence and self.memory.enrich_observation(node['node_id'], acquired, evidence):
            return
        replace_id = None
        if node['node_type'] in ('JUNCTION', 'ENDPOINT'):
            accepted, replace_id = select_keyframe(node, acquired, camera, robot, yaw,
                                                  self.memory.observation_limit(node), self.junction_angle)
            if not accepted:
                return
            distinct = True
            extra.update(capture_policy='junction_visit_v1' if node['node_type'] == 'JUNCTION' else 'deadend_alignment_v1',
                         node_distance=distance_to_node(node, robot if node['node_type'] == 'JUNCTION' else camera),
                         view_angle_error=view_error(node, camera, yaw))
        else:
            distinct = self._distinct(node, acquired, camera, yaw)
        if not distinct and evidence:
            # Upgrade an unscored representative at the same viewpoint, keeping
            # no stale reference; the old JPEG is collected after the next flush.
            equivalent = [o for o in node['observations'] if not o.get('evidence') and
                          angle_distance(yaw, o['camera_yaw']) < self.angle and
                          math.hypot(camera[0]-o['camera_pose'][0], camera[1]-o['camera_pose'][1]) < self.distance]
            if equivalent:
                replace_id = equivalent[0]['observation_id']
        if not distinct and replace_id is None:
            return
        import cv2
        rgb = self.cv.imgmsg_to_cv2(image, desired_encoding='bgr8')
        ok, data = cv2.imencode('.jpg', rgb, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            raise ValueError('JPEG encoding failed')
        self.memory.save_observation(node['node_id'], data.tobytes(),
                                     dict(stamp_ns=acquired, camera_pose=list(camera), robot_pose=list(robot),
                                          camera_yaw=yaw, bearing_to_node=math.atan2(node['y']-camera[1], node['x']-camera[0]),
                                          **extra), replace_id=replace_id)

    def _select_observations(self, frontier_cells):
        # Prefer actual scored originals, including pinned frames evicted from live window.
        for node in self.memory.frontiers if self.memory.save_frontier_images else []:
            members = frontier_cells.get(node['source_id'], [])
            tokens = {c['token'] for c in members}
            for acquired, observation in reversed(self.scored.items()):
                supported = [(int(t), float(w)) for t, w in zip(observation.cell_ids, observation.weights) if t in tokens]
                if supported:
                    self._save(node, acquired, observation.image, pose(observation.camera_pose),
                               pose(observation.robot_pose), observation.camera_yaw,
                               evidence=dict(goal_text=observation.goal_text, model_name=observation.model_name,
                                             model_version=observation.model_version or self.model_version or None,
                                             raw_similarity=observation.raw_similarity,
                                             computed_at=int(observation.computed_at_ns),
                                             projected_cells=[dict(token=t, weight=w) for t, w in supported]))
            # An unscored geometry view is still useful; it is explicitly unscored.
            cells = [(c['token'], *c['pixel']) for c in members]
            for frame in reversed(self.window.snapshot()):
                if project_cells(frame, cells):
                    self._save(node, frame.stamp, frame.image, frame.camera_pose, frame.robot_pose,
                               frame.camera_yaw, evidence=None)
        for node in self.memory.skeleton['nodes']:
            if node['status'] != 'ACTIVE':
                continue
            for frame in reversed(self.window.snapshot()):
                if node['node_type'] == 'JUNCTION':
                    # Capture the surroundings while the robot is AT this
                    # junction, including headings looking away from its centre.
                    if junction_capture_allowed(node, frame.robot_pose):
                        self._save(node, frame.stamp, frame.image, frame.camera_pose, frame.robot_pose, frame.camera_yaw)
                    continue
                ix = math.floor((node['x']-frame.origin_x)/frame.resolution)
                iy = math.floor((node['y']-frame.origin_y)/frame.resolution)
                dx, dy = node['x']-frame.camera_x, node['y']-frame.camera_y
                # Free cells are original ray support. Do not infer visibility
                # from today's global map or persist frames looking behind camera.
                if ((ix, iy) in frame.free_cells and math.hypot(dx, dy) <= frame.max_distance and
                        angle_distance(math.atan2(dy, dx), frame.camera_yaw) < math.radians(79/2)):
                    self._save(node, frame.stamp, frame.image, frame.camera_pose, frame.robot_pose,
                               frame.camera_yaw)  # Junction has NO task semantic attribute.
