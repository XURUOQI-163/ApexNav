"""ROS-independent UTM store. Call mutations/flush from one background worker.

Each JSON replacement is atomic. Layers are authoritative; the combined snapshot
is derived. A previous valid copy permits recovery; cross-file references are
validated on load (this is deliberately not a multi-file database transaction).
No method writes back into the navigation map or its semantic fusion state.
"""
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile
import threading
import uuid

SCHEMA_VERSION = 1


def encoded(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2) + '\n'


def atomic_bytes(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, name = tempfile.mkstemp(prefix='.' + path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'wb') as stream:
            stream.write(content)
            stream.flush()
            # mkstemp starts at 0600. Publish root-owned container outputs as
            # host-readable files before rename (also covers JSON and backups).
            os.fchmod(stream.fileno(), 0o644)
            os.fsync(stream.fileno())
        os.replace(name, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def atomic_json(path, value):
    payload = encoded(value).encode('utf-8')
    if path.exists():
        previous = path.read_bytes()
        if previous == payload:
            return
        # Never replace a good backup with a corrupt current file.
        try:
            json.loads(previous)
        except (ValueError, UnicodeError):
            pass
        else:
            atomic_bytes(path.with_suffix(path.suffix + '.bak'), previous)
    atomic_bytes(path, payload)


def identity(context):
    keys = ('scene_id', 'start_position', 'start_rotation', 'frame_id', 'coordinate_convention')
    if any(key not in context for key in keys):
        raise ValueError('UTM context requires scene, start pose and coordinate convention')
    return hashlib.sha256(encoded({key: context[key] for key in keys}).encode()).hexdigest()[:24]


def valid_id(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_-]+', value) is not None


class UnifiedTopologyMemory:
    def __init__(self, directory, node_match_distance=.5, max_observations=8):
        self.root = Path(directory).expanduser().resolve()
        self.node_match_distance = float(node_match_distance)
        self.max_observations = int(max_observations)
        if self.node_match_distance < 0 or self.max_observations < 1:
            raise ValueError('Invalid memory matching/observation limits')
        self.lock = threading.RLock()
        self.manifest = {}
        self.skeleton = dict(nodes=[], edges=[], map_info={})
        self.frontiers = []
        self.attachments = []
        self._dirty = False
        self._node_sources = {}
        self._edge_sources = {}
        self.issues = []

    def _document(self, **payload):
        return dict(schema_version=SCHEMA_VERSION, map_key=self.manifest.get('map_key'),
                    frame_id=self.manifest.get('frame_id'), **payload)

    def begin_session(self, context):
        """Recover matching coordinates; archive a different scene/origin intact.

        Persisted geometry is UNCONFIRMED in a new simulator run until matched.
        Old Frontier task scores never become current simply because IDs repeat.
        """
        context = deepcopy(context)
        key = identity(context)
        if not valid_id(context.get('session_id')):
            raise ValueError('Invalid session_id')
        with self.lock:
            if self.manifest.get('session_id') == context['session_id']:
                if self.manifest.get('map_key') != key or self.manifest.get('goal_text') != context.get('goal_text'):
                    raise ValueError('A changed coordinate origin or goal requires a new session_id')
                return
            if self.manifest and self.manifest.get('map_key') != key:
                self.flush()
                archive = self.root / 'archives' / (self.manifest['map_key'] + '-' + uuid.uuid4().hex[:12])
                archive.mkdir(parents=True)
                for name in ('persistent', 'dynamic', 'observations', 'snapshots', 'manifest.json', 'manifest.json.bak'):
                    if (self.root / name).exists():
                        shutil.move(str(self.root / name), str(archive / name))
                self.skeleton = dict(nodes=[], edges=[], map_info={})
                self.frontiers, self.attachments = [], []
            self.frontiers, self.attachments = [], []
            for item in self.skeleton['nodes'] + self.skeleton['edges']:
                item['status'] = 'UNCONFIRMED'
            self._node_sources, self._edge_sources = {}, {}
            self.manifest = dict(context, schema_version=SCHEMA_VERSION, map_key=key,
                                 goal_id=hashlib.sha256(str(context.get('goal_text', '')).encode()).hexdigest()[:16],
                                 paths=dict(skeleton='persistent/skeleton_graph.json',
                                            frontiers='dynamic/frontier_state.json',
                                            snapshot='snapshots/unified_graph_latest.json'))
            self._dirty = True

    def save_skeleton_graph(self, graph):
        """Adapt current source IDs to stable UTM IDs; preserve exact edge paths."""
        with self.lock:
            old = {n['node_id']: n for n in self.skeleton['nodes']}
            used, current = set(), []
            for source in sorted(graph['nodes'], key=lambda n: n['source_id']):
                src = int(source['source_id'])
                nid = self._node_sources.get(src)
                if nid not in old or nid in used:
                    candidates = [(math.hypot(n['x']-source['x'], n['y']-source['y']), i)
                                  for i, n in old.items() if i not in used and n['node_type'] == source['node_type']]
                    distance, nid = min(candidates, default=(math.inf, None))
                    if distance > self.node_match_distance:
                        nid = None
                nid = nid or 'J_' + uuid.uuid4().hex[:16]
                self._node_sources[src] = nid
                used.add(nid)
                node = dict(source, node_id=nid, layer='PERSISTENT', status='ACTIVE',
                            source_session=self.manifest['session_id'],
                            observations=deepcopy(old.get(nid, {}).get('observations', [])))
                current.append(node)
            for nid, node in old.items():
                if nid not in used:
                    node = deepcopy(node)
                    if node.get('source_session') == self.manifest['session_id']:
                        node['status'] = 'RETIRED'
                    current.append(node)
            previous_edges = {e['edge_id']: e for e in self.skeleton['edges']}
            edges, seen = [], set()
            for source in graph['edges']:
                start = self._node_sources.get(int(source['start_source_id']))
                end = self._node_sources.get(int(source['end_source_id']))
                if start not in used or end not in used:
                    continue
                eid = self._edge_sources.get(int(source['source_id']))
                if eid not in previous_edges:
                    # Parallel paths and loops must not collapse to one edge.
                    eid = next((i for i, e in previous_edges.items() if i not in seen and
                                e['start_node_id'] == start and e['end_node_id'] == end and
                                e['path'] == source['path']), None)
                eid = eid or 'E_' + uuid.uuid4().hex[:16]
                self._edge_sources[int(source['source_id'])] = eid
                seen.add(eid)
                edges.append(dict(source, edge_id=eid, edge_type='SKELETON', start_node_id=start,
                                  end_node_id=end, status='ACTIVE', source_session=self.manifest['session_id'],
                                  # Merged logical centres can differ from actual path endpoints.
                                  traversability='SKELETON_PATH_ONLY_REQUIRES_LOCAL_VALIDATION'))
            for eid, edge in previous_edges.items():
                if eid not in seen:
                    edge = deepcopy(edge)
                    if edge.get('source_session') == self.manifest['session_id']:
                        edge['status'] = 'RETIRED'
                    edges.append(edge)
            result = dict(nodes=sorted(current, key=lambda n: n['node_id']),
                          edges=sorted(edges, key=lambda e: e['edge_id']), map_info=graph.get('map_info', {}))
            if result != self.skeleton:
                self.skeleton = result
                self._dirty = True

    def save_frontier_state(self, frontiers):
        with self.lock:
            previous = {n['node_id']: n for n in self.frontiers}
            current, attachments = [], []
            active_junctions = {n['node_id'] for n in self.skeleton['nodes']
                                if n['status'] == 'ACTIVE' and n['node_type'] == 'JUNCTION'}
            for source in sorted(frontiers, key=lambda n: n['source_id']):
                source = deepcopy(source)
                nid = 'F_' + self.manifest['session_id'] + '_' + str(source['source_id'])
                parent = self._node_sources.get(source.pop('parent_source_id', None))
                parent = parent if parent in active_junctions else None
                attachment = source.pop('attachment', None)
                node = dict(source, node_id=nid, node_type='FRONTIER', layer='TRANSIENT',
                            parent_junction_id=parent, goal_id=self.manifest['goal_id'],
                            observations=deepcopy(previous.get(nid, {}).get('observations', [])))
                semantic = node.setdefault('semantic', {})
                semantic['goal_text'] = self.manifest.get('goal_text', '')
                semantic.setdefault('model_version', None)
                semantic.setdefault('computed_at', None)
                old_semantic = previous.get(nid, {}).get('semantic', {})
                if (semantic.get('raw_frame_stamp_ns') is not None and
                        semantic.get('raw_frame_stamp_ns') == old_semantic.get('raw_frame_stamp_ns') and
                        semantic.get('model_name') == old_semantic.get('model_name')):
                    for key in ('computed_at', 'model_version'):
                        if semantic.get(key) is None:
                            semantic[key] = old_semantic.get(key)
                if attachment and parent:
                    attachments.append(dict(attachment, edge_id='A_' + nid, edge_type='FRONTIER_ATTACHMENT',
                                            start_node_id=parent, end_node_id=nid, status=node['status'],
                                            traversability='ATTACHMENT_ONLY_NOT_SKELETON_EDGE'))
                current.append(node)
            if current != self.frontiers or attachments != self.attachments:
                self.frontiers, self.attachments = current, attachments
                self._dirty = True

    def _find_node(self, node_id):
        return next((n for n in self.skeleton['nodes'] + self.frontiers if n['node_id'] == node_id), None)

    def save_observation(self, node_id, image_bytes, metadata, replace_id=None):
        """Save a selected JPEG before publishing its reference; never overwrite images."""
        with self.lock:
            node = self._find_node(node_id)
            if node is None or not valid_id(node_id):
                raise ValueError('Observation refers to a missing node')
            replacing = any(o['observation_id'] == replace_id for o in node['observations'])
            if len(node['observations']) - int(replacing) >= self.max_observations:
                return None
            if any(o['stamp_ns'] == metadata['stamp_ns'] for o in node['observations']):
                return None
            oid = 'O_' + uuid.uuid4().hex
            kind = 'frontier' if node['node_type'] == 'FRONTIER' else 'junction'
            relative = Path('observations') / kind / node_id / (oid + '.jpg')
            atomic_bytes(self.root / relative, image_bytes)
            observation = dict(deepcopy(metadata), observation_id=oid, image_path=relative.as_posix(),
                               frame_id=self.manifest['frame_id'], session_id=self.manifest['session_id'])
            if replacing:
                node['observations'] = [o for o in node['observations'] if o['observation_id'] != replace_id]
            node['observations'].append(observation)
            self._dirty = True
            return deepcopy(observation)

    def enrich_observation(self, node_id, stamp_ns, evidence):
        """Attach late inference to exactly the original stored frame, no JPEG rewrite."""
        with self.lock:
            node = self._find_node(node_id)
            if not node or node['node_type'] != 'FRONTIER':
                return False
            observation = next((o for o in node['observations'] if o['stamp_ns'] == stamp_ns), None)
            if observation is None:
                return False
            previous = observation.get('evidence') or {}
            combined = deepcopy(evidence)
            cells = {c['token']: c for c in previous.get('projected_cells', [])}
            cells.update({c['token']: c for c in evidence.get('projected_cells', [])})
            combined['projected_cells'] = [cells[key] for key in sorted(cells)]
            if combined != observation.get('evidence'):
                observation['evidence'] = combined
                self._dirty = True
            return True

    def flush(self):
        with self.lock:
            if not self._dirty or not self.manifest:
                return
            for directory in ('persistent', 'dynamic', 'observations/junction', 'observations/frontier', 'snapshots'):
                (self.root / directory).mkdir(parents=True, exist_ok=True)
            # First write (including a newly archived scene) must establish the
            # coordinate identity before a crash can leave orphan canonical layers.
            if not (self.root / 'manifest.json').exists():
                atomic_json(self.root / 'manifest.json', self.manifest)
            atomic_json(self.root / 'persistent/skeleton_graph.json', self._document(**self.skeleton))
            atomic_json(self.root / 'dynamic/frontier_state.json',
                        self._document(session_id=self.manifest['session_id'], nodes=self.frontiers, attachments=self.attachments))
            self.export_unified_graph()
            atomic_json(self.root / 'manifest.json', self.manifest)
            self._dirty = False

    def _read(self, relative):
        for suffix in ('', '.bak'):
            path = self.root / (relative + suffix)
            try:
                value = json.loads(path.read_text())
                if value.get('schema_version') != SCHEMA_VERSION:
                    raise ValueError('Unsupported schema_version')
                if suffix:
                    self.issues.append('Recovered backup: ' + relative)
                return value
            except FileNotFoundError:
                continue
            except (ValueError, UnicodeError, AttributeError) as error:
                self.issues.append(f'{relative}{suffix}: {error}')
        return None

    def load_memory(self):
        """Validate canonical layers; derived snapshot is never an input."""
        with self.lock:
            self.issues = []
            manifest = self._read('manifest.json')
            if manifest is None:
                if any((self.root / p).exists() for p in ('persistent/skeleton_graph.json', 'dynamic/frontier_state.json')):
                    raise ValueError('UTM layers exist without a valid manifest; preserve directory and inspect backups')
                return self.issues
            if manifest.get('map_key') != identity(manifest):
                raise ValueError('UTM manifest coordinate identity mismatch')
            self.manifest = manifest
            # Drop obsolete metadata from older stores; no event log is read.
            removed_history_path = self.manifest.get('paths', {}).pop('history', None) is not None
            skel = self._read('persistent/skeleton_graph.json') or {}
            dynamic = self._read('dynamic/frontier_state.json') or {}
            for layer in (skel, dynamic):
                if layer.get('map_key') != manifest['map_key'] or layer.get('frame_id') != manifest['frame_id']:
                    self.issues.append('Discarded layer with mismatched map_key')
                    layer.clear()
            seen = set()

            def nodes(items, transient):
                result = []
                for node in items:
                    nid = node.get('node_id')
                    try:
                        ok = valid_id(nid) and nid not in seen and math.isfinite(node['x']) and math.isfinite(node['y'])
                        ok = ok and node['node_type'] in (('FRONTIER',) if transient else ('JUNCTION', 'ENDPOINT', 'LOOP_ANCHOR'))
                    except (KeyError, TypeError):
                        ok = False
                    if not ok:
                        self.issues.append(f'Discarded invalid node {nid}')
                        continue
                    seen.add(nid)
                    valid = []
                    for observation in node.get('observations', []):
                        relative = Path(observation.get('image_path', ''))
                        full = (self.root / relative).resolve()
                        safe = (not relative.is_absolute() and '..' not in relative.parts and
                                full.is_relative_to(self.root / 'observations') and full.is_file())
                        if safe and valid_id(observation.get('observation_id')):
                            valid.append(observation)
                        else:
                            self.issues.append(f'Missing or unsafe image for {nid}: {relative}')
                    node['observations'] = valid
                    result.append(node)
                return result

            self.skeleton = dict(nodes=nodes(skel.get('nodes', []), False), edges=[], map_info=skel.get('map_info', {}))
            self.frontiers = nodes(dynamic.get('nodes', []), True)
            persistent = {n['node_id'] for n in self.skeleton['nodes']}
            frontier_ids = {n['node_id'] for n in self.frontiers}
            edge_ids = set()

            def edges(items, attachment):
                result = []
                for edge in items:
                    eid = edge.get('edge_id')
                    start, end = edge.get('start_node_id'), edge.get('end_node_id')
                    try:
                        path = edge['path']
                        # A Junction may itself be the safe Frontier approach.
                        minimum_points = 1 if attachment and edge.get('path_role') == 'KNOWN_FREE_APPROACH' else 2
                        safe = len(path) >= minimum_points and all(len(p) >= 2 and all(math.isfinite(v) for v in p) for p in path)
                        safe = safe and math.isfinite(edge['length']) and edge['length'] >= 0
                    except (KeyError, TypeError):
                        safe = False
                    if (not safe or not valid_id(eid) or eid in edge_ids or start not in persistent or
                            end not in (frontier_ids if attachment else persistent)):
                        self.issues.append(f'Discarded invalid edge {eid}')
                        continue
                    edge_ids.add(eid)
                    result.append(edge)
                return result

            self.skeleton['edges'] = edges(skel.get('edges', []), False)
            self.attachments = edges(dynamic.get('attachments', []), True)
            by_frontier = {e['end_node_id']: e['start_node_id'] for e in self.attachments}
            for node in self.frontiers:
                if node.get('parent_junction_id') != by_frontier.get(node['node_id']):
                    self.issues.append('Cleared dangling attachment on ' + node['node_id'])
                    node['parent_junction_id'] = by_frontier.get(node['node_id'])
            # Reconstruct source maps for restarting only the recorder, not the simulator.
            self._node_sources = {n['source_id']: n['node_id'] for n in self.skeleton['nodes']
                                  if n.get('source_session') == manifest['session_id']}
            self._edge_sources = {e['source_id']: e['edge_id'] for e in self.skeleton['edges']
                                  if e.get('source_session') == manifest['session_id']}
            self._dirty = removed_history_path
            return list(self.issues)

    def get_topology_graph(self, include_inactive=False):
        with self.lock:
            nodes = self.skeleton['nodes'] + self.frontiers
            if not include_inactive:
                nodes = [n for n in nodes if n['status'] == 'ACTIVE']
            ids = {n['node_id'] for n in nodes}
            edges = [e for e in self.skeleton['edges'] + self.attachments
                     if e['start_node_id'] in ids and e['end_node_id'] in ids and
                     (include_inactive or e['status'] == 'ACTIVE')]
            return deepcopy(self._document(nodes=nodes, edges=edges))

    def get_active_frontiers(self):
        with self.lock:
            return deepcopy([n for n in self.frontiers if n['status'] == 'ACTIVE'])

    def get_frontiers_by_junction(self, node_id):
        return [n for n in self.get_active_frontiers() if n.get('parent_junction_id') == node_id]

    def get_node_observations(self, node_id):
        with self.lock:
            node = self._find_node(node_id)
            return deepcopy(node['observations']) if node else []

    def export_unified_graph(self):
        graph = self.get_topology_graph(include_inactive=True)
        atomic_json(self.root / 'snapshots/unified_graph_latest.json', graph)
        return graph
