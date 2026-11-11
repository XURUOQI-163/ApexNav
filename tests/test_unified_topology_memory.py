"""Small persistence/recovery checks; no Habitat, ROS or model servers required."""
import copy
import json
import os
import stat
from pathlib import Path
import tempfile
import unittest
from types import SimpleNamespace as NS

from topology_memory.store import UnifiedTopologyMemory


def context(session='run1', goal='chair', origin=0):
    return dict(session_id=session, scene_id='scene.glb', episode_id='1',
                start_position=[origin, 0, 0], start_rotation=[0, 0, 0, 1],
                frame_id='world', coordinate_convention='habitat_episodic_ros_v1',
                started_at_ns=100, goal_text=goal, model_name='blip2')


def skeleton():
    return dict(nodes=[dict(source_id=1, node_type='JUNCTION', x=0., y=0., visit_state='CONFIRMED', visit_radius=.5),
                       dict(source_id=2, node_type='JUNCTION', x=2., y=0., visit_state='CONFIRMED', visit_radius=.5)],
                edges=[dict(source_id=1, start_source_id=1, end_source_id=2,
                            path=[[0., 0.], [1., .2], [2., 0.]], length=2.04,
                            min_clearance=.3, mean_clearance=None)], map_info={})


def frontiers():
    return [dict(source_id=7, x=1., y=1., status='ACTIVE',
                 parent_source_id=1, approach_pose=dict(x=1., y=.9, yaw=1.57),
                 semantic=dict(goal_text='chair', model_name='blip2',
                               raw_similarity=.23, fused_score=.21, normalized_score=None),
                 cells=[dict(token=12, pixel=[1, 1], value=.21)],
                 attachment=dict(path=[[0., 0.], [1., 1.]], length=1.41))]


class MemoryTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name) / 'topology'
        self.m = UnifiedTopologyMemory(self.root, save_frontier_images=True)
        self.m.begin_session(context())
        self.m.save_skeleton_graph(skeleton())
        self.m.save_frontier_state(frontiers())

    def tearDown(self):
        self.temp.cleanup()

    def _image(self, node_id, stamp=200, replace_id=None):
        node=self.m._find_node(node_id)
        return self.m.save_observation(node_id, b'encoded test image',
            dict(stamp_ns=stamp, camera_pose=[0, 0, .88, 0, 0, 0, 1],
                 robot_pose=[node['x'], node['y'], 0, 0, 0, 0, 1], camera_yaw=0.), replace_id=replace_id)

    def test_disappeared_nodes_edges_and_images_are_removed_after_flush(self):
        junction = self.m._node_sources[1]
        frontier = self.m.frontiers[0]['node_id']
        images = [self._image(junction), self._image(frontier)]
        self.m.flush()
        remaining = skeleton()
        remaining['nodes'] = remaining['nodes'][1:]
        remaining['edges'] = []
        self.m.save_skeleton_graph(remaining)
        self.m.save_frontier_state([])
        self.assertEqual(len(self.m.skeleton['nodes']), 1)
        self.assertEqual(self.m.skeleton['edges'], [])
        self.assertEqual(set(self.m._node_sources), {2})
        self.assertEqual(self.m._edge_sources, {})
        self.assertEqual(self.m.get_node_observations(junction), [])
        self.assertTrue(all((self.root / o['image_path']).exists() for o in images))
        self.m.flush()
        self.assertTrue(all(not (self.root / o['image_path']).exists() for o in images))
        other = UnifiedTopologyMemory(self.root)
        other.load_memory()
        self.assertEqual(len(other.skeleton['nodes']), 1)
        self.assertEqual(other.frontiers, [])

    def test_candidate_and_capture_distance_gate_and_legacy_cleanup(self):
        source=skeleton();source['nodes'][0]['visit_state']='CANDIDATE'
        self.m.save_skeleton_graph(source)
        nid=self.m._node_sources[1]
        self.assertIsNone(self._image(nid))
        source['nodes'][0]['visit_state']='CONFIRMED'
        self.m.save_skeleton_graph(source)
        self.assertIsNotNone(self._image(nid))
        self.assertIsNone(self.m.save_observation(nid,b'image',dict(stamp_ns=201,robot_pose=[.6,0.])))
        self.m.flush()
        restored=UnifiedTopologyMemory(self.root);restored.load_memory()
        self.assertEqual(restored._find_node(nid)['visit_state'],'CONFIRMED')
        # Legacy remote-observation pictures are not proof of an actual visit.
        path=self.root/'persistent/skeleton_graph.json';data=json.loads(path.read_text())
        for node in data['nodes']:node.pop('visit_state',None)
        path.write_text(json.dumps(data))
        restored=UnifiedTopologyMemory(self.root);restored.load_memory();restored.flush()
        self.assertEqual(restored._find_node(nid)['visit_state'],'CANDIDATE')
        self.assertEqual(restored.get_node_observations(nid),[])

    def test_dormant_frontier_images_remain_valid(self):
        node = self.m.frontiers[0]['node_id']
        image = self._image(node)
        data = frontiers(); data[0]['status'] = 'DORMANT'
        self.m.save_frontier_state(data); self.m.flush()
        self.assertEqual(len(self.m.frontiers), 1)
        self.assertTrue((self.root / image['image_path']).exists())

    def test_observation_replacement_collects_old_jpeg_only_after_successful_flush(self):
        from unittest.mock import patch
        from topology_memory import store
        node = self.m.frontiers[0]['node_id']
        old = self._image(node); self.m.flush()
        new = self._image(node, 201, old['observation_id'])
        with patch.object(store, 'atomic_json', side_effect=OSError('disk unavailable')):
            with self.assertRaises(OSError): self.m.flush()
        self.assertTrue((self.root / old['image_path']).exists())
        self.m.flush()
        self.assertFalse((self.root / old['image_path']).exists())
        self.assertTrue((self.root / new['image_path']).exists())

    def test_load_discards_retired_records_and_collects_orphans_without_touching_archives(self):
        retired = self.m._node_sources[1]
        image = self._image(retired); self.m.flush()
        path = self.root / 'persistent/skeleton_graph.json'
        data = json.loads(path.read_text())
        for node in data['nodes']:
            if node['node_id'] == retired: node['status'] = 'RETIRED'
        path.write_text(json.dumps(data))
        orphan = self.root / 'observations/frontier/F_old/O_orphan.jpg'
        orphan.parent.mkdir(parents=True); orphan.write_bytes(b'orphan')
        archive = self.root / 'archives/previous/observations/image.jpg'
        archive.parent.mkdir(parents=True); archive.write_bytes(b'archive')
        other = UnifiedTopologyMemory(self.root); other.load_memory()
        self.assertEqual(len(other.skeleton['nodes']), 1)
        self.assertEqual(other.skeleton['edges'], [])
        other.flush()
        self.assertFalse((self.root / image['image_path']).exists())
        self.assertFalse(orphan.exists())
        self.assertTrue(archive.exists())

    def test_new_session_matches_visible_nodes_then_drops_unconfirmed_history(self):
        kept = self.m._node_sources[1]
        discarded = self.m._node_sources[2]
        image = self._image(discarded); self.m.flush()
        self.m.begin_session(context('run2'))
        current = skeleton(); current['nodes'] = current['nodes'][:1]; current['edges'] = []
        self.m.save_skeleton_graph(current); self.m.flush()
        self.assertEqual([n['node_id'] for n in self.m.skeleton['nodes']], [kept])
        self.assertFalse((self.root / image['image_path']).exists())

    def test_default_policy_disables_frontier_jpeg_and_compacts_state(self):
        memory = UnifiedTopologyMemory(self.root / 'default')
        memory.begin_session(context()); memory.save_skeleton_graph(skeleton())
        memory.save_frontier_state(frontiers())
        node = memory.frontiers[0]
        self.assertNotIn('cells', node)
        self.assertNotIn('observations', node)
        self.assertEqual(node['semantic']['fused_score'], .21)
        self.assertIsNone(memory.save_observation(node['node_id'], b'image', dict(stamp_ns=1)))
        memory.flush()
        self.assertFalse(list((memory.root / 'observations/frontier').rglob('*.jpg')))
        saved = json.loads((memory.root / 'dynamic/frontier_state.json').read_text())
        self.assertNotIn('cells', saved['nodes'][0])
        self.assertNotIn('observations', saved['nodes'][0])
        self.assertEqual(len(saved['attachments']), 1)

    def test_type_specific_limits_and_existing_image_cleanup(self):
        data = skeleton(); data['nodes'][1]['node_type'] = 'ENDPOINT'
        self.m.save_skeleton_graph(data)
        junction, endpoint = self.m._node_sources[1], self.m._node_sources[2]
        for i in range(6):
            self._image(junction, 200 + i)
            self._image(endpoint, 200 + i)
        self.assertEqual(len(self.m.get_node_observations(junction)), 4)
        self.assertEqual(len(self.m.get_node_observations(endpoint)), 1)
        # Simulate legacy records written with a larger cap, then migrate on load.
        self.m.max_junction_observations = self.m.max_deadend_observations = 8
        for i in range(3):
            self._image(junction, 300 + i); self._image(endpoint, 300 + i)
        frontier = self.m.frontiers[0]['node_id']; old = self._image(frontier)
        self.m.flush()
        other = UnifiedTopologyMemory(self.root)
        other.load_memory(); other.flush()
        self.assertNotIn('observations', other.frontiers[0])
        self.assertFalse((self.root / old['image_path']).exists())
        self.assertEqual(len(other.get_node_observations(junction)), 4)
        self.assertEqual(len(other.get_node_observations(endpoint)), 1)

    def test_atomic_files_remain_host_readable_after_replacement_and_backup(self):
        from topology_memory.store import atomic_bytes, atomic_json
        image = self.root / 'observations/frontier/readable.jpg'
        document = self.root / 'manifest.json'
        # mkstemp uses 0600 regardless of umask. All final publications must
        # still permit the host user's image viewer to read a root-owned file.
        previous_umask = os.umask(0o077)
        try:
            atomic_bytes(image, b'first image')
            atomic_json(document, {'revision': 1})
            for path in (image, document):
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o644)
            image.chmod(0o600)
            atomic_bytes(image, b'updated image')
            atomic_json(document, {'revision': 2})
            for path in (image, document, document.with_suffix('.json.bak')):
                self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o644)
            self.assertEqual(image.read_bytes(), b'updated image')
            self.assertEqual(json.loads(document.with_suffix('.json.bak').read_text()), {'revision': 1})
        finally:
            os.umask(previous_umask)

    def test_round_trip_queries_and_no_rewrite_when_unchanged(self):
        self.m.flush()
        target = self.root / 'persistent/skeleton_graph.json'
        before = target.stat().st_mtime_ns
        self.m.save_skeleton_graph(skeleton())
        self.m.save_frontier_state(frontiers())
        self.m.flush()
        self.assertEqual(before, target.stat().st_mtime_ns)
        other = UnifiedTopologyMemory(self.root)
        other.load_memory()
        graph = other.get_topology_graph()
        self.assertEqual(len(graph['nodes']), 3)
        self.assertEqual(len(graph['edges']), 2)
        frontier = other.get_active_frontiers()[0]
        self.assertEqual(frontier['semantic']['fused_score'], .21)
        self.assertEqual(other.get_frontiers_by_junction(frontier['parent_junction_id']), [frontier])
        self.assertEqual(graph['edges'][0]['path'], skeleton()['edges'][0]['path'])

    def test_zero_length_safe_attachment_preserves_separate_association(self):
        data = frontiers()
        data[0]['approach_pose'] = dict(x=0., y=0., yaw=0.)
        data[0]['attachment'] = dict(path=[[0., 0.]], length=0.,
                                    association_path=[[0., 0.], [1., 1.]],
                                    association_traversable=False,
                                    path_role='KNOWN_FREE_APPROACH')
        self.m.save_frontier_state(data)
        self.m.flush()
        other = UnifiedTopologyMemory(self.root)
        self.assertEqual(other.load_memory(), [])
        self.assertEqual(other.attachments[0]['path'], [[0., 0.]])
        self.assertFalse(other.attachments[0]['association_traversable'])

    def test_session_identity_and_goal_do_not_reuse_active_semantics(self):
        old = self.m.get_active_frontiers()[0]['node_id']
        self.m.flush()
        self.m.begin_session(context('run2', 'table'))
        self.assertEqual(self.m.get_active_frontiers(), [])
        data = frontiers()
        data[0]['semantic']['goal_text'] = 'table'
        self.m.save_skeleton_graph(skeleton())
        self.m.save_frontier_state(data)
        self.assertNotEqual(old, self.m.get_active_frontiers()[0]['node_id'])
        self.assertEqual(len(self.m.skeleton['nodes']), 2)
        self.m.begin_session(context('run3', origin=5))
        self.assertEqual(self.m.get_topology_graph()['nodes'], [])
        self.assertTrue(list((self.root / 'archives').glob('*/manifest.json')))

    def test_observation_roundtrip_and_missing_image_is_reported(self):
        node = self.m.get_active_frontiers()[0]['node_id']
        meta = dict(stamp_ns=200, camera_pose=[0, 0, .88, 0, 0, 0, 1],
                    robot_pose=[0, 0, 0, 0, 0, 0, 1], camera_yaw=0., bearing_to_node=.5)
        # The store accepts encoded bytes; codec validation belongs to the collector.
        obs = self.m.save_observation(node, b'encoded test image', meta)
        self.m.flush()
        self.assertEqual(len(self.m.get_node_observations(node)), 1)
        (self.root / obs['image_path']).unlink()
        other = UnifiedTopologyMemory(self.root)
        issues = other.load_memory()
        self.assertTrue(any('image' in issue for issue in issues))
        self.assertFalse(other.get_node_observations(node))

    def test_recovery_rejects_dangling_edges_and_unsafe_image_paths(self):
        self.m.flush()
        p = self.root / 'persistent/skeleton_graph.json'
        data = json.loads(p.read_text())
        data['edges'][0]['end_node_id'] = 'missing'
        data['nodes'][0]['observations'] = [dict(observation_id='bad', image_path='../outside.jpg')]
        p.write_text(json.dumps(data))
        other = UnifiedTopologyMemory(self.root)
        issues = other.load_memory()
        self.assertTrue(any('edge' in i for i in issues))
        self.assertTrue(any('image' in i for i in issues))
        self.assertEqual(other.skeleton['edges'], [])

    def test_frontier_updates_and_retirement_save_state_without_history(self):
        self.m.flush()
        data = frontiers()
        data[0]['semantic']['fused_score'] = .3
        self.m.save_frontier_state(data)
        self.m.flush()
        self.m.save_frontier_state([])
        self.m.flush()
        self.assertFalse(self.m.get_active_frontiers())
        saved = json.loads((self.root / 'dynamic/frontier_state.json').read_text())
        self.assertEqual(saved['nodes'], [])
        self.assertEqual(saved['attachments'], [])
        self.assertFalse((self.root / 'dynamic/frontier_history.jsonl').exists())
        self.assertNotIn('history', json.loads((self.root / 'manifest.json').read_text())['paths'])

    def test_metadata_survives_eviction_of_scored_frame_cache(self):
        data = frontiers()
        data[0]['semantic'].update(raw_frame_stamp_ns=500, computed_at=600, model_version='weights-v1')
        self.m.save_frontier_state(data)
        data[0]['semantic'].update(computed_at=None, model_version=None)
        self.m.save_frontier_state(data)
        semantic = self.m.get_active_frontiers()[0]['semantic']
        self.assertEqual(semantic['computed_at'], 600)
        self.assertEqual(semantic['model_version'], 'weights-v1')

    def test_legacy_history_is_ignored_and_removed_from_manifest_paths(self):
        self.m.flush()
        manifest_path = self.root / 'manifest.json'
        manifest = json.loads(manifest_path.read_text())
        manifest['paths']['history'] = 'dynamic/frontier_history.jsonl'
        manifest_path.write_text(json.dumps(manifest))
        legacy = self.root / 'dynamic/frontier_history.jsonl'
        legacy.write_bytes(b'old incomplete event')
        other = UnifiedTopologyMemory(self.root)
        self.assertEqual(other.load_memory(), [])
        other.flush()
        self.assertNotIn('history', json.loads(manifest_path.read_text())['paths'])
        self.assertEqual(legacy.read_bytes(), b'old incomplete event')
        self.assertEqual(len(other.get_active_frontiers()), 1)

    def test_backup_recovers_corrupt_json(self):
        self.m.flush()
        changed = skeleton()
        changed['nodes'][0]['x'] = .1
        self.m.save_skeleton_graph(changed)
        self.m.flush()
        (self.root / 'persistent/skeleton_graph.json').write_text('{partial')
        other = UnifiedTopologyMemory(self.root)
        self.assertTrue(any('Recovered backup' in s for s in other.load_memory()))

    def test_interrupted_first_flush_has_recoverable_coordinate_manifest(self):
        from unittest.mock import patch
        from topology_memory import store
        real_write = store.atomic_json
        def interrupted(path, data):
            if path.name == 'frontier_state.json':
                raise OSError('simulated interrupted first flush')
            real_write(path, data)
        with patch.object(store, 'atomic_json', interrupted):
            with self.assertRaises(OSError):
                self.m.flush()
        recovered = UnifiedTopologyMemory(self.root)
        recovered.load_memory()
        self.assertEqual(recovered.manifest['scene_id'], 'scene.glb')


class KeyframePolicyTest(unittest.TestCase):
    def test_distinct_view_uses_angle_or_distance_against_every_saved_frame(self):
        from topology_memory.recorder import MemoryRecorder
        import math
        memory = NS(max_observations=8)
        memory.observation_limit = lambda node: memory.max_observations
        recorder = MemoryRecorder(memory, None)
        node = dict(observations=[dict(stamp_ns=1, camera_pose=[0., 0.], camera_yaw=0.)])
        self.assertFalse(recorder._distinct(node, 1, [2., 0.], math.pi))
        self.assertFalse(recorder._distinct(node, 2, [.1, 0.], .1))
        self.assertTrue(recorder._distinct(node, 2, [.75, 0.], 0.))
        self.assertTrue(recorder._distinct(node, 2, [0., 0.], math.pi / 4))
        node['observations'].append(dict(stamp_ns=2, camera_pose=[1., 0.], camera_yaw=0.))
        self.assertFalse(recorder._distinct(node, 3, [1.1, 0.], 0.))
        memory.max_observations = 2
        self.assertFalse(recorder._distinct(node, 3, [5., 0.], math.pi))


class FrontierAttachmentTest(unittest.TestCase):
    def test_adapter_keeps_unknown_association_out_of_safe_path(self):
        from topology_memory.recorder import frontier_data
        def point(x, y):
            return NS(x=x, y=y)
        node = NS(frontier_id=7, active=True, centroid=point(1., 1.), cells=[],
                  scored_cells=0, fused_score=0., confidence=0.)
        link = NS(frontier_id=7, active=True, connected=True, junction_node_id=1,
                  frontier_centroid=node.centroid, approach=point(1., .9), length=1.,
                  display_path=[point(0., .9), point(1., .9)],
                  path=[point(0., .9), point(.5, .9), point(1., .9)])
        state = NS(nodes=[node], backend='blip2')
        graph = NS(frontier_links=[link], map_info=NS(resolution=.05))
        for legacy in (False, True):
            if legacy:
                # A recorder restart may briefly see a latched old producer.
                link.display_path.append(node.centroid)
                link.length = 1.1
            data = frontier_data(state, graph, context(), {})[0]
            a = data['attachment']
            self.assertEqual(a['path'][-1], [1., .9])
            self.assertAlmostEqual(a['length'], 1.)
            self.assertEqual(a['association_path'], [[1., .9], [1., 1.]])
            self.assertFalse(a['association_traversable'])
            self.assertFalse(a['terminal_may_be_frontier_unknown'])
            self.assertEqual(data['x'], 1.)
            self.assertEqual(data['y'], 1.)


if __name__ == '__main__':
    unittest.main()
