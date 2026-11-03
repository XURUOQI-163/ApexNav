"""Read the actual C++ fixture with a strict JSON parser, independently of writer."""
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
doc = json.loads((root / 'fixture_graph.json').read_text(),
                 parse_constant=lambda v: (_ for _ in ()).throw(AssertionError(v)))
assert doc['frame_id'] == 'world' and doc['started_at_ns'] == 9
assert doc['captured_at_ns'] == 11 and doc['planner_state'] == 'TRACK'
assert doc['edge_count'] == len(doc['edges']) > 0
assert doc['waypoint_count'] == 1 and doc['frontier_count'] > 0
ids = {n['id'] for n in doc['nodes']}
for n in doc['nodes']:
    d = n['descriptor']
    assert len(d['sectors']) == 12 and len(d['directions']) == 72
    assert d['origin_xy_m'] == [0, 0]
    assert any(b['state'] == 'UNOBSERVED' and b['free_range_m'] is None for b in d['directions'])
    assert n['measured_at_node'] == (n['type'] == 'WAYPOINT')
    assert n['images']
    for image in n['images']:
        assert image['stamp_ns'] == 10 and image['camera_position_xyz_m'] == [0, 0, .88]
        assert image['orientation_xyzw'] == [0, 0, 0, 1]
        assert image['path'] == f"images/node_{n['id']}/slot_0_stamp_10.jpg"
for e in doc['edges']:
    assert e['a'] in ids and e['b'] in ids and e['verified'] and e['weight_m'] > 0
    assert len(e['path_xy_m']) >= 2
pruned = json.loads((root / 'pruned_graph.json').read_text())
for n in pruned['nodes']:
    for image in n['images']:
        assert (root / 'topology_snapshot' / image['path']).read_bytes() == bytes([1, 2, 3])
assert all(e['a'] in {n['id'] for n in pruned['nodes']} and
           e['b'] in {n['id'] for n in pruned['nodes']} for e in pruned['edges'])
print('PASS strict JSON: full descriptors, image poses/references and full edge geometry')
