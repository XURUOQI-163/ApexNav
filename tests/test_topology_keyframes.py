import math
import unittest
from topology_memory.keyframe_policy import select_keyframe


class KeyframePolicyTest(unittest.TestCase):
    def node(self, kind='JUNCTION', state='CONFIRMED'):
        return dict(node_type=kind, visit_state=state, visit_radius=.5, x=0., y=0., observations=[])

    def view(self, oid, yaw, distance=.4):
        return dict(observation_id=oid, stamp_ns=int(oid[-1])+1,
                    camera_yaw=math.radians(yaw), camera_pose=[distance, 0.], robot_pose=[distance, 0.])

    def select(self, node, yaw=0., distance=.3, limit=4):
        return select_keyframe(node, 100, [distance, 0.], [distance, 0.], math.radians(yaw), limit)

    def test_visit_required_at_capture_time(self):
        self.assertEqual(self.select(self.node(state='CANDIDATE')), (False, None))
        self.assertEqual(self.select(self.node(), distance=.51), (False, None))
        self.assertEqual(self.select(self.node(), distance=.5), (True, None))

    def test_strict_angle_and_closer_replacement(self):
        node=self.node();node['observations']=[self.view('O1',0)]
        self.assertEqual(self.select(node, 61), (True, None))
        self.assertEqual(self.select(node, 60), (True, 'O1'))
        self.assertEqual(self.select(node, 20, .45), (False, None))
        self.assertEqual(self.select(node, 20, .2), (True, 'O1'))

    def test_capacity_wraparound_and_sector_overlap(self):
        node=self.node();node['observations']=[self.view('O1',0),self.view('O2',90),self.view('O3',180),self.view('O4',270)]
        self.assertEqual(self.select(node, 355), (True, 'O1'))
        self.assertEqual(self.select(node, 45), (False, None))
        node['observations']=node['observations'][:3]
        self.assertEqual(self.select(node, 270, limit=3), (False, None))

    def test_deadend_needs_no_visit_and_prefers_alignment(self):
        node=self.node('ENDPOINT', 'CANDIDATE')
        self.assertEqual(self.select(node, 180, 3., 1), (True, None))
        node['observations']=[self.view('O1',160,3.)]
        self.assertEqual(self.select(node,180,4.,1), (True,'O1'))
        self.assertEqual(self.select(node,150,1.,1), (False,None))
        self.assertEqual(self.select(node,160,2.,1), (True,'O1'))


if __name__ == '__main__':
    unittest.main()
