import unittest
from basic_utils.simulation_cli import parse_simulation_args

class GeometryCliTest(unittest.TestCase):
    def test_default_keeps_apexnav(self):
        args, _ = parse_simulation_args([])
        self.assertEqual(args.planner, 'APEXNAV')

    def test_topology_uses_continuous(self):
        args, _ = parse_simulation_args(['--planner','TOPOLOGY','--mode','continuous'])
        self.assertEqual(args.planner, 'TOPOLOGY')
        with self.assertRaises(SystemExit):
            parse_simulation_args(['--planner','TOPOLOGY'])

    def test_geometric_mode_rejects_semantic_target(self):
        with self.assertRaises(SystemExit):
            parse_simulation_args(['--planner','TOPOLOGY','--mode','continuous','--target','chair'])
