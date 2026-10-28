"""Compile small fixtures against the production sparse semantic memory."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest


class FrontierSemanticsTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'C++ compiler required')
    def test_sparse_fusion_lifecycle_and_delayed_projection(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(pathlib.Path(directory) / 'frontier_test')
            result = subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra',
                '-I', str(root / 'src/planner/plan_env/include'),
                str(root / 'tests/cpp/test_frontier_semantic_memory.cpp'), '-o', binary],
                capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([binary], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
