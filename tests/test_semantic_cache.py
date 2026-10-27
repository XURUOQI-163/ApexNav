"""Compile/run the production timestamp join against delayed-frame fixtures."""
import pathlib
import shutil
import subprocess
import tempfile
import unittest


class SemanticCacheTests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('g++'), 'C++ compiler required for cache regression')
    def test_delayed_frame_join_expiry_duplicates_and_reset(self):
        root = pathlib.Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            binary = str(pathlib.Path(directory) / 'cache_test')
            compile_result = subprocess.run(['g++','-std=c++17','-Wall','-Wextra',
                '-I',str(root/'src/planner/plan_env/include'),
                str(root/'tests/cpp/test_semantic_observation_cache.cpp'),'-o',binary],
                capture_output=True,text=True)
            self.assertEqual(compile_result.returncode,0,compile_result.stderr)
            result = subprocess.run([binary],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
