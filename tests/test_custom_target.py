import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from basic_utils.simulation_cli import parse_simulation_args
from vlm import label_utils


class CustomTargetTests(unittest.TestCase):
    def test_cli_preserves_description_outside_hydra(self):
        description = 'yellow chair near the desk, marked "A" {left}'
        args, overrides = parse_simulation_args([
            '--mode', 'continuous', '--target', description,
            '--semantic-model', 'blip2', 'test_epi_num=550'])
        self.assertEqual(args.target, description)
        self.assertEqual(overrides, ['test_epi_num=550'])
        self.assertEqual(parse_simulation_args(['--target', '黄色的椅子'])[0].target, '黄色的椅子')
        self.assertIsNone(parse_simulation_args([])[0].target)

    def test_empty_description_is_rejected(self):
        for value in ('   ', 'yellow chair\nred chair'):
            with self.subTest(value=value), self.assertRaises(SystemExit):
                parse_simulation_args(['--target', value])

    def test_custom_description_replaces_episode_category_without_normalization(self):
        self.assertEqual(label_utils.resolve_navigation_target(
            'tv_monitor', '/data/hm3d/val.json.gz', '  red bookshelf  '), 'red bookshelf')
        self.assertEqual(label_utils.resolve_navigation_target(
            'tv_monitor', '/data/hm3d/val.json.gz'), 'tv')

    def test_new_description_without_llm_uses_default_context_without_cache_write(self):
        from llm.answer_reader import answer_reader
        with tempfile.TemporaryDirectory() as directory:
            answers = Path(directory)/'answers.txt'
            responses = Path(directory)/'responses.txt'
            existing = "chair: [0.45, 'living room']\n"
            answers.write_text(existing)
            with patch.object(answer_reader, 'get_answer', side_effect=AssertionError('LLM called')):
                result = answer_reader.read_answer(str(answers), str(responses),
                    'yellow chair', SimpleNamespace(llm_client='none'))
            self.assertEqual(result, ('everywhere', .5))
            self.assertEqual(answers.read_text(), existing)
            self.assertFalse(responses.exists())
            self.assertEqual(answer_reader.read_answer(str(answers), str(responses),
                'chair', SimpleNamespace(llm_client='none')), ('living room', .45))

    def test_full_description_reaches_detector_and_both_semantic_backends(self):
        import numpy as np
        from vlm.utils.get_object_utils import get_object
        from vlm.utils.get_semantic_similarity import get_semantic_similarity
        target = label_utils.resolve_navigation_target('chair', '/data/hm3d/val.json.gz',
                                                       'yellow chair near the desk')
        rgb = np.zeros((4,4,3), dtype=np.uint8)
        detections = SimpleNamespace(boxes=[[0.,0.,1.,1.]], logits=[.8],
            phrases=[target], masks=[np.ones((4,4), dtype=np.uint8)])
        cfg = SimpleNamespace(yoloe=SimpleNamespace(confidence_threshold=.3,
            iou_threshold=.5, agnostic_nms=True))
        with patch('vlm.utils.get_object_utils.yoloe_detector.predict', return_value=detections) as detect:
            _, scores, masks = get_object(target, rgb, cfg)
            self.assertEqual(detect.call_args.kwargs['classes'], [target])
        for backend in ('clip', 'blip2'):
            with patch(f'vlm.utils.get_{backend}_similarity.{backend}_client.cosine', return_value=.3) as infer:
                self.assertEqual(get_semantic_similarity(rgb, target, 'everywhere', backend), .3)
                self.assertEqual(infer.call_args.args[1], f'Seems like there is a {target} ahead?')
