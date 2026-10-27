"""Vision contract tests: no ROS, GPU, weights or network downloads required."""
import importlib
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np


def require_module(test, name):
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as exc:
        test.fail(f"Required vision module {name} is not available: {exc}")


class VisionContractTests(unittest.TestCase):
    def test_goal_aliases_work_without_dataset_metadata(self):
        labels = require_module(self, 'vlm.label_utils')
        for raw, expected in [('plant', 'potted plant'), ('sofa', 'couch'), ('tv_monitor', 'tv')]:
            self.assertEqual(labels.normalize_objectnav_label(raw, '/data/hm3d/v2/val.json.gz'), expected)
        self.assertEqual(labels.normalize_objectnav_label('cushion', '/data/mp3d/v1/val.json.gz'), 'pillow')

    def test_target_aliases_duplicates_and_masks_stay_aligned(self):
        module = require_module(self, 'vlm.utils.get_object_utils')
        # Reject a partial migration before importing the old four-model clients.
        self.assertTrue(hasattr(module, 'yoloe_detector'), 'YOLOE must be the sole detector')
        from vlm.detector.detections import ObjectDetections
        rgb = np.zeros((12, 20, 3), dtype=np.uint8)
        first = np.zeros((12, 20), dtype=np.uint8); first[2:5, 3:7] = 1
        second = np.zeros_like(first); second[6:9, 9:13] = 1
        result = ObjectDetections(
            boxes=[[.1, .1, .4, .5], [.4, .4, .8, .9], [.1, .2, .3, .4]],
            logits=[.91, .72, .81], phrases=['chair', 'couch', 'sofa'],
            masks=[first, second, first])
        cfg = SimpleNamespace(yoloe=SimpleNamespace(confidence_threshold=.3, iou_threshold=.5, agnostic_nms=True))
        with patch.object(module.yoloe_detector, 'predict', return_value=result) as predict:
            annotated, scores, masks, indices = module.get_object('couch | sofa | couch', rgb, cfg, ['chair', 'chair', 'sofa'])
        self.assertEqual(predict.call_args.kwargs['classes'], ['couch', 'sofa', 'chair'])
        self.assertEqual(indices, [1, 0, 0])
        np.testing.assert_allclose(scores, [.91, .72, .81])
        np.testing.assert_array_equal(masks[0], first)
        np.testing.assert_array_equal(masks[1], second)
        self.assertEqual(annotated.shape, rgb.shape)
        self.assertFalse(rgb.any(), 'Annotation must not mutate the RGB observation')

    def test_no_detections_returns_empty_aligned_lists(self):
        module = require_module(self, 'vlm.utils.get_object_utils')
        self.assertTrue(hasattr(module, 'yoloe_detector'), 'YOLOE must be the sole detector')
        from vlm.detector.detections import ObjectDetections
        cfg = SimpleNamespace(yoloe=SimpleNamespace(confidence_threshold=.3, iou_threshold=.5, agnostic_nms=True))
        with patch.object(module.yoloe_detector, 'predict', return_value=ObjectDetections([], [], [], [])):
            _, scores, masks, indices = module.get_object('chair', np.zeros((12,20,3), dtype=np.uint8), cfg, [])
        self.assertEqual((scores, masks, indices), ([], [], []))

    def test_yoloe_client_decodes_lite_masks_at_original_resolution(self):
        module = require_module(self, 'vlm.detector.yoloe')
        from vlm.server_wrapper import bool_arr_to_str
        rgb = np.zeros((12,20,3), dtype=np.uint8)
        mask = np.zeros((12,20), dtype=np.uint8); mask[3:7,5:11] = 1
        payload = dict(boxes=[[.1,.2,.6,.8]], logits=[.8], phrases=['chair'], masks=[bool_arr_to_str(mask)])
        with patch.object(module, 'send_request', return_value=payload):
            detections = module.YOLOEClient().predict(rgb, ['chair'])
        np.testing.assert_array_equal(detections.masks[0], mask)
        self.assertEqual(detections.boxes.shape, (1,4))

    def test_yoloe_rejects_missing_masks_instead_of_using_boxes_as_objects(self):
        module = require_module(self, 'vlm.detector.yoloe')
        payload = dict(boxes=[[0,0,1,1]], logits=[.8], phrases=['chair'], masks=[])
        with patch.object(module, 'send_request', return_value=payload):
            with self.assertRaises(ValueError):
                module.YOLOEClient().predict(np.zeros((12,20,3), dtype=np.uint8), ['chair'])

    def test_clip_keeps_raw_cosine_including_negative_values(self):
        module = require_module(self, 'vlm.similarity.clip')
        with patch.object(module, 'send_request', return_value={'cosine': -.2}):
            self.assertAlmostEqual(module.CLIPClient().cosine(np.zeros((2,2,3), dtype=np.uint8), 'chair'), -.2)

    def test_clip_rejects_invalid_score(self):
        module = require_module(self, 'vlm.similarity.clip')
        for score in [float('nan'), float('inf'), 3.0]:
            with patch.object(module, 'send_request', return_value={'cosine': score}):
                with self.assertRaises(ValueError):
                    module.CLIPClient().cosine(np.zeros((2,2,3), dtype=np.uint8), 'chair')

    def test_clip_context_prompt_preserves_lite_behavior(self):
        module = require_module(self, 'vlm.utils.get_clip_similarity')
        for room, expected in [('bedroom', 'Seems like there is a bedroom or a bed ahead?'), ('everywhere', 'Seems like there is a bed ahead?')]:
            with patch.object(module.clip_client, 'cosine', return_value=.25) as cosine:
                self.assertEqual(module.get_clip_similarity(np.zeros((2,2,3), dtype=np.uint8), 'bed', room), .25)
            self.assertEqual(cosine.call_args.args[1], expected)


class YOLOEModelAdapterTests(unittest.TestCase):
    def test_empty_prompt_does_not_call_model_and_has_shaped_empty_boxes(self):
        from vlm.detector.yoloe import YOLOEDetector
        detector = YOLOEDetector.__new__(YOLOEDetector)
        result = detector.predict(np.zeros((12,20,3), dtype=np.uint8), [])
        self.assertEqual(result.boxes.shape, (0,4))
        self.assertEqual(result.masks, [])

    def test_model_adapter_uses_bgr_retina_masks_and_returns_numpy(self):
        import threading
        import torch
        from vlm.detector.yoloe import YOLOEDetector
        detector = YOLOEDetector.__new__(YOLOEDetector)
        detector._predict_lock = threading.Lock()
        detector._apply_classes = lambda classes: None
        detector.image_size = 640
        detector.device = 'cpu'
        rgb = np.empty((12,20,3), dtype=np.uint8); rgb[:] = [220,30,10]
        class Boxes:
            xyxyn = torch.tensor([[.1,.2,.5,.6]])
            conf = torch.tensor([.9])
            cls = torch.tensor([0])
            def __len__(self):
                return 1
        def infer(**kwargs):
            np.testing.assert_array_equal(kwargs['source'][0,0], [10,30,220])
            self.assertTrue(kwargs['retina_masks'])
            return [SimpleNamespace(boxes=Boxes(), names={0:'chair'}, masks=SimpleNamespace(data=torch.ones((1,12,20))))]
        detector.model = SimpleNamespace(predict=infer)
        result = detector.predict(rgb, ['chair'])
        self.assertEqual(result.masks[0].shape, (12,20))
        self.assertEqual(result.boxes.shape, (1,4))
        self.assertEqual(result.phrases, ['chair'])


if __name__ == '__main__':
    unittest.main()
