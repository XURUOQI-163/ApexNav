import unittest
from types import SimpleNamespace

import numpy as np
import torch

from vlm.similarity.clip import CLIPSimilarity


class CLIPModelTests(unittest.TestCase):
    def model_with_features(self, image_features):
        model = CLIPSimilarity.__new__(CLIPSimilarity)
        model.torch = torch
        model.device = torch.device('cpu')
        model.clip = SimpleNamespace(tokenize=lambda texts, **kwargs: torch.zeros((1,77)))
        model.preprocess = lambda image: torch.zeros((3,2,2))
        model.model = SimpleNamespace(
            encode_image=lambda image: torch.tensor([image_features], dtype=torch.float32),
            encode_text=lambda text: torch.tensor([[0.,5.]]))
        return model

    def test_model_returns_normalized_cosine_not_logit_or_probability(self):
        score = self.model_with_features([3.,4.]).cosine(np.zeros((2,2,3), dtype=np.uint8), 'chair')
        self.assertAlmostEqual(score, .8)

    def test_invalid_features_are_not_hidden_as_a_high_similarity(self):
        with self.assertRaises(ValueError):
            self.model_with_features([0.,0.]).cosine(np.zeros((2,2,3), dtype=np.uint8), 'chair')
