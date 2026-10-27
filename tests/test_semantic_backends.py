import importlib
import sys
import subprocess
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
import torch
import requests
from vlm import server_wrapper as transport


def require(test, name):
    try:
        return importlib.import_module(name)
    except ModuleNotFoundError as exc:
        test.fail(str(exc))


class SemanticBackendTests(unittest.TestCase):
    def test_cli_selects_blip_without_swallowing_hydra_overrides(self):
        from basic_utils.simulation_cli import parse_simulation_args
        args, overrides = parse_simulation_args(['--mode','continuous','--semantic-model','blip2','test_epi_num=150'])
        self.assertEqual(args.semantic_model, 'blip2')
        self.assertEqual(overrides, ['test_epi_num=150'])
        self.assertEqual(parse_simulation_args([])[0].semantic_model, 'clip')

    def test_backend_selection_keeps_clip_path_and_prompt(self):
        routing = require(self, 'vlm.utils.get_semantic_similarity')
        rgb = np.zeros((4,4,3), dtype=np.uint8)
        with patch('vlm.utils.get_clip_similarity.clip_client.cosine', return_value=.123) as score:
            self.assertEqual(routing.get_semantic_similarity(rgb,'toilet','bathroom','clip'), .123)
            self.assertEqual(score.call_args.args[1], 'Seems like there is a bathroom or a toilet ahead?')
        with patch('vlm.utils.get_blip2_similarity.blip2_client.cosine', return_value=-.2) as score:
            self.assertEqual(routing.get_semantic_similarity(rgb,'chair','everywhere','blip2'), -.2)
            self.assertEqual(score.call_args.args[1], 'Seems like there is a chair ahead?')
        with self.assertRaises(ValueError):
            routing.get_semantic_similarity(rgb, 'chair','everywhere','invalid')

    def test_blip_itc_uses_preprocessing_and_never_itm_probability(self):
        module = require(self, 'vlm.similarity.blip2')
        model = module.BLIP2Similarity.__new__(module.BLIP2Similarity)
        model.torch = torch
        model.device = torch.device('cpu')
        model.vis_processors = {'eval': lambda image: torch.zeros((3,2,2))}
        model.text_processors = {'eval': lambda text: text.lower()}
        def inference(sample, match_head):
            self.assertEqual(match_head, 'itc')
            self.assertEqual(sample['text_input'], ['chair'])
            return torch.tensor([[-.25]])
        model.model = inference
        self.assertEqual(model.cosine(np.zeros((2,2,3), dtype=np.uint8),'CHAIR'), -.25)

    def test_blip_client_and_service_round_trip(self):
        module = require(self, 'vlm.similarity.blip2')
        server = module.BLIP2Server.__new__(module.BLIP2Server)
        server.cosine = lambda image, text: -.125
        http = transport.create_app(server, 'blip2').test_client()
        def post(url, **kwargs):
            self.assertTrue(url.endswith('/blip2'))
            result = http.post('/blip2', json=kwargs['json'])
            response = requests.Response(); response.status_code = result.status_code
            response._content = result.data; response.url = url
            return response
        with patch.object(transport.requests, 'post', side_effect=post):
            score = module.BLIP2Client().cosine(np.zeros((4,4,3), dtype=np.uint8), 'chair')
        self.assertEqual(score, -.125)

    def test_blip_rejects_invalid_scores(self):
        module = require(self, 'vlm.similarity.blip2')
        for value in [float('nan'),float('inf'),1.2]:
            with patch.object(module, 'send_request', return_value={'cosine':value}):
                with self.assertRaises(ValueError):
                    module.BLIP2Client().cosine(np.zeros((4,4,3), dtype=np.uint8),'chair')

    def test_clients_do_not_import_model_frameworks(self):
        require(self, 'vlm.similarity.blip2')
        result = subprocess.run([sys.executable,'-c',
            "import vlm.similarity.blip2, vlm.utils.get_semantic_similarity; import sys; "
            "assert not ({'lavis','torch','clip','rclpy'} & sys.modules.keys())"], capture_output=True,text=True)
        self.assertEqual(result.returncode,0,result.stderr)
