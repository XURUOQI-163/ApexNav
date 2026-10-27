import unittest
from unittest.mock import patch

import numpy as np
import requests

from vlm import server_wrapper as transport


class TransportTests(unittest.TestCase):
    def test_rgb_round_trip_preserves_channel_order(self):
        rgb = np.empty((16, 24, 3), dtype=np.uint8)
        rgb[:] = [220, 30, 10]
        result = transport.str_to_image(transport.image_to_str(rgb, quality=100))
        np.testing.assert_allclose(result.astype(float), rgb.astype(float), atol=3)

    def test_http_error_is_reported_without_exiting_the_process(self):
        response = requests.Response(); response.status_code = 500
        response._content = b'{"error":"wrong checkpoint"}'
        response.url = 'http://127.0.0.1:12184/yoloe'
        with patch.object(transport.requests, 'post', return_value=response), patch('time.sleep'):
            with self.assertRaises(RuntimeError):
                transport.send_request(response.url, image=np.zeros((2,2,3), dtype=np.uint8))

    def test_yoloe_request_and_mask_survive_real_http_serialization(self):
        self.assertTrue(hasattr(transport, 'create_app'), 'Service needs a testable Flask application')
        from vlm.detector.yoloe import YOLOEServer, YOLOEClient
        from vlm.detector.detections import ObjectDetections
        server = YOLOEServer.__new__(YOLOEServer)  # Do not load weights.
        rgb = np.empty((16,24,3), dtype=np.uint8); rgb[:] = [220,30,10]
        mask = np.zeros((16,24), dtype=np.uint8); mask[3:10,4:12] = 1
        def predict(image, classes, **kwargs):
            np.testing.assert_allclose(image.astype(float), rgb.astype(float), atol=5)
            self.assertEqual(classes, ['chair'])
            return ObjectDetections([[.1,.2,.5,.6]], [.85], ['chair'], [mask])
        server.predict = predict
        app = transport.create_app(server, 'yoloe')
        client = app.test_client()
        def post(url, **kwargs):
            result = client.post('/yoloe', json=kwargs['json'])
            response = requests.Response(); response.status_code = result.status_code
            response._content = result.data; response.url = url
            return response
        with patch.object(transport.requests, 'post', side_effect=post):
            detections = YOLOEClient().predict(rgb, ['chair'])
        np.testing.assert_array_equal(detections.masks[0], mask)
        self.assertEqual(detections.phrases, ['chair'])

    def test_clip_server_client_wire_contract(self):
        self.assertTrue(hasattr(transport, 'create_app'), 'Service needs a testable Flask application')
        from vlm.similarity.clip import CLIPServer, CLIPClient
        server = CLIPServer.__new__(CLIPServer)
        server.cosine = lambda image, text: -.125
        client = transport.create_app(server, 'clip').test_client()
        def post(url, **kwargs):
            result = client.post('/clip', json=kwargs['json'])
            response = requests.Response(); response.status_code = result.status_code
            response._content = result.data; response.url = url
            return response
        with patch.object(transport.requests, 'post', side_effect=post):
            score = CLIPClient().cosine(np.zeros((4,4,3), dtype=np.uint8), 'chair')
        self.assertEqual(score, -.125)
