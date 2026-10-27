"""HTTP transport for YOLOE and CLIP, with Lite-compatible image/mask encoding."""
import base64
import os
import threading

import cv2
import numpy as np
import requests


class ServerMixin:
    def process_payload(self, payload):
        raise NotImplementedError


def create_app(model, name):
    from flask import Flask, jsonify, request

    app = Flask(__name__)
    request_lock = threading.Lock()

    @app.get('/health')
    def health():
        return jsonify({'model': name, 'status': 'ready'})

    @app.post(f'/{name}')
    def process_request():
        payload = request.get_json()
        try:
            with request_lock:
                result = model.process_payload(payload)
        except (ValueError, KeyError, TypeError) as exc:
            return jsonify({'error': str(exc)}), 400
        return jsonify(result)

    return app


def host_model(model, name, port=5000):
    create_app(model, name).run(host='127.0.0.1', port=port)


def bool_arr_to_str(arr):
    return base64.b64encode(np.asarray(arr, dtype=np.uint8).tobytes()).decode('ascii')


def str_to_bool_arr(value, shape):
    data = base64.b64decode(value, validate=True)
    return np.frombuffer(data, dtype=np.uint8).reshape(shape)


def image_to_str(image, quality=90):
    # Preserve Lite's array-channel round trip: OpenCV encodes and decodes the
    # same channels. Callers and models receive RGB; YOLOE converts at inference.
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError('Vision services require an HxWx3 uint8 RGB image')
    ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError('Could not encode RGB image')
    return base64.b64encode(encoded).decode('ascii')


def str_to_image(value):
    data = np.frombuffer(base64.b64decode(value, validate=True), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError('Invalid encoded image')
    return image


def send_request(url, **kwargs):
    timeout = float(kwargs.pop('request_timeout', os.environ.get('VLM_REQUEST_TIMEOUT', '120')))
    payload = {key: image_to_str(value) if isinstance(value, np.ndarray) else value
               for key, value in kwargs.items()}
    try:
        response = requests.post(url, json=payload, timeout=timeout)
        response.raise_for_status()
        result = response.json()
    except (requests.RequestException, ValueError) as exc:
        raise RuntimeError(f'Vision service request failed at {url}: {exc}. Check the service terminal.') from exc
    if not isinstance(result, dict):
        raise ValueError(f'Vision service {url} must return a JSON object')
    return result
