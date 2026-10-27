"""Independent BLIP2 ITC service, adapted from ApexNav's original BLIP2ITM.

Original: vlm/itm/blip2itm.py in the repository's ros2-jazzy HEAD.
Only the server imports LAVIS/torch. CLIP remains in similarity/clip.py.
"""
import math
import os
import time
from vlm.server_wrapper import ServerMixin, host_model, send_request, str_to_image


class BLIP2Client:
    def __init__(self, port=12185):
        self.url = os.environ.get('BLIP2_SERVER_URL', f'http://127.0.0.1:{port}/blip2')

    def cosine(self, image, text):
        score = float(send_request(self.url, image=image, txt=text)['cosine'])
        if not math.isfinite(score) or not -1.0 <= score <= 1.0:
            raise ValueError(f'BLIP2 returned invalid ITC similarity: {score}')
        return score


class BLIP2Similarity:
    def __init__(self, model_type='pretrain', device=None):
        import torch
        from lavis.models import load_model_and_preprocess
        self.torch = torch
        self.device = torch.device(device or ('cuda' if torch.cuda.is_available() else 'cpu'))
        self.model, self.vis_processors, self.text_processors = load_model_and_preprocess(
            name='blip2_image_text_matching', model_type=model_type,
            is_eval=True, device=self.device)

    def cosine(self, image, text):
        from PIL import Image
        img = self.vis_processors['eval'](Image.fromarray(image)).unsqueeze(0).to(self.device)
        txt = self.text_processors['eval'](text)
        with self.torch.inference_mode():
            score = self.model({'image': img, 'text_input': [txt]}, match_head='itc').item()
        if not math.isfinite(score) or not -1.0 <= score <= 1.0:
            raise ValueError(f'BLIP2 returned invalid ITC similarity: {score}')
        return float(score)


class BLIP2Server(ServerMixin, BLIP2Similarity):
    def process_payload(self, payload):
        start = time.perf_counter()
        score = self.cosine(str_to_image(payload['image']), payload['txt'])
        return {'cosine': score, 'backend': 'blip2', 'head': 'itc',
                'timing': {'server_total_ms': (time.perf_counter() - start) * 1000}}


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='BLIP2 ITC semantic similarity service')
    parser.add_argument('--port', type=int, default=12185)
    parser.add_argument('--device', default=None)
    parser.add_argument('--model-type', default='pretrain', choices=['pretrain','pretrain_vitL','coco'])
    args = parser.parse_args()
    model = BLIP2Server(model_type=args.model_type, device=args.device)
    print(f'BLIP2 ITC ready on port {args.port}', flush=True)
    host_model(model, name='blip2', port=args.port)
