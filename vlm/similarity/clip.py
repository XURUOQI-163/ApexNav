"""CLIP ViT-B/32 cosine similarity, using the same model and encoding as Lite.

The HTTP client does not import torch or CLIP. Only the service loads weights.
"""
import math
import os
import time

from vlm.server_wrapper import ServerMixin, host_model, send_request, str_to_image


class CLIPClient:
    def __init__(self, port=12182):
        self.url = os.environ.get('CLIP_SERVER_URL', f'http://127.0.0.1:{port}/clip')

    def cosine(self, image, text):
        score = float(send_request(self.url, image=image, txt=text)['cosine'])
        if not math.isfinite(score) or not -1.0 <= score <= 1.0:
            raise ValueError(f'CLIP returned invalid cosine similarity: {score}')
        return score


class CLIPSimilarity:
    def __init__(self, model_name='ViT-B/32', device=None, download_root=None):
        import clip
        import torch

        self.clip = clip
        self.torch = torch
        self.device = torch.device(device or ('cuda' if torch.cuda.is_available() else 'cpu'))
        self.model, self.preprocess = clip.load(model_name, device=self.device, download_root=download_root)
        self.model.eval()

    def cosine(self, image, text):
        from PIL import Image

        image_input = self.preprocess(Image.fromarray(image)).unsqueeze(0).to(self.device)
        text_input = self.clip.tokenize([text], truncate=True).to(self.device)
        with self.torch.inference_mode():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_input)
            image_features = image_features / image_features.norm(dim=-1, keepdim=True)
            text_features = text_features / text_features.norm(dim=-1, keepdim=True)
            score = (image_features @ text_features.T).item()
        if not math.isfinite(score):
            raise ValueError('CLIP produced non-finite image/text features')
        return max(-1.0, min(1.0, float(score)))


class CLIPServer(ServerMixin, CLIPSimilarity):
    def process_payload(self, payload):
        start = time.perf_counter()
        score = self.cosine(str_to_image(payload['image']), payload['txt'])
        return {'cosine': score, 'timing': {'server_total_ms': (time.perf_counter() - start) * 1000}}


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='CLIP cosine similarity server')
    parser.add_argument('--port', type=int, default=12182)
    parser.add_argument('--model-name', default=os.environ.get('CLIP_MODEL_NAME', 'ViT-B/32'))
    parser.add_argument('--device', default=None)
    parser.add_argument('--download-root', default=os.environ.get('CLIP_DOWNLOAD_ROOT'))
    args = parser.parse_args()
    model = CLIPServer(args.model_name, device=args.device, download_root=args.download_root)
    print(f'CLIP ready on port {args.port}')
    host_model(model, name='clip', port=args.port)
