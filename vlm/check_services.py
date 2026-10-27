"""Smoke-test the running YOLOE and the selected semantic service without ROS or Habitat."""
import argparse

import cv2
import numpy as np

from vlm.detector.yoloe import YOLOEClient
from vlm.utils.get_semantic_similarity import get_semantic_similarity


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', help='Optional image file; otherwise send a blank RGB frame')
    parser.add_argument('--label', default='chair')
    parser.add_argument('--semantic-model', choices=['clip','blip2'], default='clip')
    args = parser.parse_args()
    if args.image:
        image = cv2.imread(args.image)
        if image is None:
            parser.error(f'Cannot read image: {args.image}')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    else:
        image = np.zeros((480, 640, 3), dtype=np.uint8)
    detections = YOLOEClient().predict(image, classes=[args.label])
    for mask in detections.masks:
        if mask.shape != image.shape[:2]:
            raise ValueError(f'Mask shape {mask.shape} does not match RGB {image.shape[:2]}')
    cosine = get_semantic_similarity(image, args.label, 'everywhere', args.semantic_model)
    print(f'YOLOE OK: {len(detections.phrases)} instances, masks aligned with {image.shape[:2]}')
    print(f'{args.semantic_model} OK: cosine={cosine:.6f}')
    print('HTTP contracts passed. A blank image may correctly produce zero detections.')


if __name__ == '__main__':
    main()
