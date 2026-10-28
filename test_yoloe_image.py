"""Detect a text-described object in one image using the existing YOLOE service.

Example: python test_yoloe_image.py --text "black floor lamp"
No ROS, Habitat, BLIP2 or additional model loading is required.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageOps

from vlm.detector.yoloe import YOLOEClient


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path,
                        default=Path('/home/xuyang/nav_repro/6a212fed9148c2419.png_e1080.webp'))
    parser.add_argument('--text', required=True, help='One complete description, e.g. "yellow pillow"')
    parser.add_argument('--output', type=Path, help='Default: <input stem>_yoloe.png')
    parser.add_argument('--conf', type=float, default=.25, help='Detection confidence threshold')
    parser.add_argument('--iou', type=float, default=.45, help='NMS IoU threshold')
    parser.add_argument('--port', type=int, default=12184)
    args = parser.parse_args(argv)
    text = args.text.strip()
    if not text:
        parser.error('--text must not be empty')
    if not 0 <= args.conf <= 1 or not 0 <= args.iou <= 1:
        parser.error('--conf and --iou must be between 0 and 1')
    output = args.output or args.image.with_name(args.image.stem + '_yoloe.png')
    if output.resolve() == args.image.resolve():
        parser.error('--output must differ from the input image')

    with Image.open(args.image) as source:
        rgb = np.array(ImageOps.exif_transpose(source).convert('RGB'))
    client = YOLOEClient(port=args.port)
    print(f'Text: {text}\nService: {client.url}')
    detections = client.predict(rgb, classes=[text], conf_thres=args.conf, iou_thres=args.iou)
    canvas = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    height, width = canvas.shape[:2]
    for index, (box, score, label) in enumerate(zip(
            detections.boxes, detections.logits, detections.phrases), start=1):
        xyxy = np.rint(box * np.array([width, height, width, height])).astype(int)
        xyxy[[0, 2]] = np.clip(xyxy[[0, 2]], 0, width - 1)
        xyxy[[1, 3]] = np.clip(xyxy[[1, 3]], 0, height - 1)
        x1, y1, x2, y2 = xyxy.tolist()
        cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 255, 0), 3)
        caption = f'{label} {score:.3f}'
        origin = (x1, max(24, y1 - 8))
        cv2.putText(canvas, caption, origin, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 4)
        cv2.putText(canvas, caption, origin, cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 2)
        print(f'{index}: {label}, confidence={score:.4f}, xyxy={xyxy.tolist()}')

    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), canvas):
        raise RuntimeError(f'Could not save output image: {output}')
    count = len(detections.phrases)
    print(f'Detections: {count}\nSaved: {output.resolve()}')
    if count == 0:
        print('No matching detections; saved the image without boxes.')


if __name__ == '__main__':
    main()
