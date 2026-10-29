"""Detect only the requested target (including its aliases)."""
import cv2
import numpy as np

from vlm.detector.yoloe import YOLOEClient

yoloe_detector = YOLOEClient()


def get_object(right_label, img, cfg):
    """Return annotated RGB, target confidence scores and aligned masks."""
    targets = list(dict.fromkeys(label.strip() for label in right_label.split('|') if label.strip()))
    classes = targets
    annotated = img.copy()
    scores, masks = [], []
    if not classes:
        return annotated, scores, masks

    params = cfg.yoloe
    detections = yoloe_detector.predict(
        img, classes=classes, conf_thres=params.confidence_threshold,
        iou_thres=params.iou_threshold, agnostic_nms=params.agnostic_nms,
    )
    height, width = img.shape[:2]
    for box, score, label, mask in zip(detections.boxes, detections.logits, detections.phrases, detections.masks):
        if label not in targets:
            raise ValueError(f"YOLOE returned an unrequested class: {label}")
        if mask.shape != (height, width):
            raise ValueError(f"YOLOE mask shape {mask.shape} does not match RGB {(height, width)}")
        mask = (mask > 0).astype(np.uint8)
        color = (255, 0, 0)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(annotated, contours, -1, color, 2)
        xyxy = (box * np.array([width, height, width, height])).astype(int)
        xyxy[[0, 2]] = np.clip(xyxy[[0, 2]], 0, width - 1)
        xyxy[[1, 3]] = np.clip(xyxy[[1, 3]], 0, height - 1)
        x1, y1, x2, y2 = xyxy.tolist()
        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
        cv2.putText(annotated, f'{label} ({score:.2f})', (x1, max(y1 - 5, 12)), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
        scores.append(float(score))
        masks.append(mask)
    return annotated, scores, masks
