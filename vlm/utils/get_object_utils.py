"""Convert YOLOE instances into aligned masks, scores and planner label indices."""
import cv2
import numpy as np

from vlm.detector.yoloe import YOLOEClient

yoloe_detector = YOLOEClient()


def get_object(right_label, img, cfg, similar_answer):
    """Return annotated RGB, confidence scores, masks, and labels (0 = target).

    Target aliases share index 0. Each distinct related class gets a positive
    index. The three output lists always have the same order and length.
    """
    targets = list(dict.fromkeys(label.strip() for label in right_label.split('|') if label.strip()))
    related = list(dict.fromkeys(label.strip() for label in similar_answer if label.strip() and label.strip() not in targets))
    classes = targets + related
    annotated = img.copy()
    scores, masks, indices = [], [], []
    if not classes:
        return annotated, scores, masks, indices

    params = cfg.yoloe
    detections = yoloe_detector.predict(
        img, classes=classes, conf_thres=params.confidence_threshold,
        iou_thres=params.iou_threshold, agnostic_nms=params.agnostic_nms,
    )
    label_indices = {label: 0 for label in targets}
    label_indices.update({label: i + 1 for i, label in enumerate(related)})
    height, width = img.shape[:2]
    for box, score, label, mask in zip(detections.boxes, detections.logits, detections.phrases, detections.masks):
        if label not in label_indices:
            raise ValueError(f"YOLOE returned an unrequested class: {label}")
        if mask.shape != (height, width):
            raise ValueError(f"YOLOE mask shape {mask.shape} does not match RGB {(height, width)}")
        mask = (mask > 0).astype(np.uint8)
        index = label_indices[label]
        color = (255, 0, 0) if index == 0 else (0, 255, 0)
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
        indices.append(index)
    return annotated, scores, masks, indices
