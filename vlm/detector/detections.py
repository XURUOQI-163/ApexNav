"""YOLOE output contract: normalized XYXY boxes and full-resolution binary masks."""
from dataclasses import dataclass
from typing import List

import numpy as np


@dataclass
class ObjectDetections:
    boxes: np.ndarray
    logits: np.ndarray
    phrases: List[str]
    masks: List[np.ndarray]

    def __post_init__(self):
        self.boxes = np.asarray(self.boxes, dtype=np.float32).reshape(-1, 4)
        self.logits = np.asarray(self.logits, dtype=np.float32).reshape(-1)
        count = len(self.phrases)
        if not (len(self.boxes) == len(self.logits) == len(self.masks) == count):
            raise ValueError("YOLOE boxes, scores, labels and masks must have equal lengths")
        if not np.isfinite(self.boxes).all() or not np.isfinite(self.logits).all():
            raise ValueError("YOLOE boxes and scores must be finite")
        for mask in self.masks:
            if not isinstance(mask, np.ndarray) or mask.ndim != 2:
                raise ValueError("YOLOE must return a 2D segmentation mask for every detection")

    def to_json(self):
        # Masks are encoded by the HTTP server separately.
        return dict(boxes=self.boxes.tolist(), logits=self.logits.tolist(), phrases=self.phrases)
