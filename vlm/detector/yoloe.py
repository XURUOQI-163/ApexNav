"""YOLOE instance segmentation service and lightweight HTTP client."""
from __future__ import annotations

import os
import threading
from typing import Any, List, Optional, Tuple

import cv2
import numpy as np

from basic_utils.path_utils import PROJECT_ROOT, WORKSPACE_ROOT, resolve_existing_path
from vlm.detector.detections import ObjectDetections

from ..server_wrapper import (
    ServerMixin,
    bool_arr_to_str,
    host_model,
    send_request,
    str_to_bool_arr,
    str_to_image,
)


YOLOE_WEIGHTS = "data/yoloe-11l-seg.pt"


class YOLOEDetector:
    def __init__(
        self,
        weights: str = YOLOE_WEIGHTS,
        image_size: int = 640,
        device: Optional[str] = None,
    ) -> None:
        import torch
        from ultralytics import YOLOE

        weights = resolve_existing_path(
            weights,
            WORKSPACE_ROOT / "data/yoloe-11l-seg.pt",
            PROJECT_ROOT / "data/yoloe-11l-seg.pt",
        )
        if device is None:
            device = "cuda:0" if torch.cuda.is_available() else "cpu"

        self.weights = weights
        self.image_size = image_size
        self.device = device
        self.model = YOLOE(weights)
        self._predict_lock = threading.Lock()
        self._cached_classes: Tuple[str, ...] = ()
        self._cached_text_pe: Optional[Any] = None
        self._supports_text_pe_cache = hasattr(self.model, "get_text_pe")

    def warmup(self) -> None:
        dummy_image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        self.predict(dummy_image, classes=["chair"])

    def predict(
        self,
        image: np.ndarray,
        classes: List[str],
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        agnostic_nms: bool = True,
    ) -> ObjectDetections:
        import torch

        classes = self._normalize_classes(classes)

        if not classes:
            return self._empty_detections(image)

        with self._predict_lock:
            self._apply_classes(classes)
            with torch.inference_mode():
                result = self.model.predict(
                    source=cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                    conf=conf_thres,
                    iou=iou_thres,
                    agnostic_nms=agnostic_nms,
                    imgsz=self.image_size,
                    device=self.device,
                    verbose=False,
                    retina_masks=True,
                )[0]

            if result.boxes is None or len(result.boxes) == 0:
                return self._empty_detections(image)

            boxes = result.boxes.xyxyn.detach().cpu().float().numpy()
            logits = result.boxes.conf.detach().cpu().float().numpy()
            class_ids = result.boxes.cls.detach().cpu().int().tolist()
            phrases = [result.names[int(class_id)] for class_id in class_ids]
            masks = self._extract_masks(result, image.shape[:2])

        return ObjectDetections(
            boxes=boxes,
            logits=logits,
            phrases=phrases,
            masks=masks,
        )

    @staticmethod
    def _normalize_classes(classes: List[str]) -> Tuple[str, ...]:
        normalized = [label.strip() for label in classes if label and label.strip()]
        return tuple(dict.fromkeys(normalized))

    @staticmethod
    def _empty_detections(image: np.ndarray) -> ObjectDetections:
        return ObjectDetections(
            boxes=np.empty((0, 4), dtype=np.float32),
            logits=np.empty((0,), dtype=np.float32),
            phrases=[],
            masks=[],
        )

    def _apply_classes(self, classes: Tuple[str, ...]) -> None:
        import torch

        classes_list = list(classes)

        # Some YOLOE builds do not keep class prompts reliably across inference calls.
        # Keep applying prompts on every request, but cache the text embeddings so we only
        # recompute them when the class list changes.
        if self._supports_text_pe_cache:
            if self._cached_text_pe is None or classes != self._cached_classes:
                with torch.inference_mode():
                    self._cached_text_pe = self.model.get_text_pe(classes_list)
                self._cached_classes = classes

            try:
                self._set_model_classes(classes_list, self._cached_text_pe)
                return
            except TypeError:
                self._supports_text_pe_cache = False
                self._cached_text_pe = None
                self._cached_classes = ()

        self._set_model_classes(classes_list)

    def _set_model_classes(
        self,
        classes: List[str],
        text_pe: Optional[Any] = None,
    ) -> None:
        self._reset_uninitialized_predictor()
        try:
            if text_pe is None:
                self.model.set_classes(classes)
            else:
                self.model.set_classes(classes, text_pe)
        except AttributeError as exc:
            if "'NoneType' object has no attribute 'names'" not in str(exc):
                raise
            self.model.predictor = None
            if text_pe is None:
                self.model.set_classes(classes)
            else:
                self.model.set_classes(classes, text_pe)

    def _reset_uninitialized_predictor(self) -> None:
        predictor = getattr(self.model, "predictor", None)
        if predictor is not None and getattr(predictor, "model", None) is None:
            self.model.predictor = None

    @staticmethod
    def _extract_masks(result, image_shape: tuple[int, int]) -> List[np.ndarray]:
        if result.masks is None or result.masks.data is None:
            raise ValueError("YOLOE returned boxes without segmentation masks; use a -seg checkpoint")

        mask_data = result.masks.data.detach().cpu().numpy()
        height, width = image_shape
        masks: List[np.ndarray] = []
        for mask in mask_data:
            if mask.shape != (height, width):
                raise ValueError("YOLOE masks must match the input image; retina_masks=True is required")
            masks.append((mask > 0.5).astype(np.uint8))
        return masks


class YOLOEClient:
    def __init__(self, port: int = 12184):
        self.url = os.environ.get("YOLOE_SERVER_URL", f"http://127.0.0.1:{port}/yoloe")

    def predict(
        self,
        image_numpy: np.ndarray,
        classes: List[str],
        agnostic_nms: bool = True,
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
    ) -> ObjectDetections:
        response = send_request(
            self.url,
            image=image_numpy,
            classes=classes,
            agnostic_nms=agnostic_nms,
            conf_thres=conf_thres,
            iou_thres=iou_thres,
        )
        masks = [
            str_to_bool_arr(mask, shape=image_numpy.shape[:2])
            for mask in response["masks"]
        ]
        return ObjectDetections(
            boxes=response["boxes"], logits=response["logits"],
            phrases=response["phrases"], masks=masks,
        )


class YOLOEServer(ServerMixin, YOLOEDetector):
    def process_payload(self, payload: dict) -> dict:
        image = str_to_image(payload["image"])
        detections = self.predict(
            image, classes=payload["classes"],
            agnostic_nms=payload["agnostic_nms"],
            conf_thres=payload["conf_thres"], iou_thres=payload["iou_thres"],
        )
        response = detections.to_json()
        response["masks"] = [bool_arr_to_str(mask) for mask in detections.masks]
        return response


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="YOLOE instance segmentation server")
    parser.add_argument("--port", type=int, default=12184)
    parser.add_argument("--weights", default=os.environ.get("YOLOE_WEIGHTS", YOLOE_WEIGHTS))
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default=None)
    args = parser.parse_args()
    model = YOLOEServer(weights=args.weights, image_size=args.imgsz, device=args.device)
    print("Warming up YOLOE...")
    model.warmup()
    print(f"YOLOE ready on port {args.port}")
    host_model(model, name="yoloe", port=args.port)
