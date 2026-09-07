"""Optional YOLO backend; imported without requiring torch/ultralytics."""
from pathlib import Path
import cv2
import numpy as np


def load_model(path):
    if not Path(path).is_file():
        raise ValueError('Select an existing shape-trained YOLO .pt file.')
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise RuntimeError('Ultralytics is missing. Follow docs/OBJECT_FEATURE_DETECTOR_ID.md to install the optional YOLO environment.') from exc
    model = YOLO(path)
    if model.task not in ('detect', 'segment'):
        raise ValueError('Use a detection or instance segmentation model.')
    return model


def predict(model, frame, confidence):
    result = model.predict(frame, imgsz=320, conf=confidence, device='cpu', verbose=False, max_det=100)[0]
    detections = []
    for i, box in enumerate(result.boxes):
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = max(0,x1), max(0,y1), min(w,x2), min(h,y2)
        if x2 <= x1 or y2 <= y1:
            continue
        mask = None
        if result.masks is not None:
            mask = np.zeros((h,w), np.uint8)
            cv2.fillPoly(mask, [result.masks.xy[i].astype(np.int32)], 255)
            mask = mask[y1:y2, x1:x2]
        detections.append(dict(shape=result.names[int(box.cls.item())], confidence=float(box.conf.item()),
                               bbox=(x1,y1,x2-x1,y2-y1), mask=mask))
    return frame, detections


def match_objects(shape, colors, rules):
    """First enabled rule wins; required colors must all occur on one detection."""
    for rule in rules:
        if rule['enabled'] and rule['shape'] in ('Any', shape) and set(rule['colors']).issubset(colors):
            return rule
    return None


def validate_objects(rules, formatter):
    if not isinstance(rules, list):
        raise ValueError('Invalid object list')
    names = set()
    for rule in rules:
        if not isinstance(rule['name'], str) or not rule['name'].strip() or rule['name'] in names:
            raise ValueError('Object names must be unique and non-empty')
        names.add(rule['name'])
        if not isinstance(rule['shape'], str) or not rule['shape'].strip():
            raise ValueError('Select a shape class')
        if not isinstance(rule['colors'], list) or not all(isinstance(c,str) for c in rule['colors']):
            raise ValueError('Invalid color selection')
        if type(rule['enabled']) is not bool:
            raise ValueError('Invalid enabled state')
        if rule['label']:
            formatter(rule['label'])
