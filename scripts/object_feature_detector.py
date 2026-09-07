#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Raspberry Pi color-profile detector with an English Tkinter GUI.

Teach a named HSV profile from a rectangular color sample, then detect it
across the entire image. Profiles support editing, resampling, enable/disable,
and JSON persistence. Shape, current count, and barcode are optional features.
"""

import cv2
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
import time
import argparse
from concurrent.futures import ThreadPoolExecutor
import sys
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import vision_yolo
import json
import string
from pathlib import Path
from PIL import Image, ImageTk

# ------------------------------------------------------------
# Optional barcode backends
# ------------------------------------------------------------
HAS_CV_BARCODE = hasattr(cv2, "barcode") and hasattr(cv2.barcode, "BarcodeDetector")

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    HAS_PYZBAR = True
except Exception:
    HAS_PYZBAR = False


# ------------------------------------------------------------
# Utility
# ------------------------------------------------------------
def resize_keep_ratio(frame, max_w=900, max_h=600):
    h, w = frame.shape[:2]
    scale = min(max_w / w, max_h / h, 1.0)
    if scale != 1.0:
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return frame


def classify_color(roi, mask=None):
    """Estimate dominant object color using HSV.
    Returns one of: Red, Orange, Yellow, Green, Cyan, Blue, Purple,
    White, Gray, Black, Unknown.
    """
    if roi is None or roi.size == 0:
        return "Unknown"

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    if mask is not None:
        h, s, v = h[mask > 0], s[mask > 0], v[mask > 0]
        if v.size == 0:
            return "Unknown"

    # Ignore very dark pixels and very low saturation pixels when
    # estimating chromatic color.
    valid = v > 45
    if np.count_nonzero(valid) < 20:
        return "Black"

    mean_s = float(np.mean(s[valid]))
    mean_v = float(np.mean(v[valid]))

    # White / gray / black
    if mean_v < 55:
        return "Black"
    if mean_s < 35:
        if mean_v > 180:
            return "White"
        return "Gray"

    hues = h[valid & (s > 50)]
    histogram = np.bincount(hues, minlength=180)
    hue = float(np.argmax(histogram)) if hues.size else 0

    if hue < 10 or hue >= 170:
        return "Red"
    if hue < 22:
        return "Orange"
    if hue < 35:
        return "Yellow"
    if hue < 85:
        return "Green"
    if hue < 100:
        return "Cyan"
    if hue < 135:
        return "Blue"
    if hue < 170:
        return "Purple"

    return "Unknown"


def shape_from_contour(cnt):
    """Simple 2D shape classification from contour geometry."""
    area = cv2.contourArea(cnt)
    if area < 500:
        return None

    perimeter = cv2.arcLength(cnt, True)
    if perimeter <= 0:
        return None

    approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
    x, y, w, h = cv2.boundingRect(cnt)

    if h == 0:
        return None

    aspect = w / float(h)
    circularity = 4.0 * np.pi * area / (perimeter * perimeter)

    vertices = len(approx)

    if vertices == 3:
        return "Triangle"

    if vertices == 4:
        # Distinguish square-ish vs rectangle
        if 0.85 <= aspect <= 1.15:
            return "Square"
        return "Rectangle"

    if circularity > 0.78:
        return "Circle"

    # Ellipse / round-ish objects
    if vertices >= 5 and circularity > 0.55:
        return "Oval / Round"

    return "Irregular"


def find_objects(frame, bg_subtractor, min_area=1200):
    """Find foreground objects for shape/count.
    Camera should be fixed.
    """
    fg = bg_subtractor.apply(frame, learningRate=0)

    # Clean mask
    kernel = np.ones((5, 5), np.uint8)
    fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kernel)
    fg = cv2.morphologyEx(fg, cv2.MORPH_CLOSE, kernel)
    fg = cv2.dilate(fg, kernel, iterations=1)

    contours, _ = cv2.findContours(fg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    objects = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        objects.append({
            "contour": cnt,
            "bbox": (x, y, w, h),
            "area": area,
        })

    # Stable order from left to right
    objects.sort(key=lambda o: o["bbox"][0])
    return objects, fg


# ------------------------------------------------------------
# Barcode
# ------------------------------------------------------------
class BarcodeReader:
    def __init__(self):
        self.cv_detector = None
        if HAS_CV_BARCODE:
            try:
                self.cv_detector = cv2.barcode.BarcodeDetector()
            except Exception:
                self.cv_detector = None

    def read(self, frame):
        results = []

        # OpenCV BarcodeDetector
        if self.cv_detector is not None:
            try:
                out = self.cv_detector.detectAndDecodeWithType(frame)

                # OpenCV versions may return different tuple forms.
                if isinstance(out, tuple) and len(out) >= 3:
                    ok = out[0]
                    decoded_info = out[1]
                    decoded_type = out[2]
                    points = out[3] if len(out) > 3 else None

                    if ok and decoded_info:
                        for i, value in enumerate(decoded_info):
                            if value:
                                btype = decoded_type[i] if i < len(decoded_type) else "UNKNOWN"
                                poly = None
                                if points is not None:
                                    try:
                                        poly = points[i]
                                    except Exception:
                                        poly = None
                                results.append((str(value), str(btype), poly))
            except Exception:
                pass

        # pyzbar fallback / additional decoder
        if HAS_PYZBAR:
            try:
                decoded = pyzbar_decode(frame)
                for item in decoded:
                    value = item.data.decode("utf-8", errors="replace")
                    btype = item.type
                    r = item.rect
                    poly = np.array([
                        [r.left, r.top],
                        [r.left + r.width, r.top],
                        [r.left + r.width, r.top + r.height],
                        [r.left, r.top + r.height]
                    ], dtype=np.int32)
                    results.append((value, btype, poly))
            except Exception:
                pass

        # Remove duplicates
        unique = []
        seen = set()
        for item in results:
            key = (item[0], item[1])
            if key not in seen:
                seen.add(key)
                unique.append(item)

        return unique


DEFAULT_LABEL = "{roi} | OBJ-{id:03d} | {color} | {shape}"
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "object_feature_profiles.json"
LABEL_VALUES = dict(name="Object", confidence=0.9, colors="Red, Blue", roi="Area", id=1, color="Red", shape="Square", count=2,
                    x=10, y=20, w=30, h=40, barcode="12345", type="EAN13")


def format_label(template, **values):
    if len(template) > 300:
        raise ValueError("Use at most 300 characters in the label format")
    for _, field, spec, conversion in string.Formatter().parse(template):
        if field is not None and (field not in LABEL_VALUES or
                                  "{" in spec or "}" in spec or len(spec) > 10):
            raise ValueError("Unknown placeholder or invalid format")
        if spec and any(c.isdigit() for c in spec):
            import re
            if any(int(n) > 100 for n in re.findall(r"\d+", spec)):
                raise ValueError("Maximum format width is 100")
    return template.format(**(LABEL_VALUES | values))


def learn_hsv(sample):
    pixels = cv2.cvtColor(sample, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    useful = pixels[(pixels[:, 1] > 25) & (pixels[:, 2] > 30)]
    if len(useful) < 20:
        useful = pixels
    # Unwrap hue around its circular mean, including the red 179/0 boundary.
    hues = useful[:, 0].astype(float)
    angles = hues * np.pi / 90
    center = np.arctan2(np.sin(angles).mean(), np.cos(angles).mean()) * 90 / np.pi
    unwrapped = center + (hues - center + 90) % 180 - 90
    lo, hi = np.percentile(unwrapped, [5, 95]) + [-8, 8]
    low_sv = np.clip(np.percentile(useful[:, 1:], 5, axis=0) - [25, 25], 0, 255).astype(int)
    high_sv = np.clip(np.percentile(useful[:, 1:], 95, axis=0) + [25, 25], 0, 255).astype(int)
    low_h, high_h = (0, 179) if hi-lo >= 179 else (int(lo) % 180, int(hi) % 180)
    return [low_h, *map(int, low_sv)], [high_h, *map(int, high_sv)]


def profile_mask(hsv, profile):
    low, high = np.array(profile["low"], np.uint8), np.array(profile["high"], np.uint8)
    if low[0] <= high[0]:
        return cv2.inRange(hsv, low, high)
    return cv2.bitwise_or(cv2.inRange(hsv, low, np.array([179, high[1], high[2]], np.uint8)),
                         cv2.inRange(hsv, np.array([0, low[1], low[2]], np.uint8), high))


def validate_profiles(profiles):
    if not isinstance(profiles, list):
        raise ValueError("Invalid profile list")
    names = set()
    for profile in profiles:
        name = profile["name"]
        if not isinstance(name, str) or not name.strip() or name in names:
            raise ValueError("Each profile needs a unique, non-empty name")
        names.add(name)
        for key in ("low", "high"):
            values = profile[key]
            if len(values) != 3 or any(type(v) is not int or not 0 <= v <= limit
                                      for v, limit in zip(values, (179, 255, 255))):
                raise ValueError("HSV limits: H 0-179, S/V 0-255")
        if any(profile["low"][i] > profile["high"][i] for i in (1, 2)):
            raise ValueError("S/V minimum must not exceed maximum")
        if type(profile["enabled"]) is not bool:
            raise ValueError("Invalid enabled state")


def detect_profiles(frame, profiles, min_area):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    found = []
    kernel = np.ones((5, 5), np.uint8)
    # Priority follows list order: overlapping HSV ranges cannot double-count pixels.
    claimed = np.zeros(frame.shape[:2], np.uint8)
    for profile in profiles:
        if not profile["enabled"]:
            continue
        mask = profile_mask(hsv, profile)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.bitwise_and(mask, cv2.bitwise_not(claimed))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in sorted(contours, key=lambda c: cv2.boundingRect(c)[0]):
            if cv2.contourArea(cnt) >= min_area:
                found.append((profile["name"], cnt))
                cv2.drawContours(claimed, [cnt], -1, 255, -1)
    return found


def roi_pixels(roi, width, height):
    x1, y1, x2, y2 = roi["rect"]
    return (min(width-1, int(x1*width)), min(height-1, int(y1*height)),
            min(width, max(1, int(x2*width))), min(height, max(1, int(y2*height))))


# ------------------------------------------------------------
# Main application
# ------------------------------------------------------------
class App:
    def __init__(self, root, camera=0):
        self.after_id = None
        self.read_failures = 0
        self.root = root
        self.root.title("Raspberry Pi Object Feature Detector")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self.running = False
        self.cap = None

        # Feature switches
        self.color_var = tk.BooleanVar(value=True)
        self.shape_var = tk.BooleanVar(value=False)
        self.count_var = tk.BooleanVar(value=False)
        self.barcode_var = tk.BooleanVar(value=False)

        # Camera settings
        self.camera_index = tk.IntVar(value=camera)
        self.min_area_var = tk.IntVar(value=1200)

        # Counters
        self.last_count = 0
        self.last_barcode = "-"
        self.last_barcode_type = "-"

        self.barcode_reader = BarcodeReader()

        self.rois = []
        self.latest_frame = None
        self.sample_frame = None
        self.resample_index = None
        self.drawing_roi = False
        self.drag_start = self.drag_end = None
        self.display_size = None
        self.label_template = DEFAULT_LABEL
        self.label_var = tk.StringVar(value=DEFAULT_LABEL)
        self.object_rules = []
        self.model = None
        self.model_path = ""
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.yolo_future = None
        self.generation = 0
        self.future_generation = 0
        self.yolo_result = None
        self.yolo_var = tk.BooleanVar(value=False)
        self.confidence_var = tk.StringVar(value="0.50")
        self.build_gui()
        self.load_settings()

        if not HAS_CV_BARCODE and not HAS_PYZBAR:
            self.barcode_status.config(
                text="Barcode backend: unavailable (install pyzbar/libzbar)"
            )
        elif HAS_CV_BARCODE:
            self.barcode_status.config(text="Barcode backend: OpenCV BarcodeDetector")
        else:
            self.barcode_status.config(text="Barcode backend: pyzbar")

    def build_gui(self):
        main = ttk.Frame(self.root, padding=10)
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(0, weight=1)
        main.rowconfigure(2, weight=1)

        title = ttk.Label(
            main,
            text="OBJECT DETECTION SYSTEM - Raspberry Pi",
            font=("TkDefaultFont", 15, "bold")
        )
        title.grid(row=0, column=0, sticky="w", pady=(0, 8))

        controls = ttk.LabelFrame(main, text="Feature Select", padding=8)
        controls.grid(row=1, column=0, sticky="ew")
        controls.columnconfigure(10, weight=1)

        ttk.Checkbutton(controls, text="Color", variable=self.color_var).grid(
            row=0, column=0, padx=5
        )
        ttk.Checkbutton(controls, text="Shape", variable=self.shape_var).grid(
            row=0, column=1, padx=5
        )
        ttk.Checkbutton(controls, text="Count", variable=self.count_var).grid(
            row=0, column=2, padx=5
        )
        ttk.Checkbutton(controls, text="Barcode", variable=self.barcode_var).grid(
            row=0, column=3, padx=5
        )

        ttk.Label(controls, text="Camera:").grid(row=0, column=5, padx=(20, 4))
        ttk.Spinbox(
            controls, from_=0, to=5, width=4, textvariable=self.camera_index
        ).grid(row=0, column=6)

        ttk.Label(controls, text="Min area:").grid(row=0, column=7, padx=(12, 4))
        ttk.Spinbox(
            controls, from_=300, to=50000, increment=100,
            width=7, textvariable=self.min_area_var
        ).grid(row=0, column=8)

        self.start_btn = ttk.Button(
            controls, text="START", command=self.start_camera
        )
        self.start_btn.grid(row=0, column=9, padx=(15, 4))

        self.stop_btn = ttk.Button(
            controls, text="STOP", command=self.stop_camera
        )
        self.stop_btn.grid(row=0, column=10, padx=4)



        roi_controls = ttk.Frame(controls)
        roi_controls.grid(row=2, column=0, columnspan=11, sticky="ew")
        ttk.Button(roi_controls, text="Add Color Sample (ROI)", command=self.begin_roi).pack(side="left")
        ttk.Button(roi_controls, text="Profiles / Edit", command=self.show_profiles).pack(side="left", padx=4)
        ttk.Button(roi_controls, text="Save Settings", command=self.save_settings).pack(side="left", padx=4)
        ttk.Button(roi_controls, text="Add Object", command=self.add_object).pack(side="left", padx=4)
        ttk.Button(roi_controls, text="Objects / Edit", command=self.show_objects).pack(side="left", padx=4)
        yolo_controls = ttk.Frame(controls)
        yolo_controls.grid(row=1, column=0, columnspan=11, sticky="ew", pady=4)
        ttk.Checkbutton(yolo_controls, text="YOLO Shapes", variable=self.yolo_var, command=self.toggle_yolo).pack(side="left")
        ttk.Button(yolo_controls, text="Load Shape Model", command=self.choose_model).pack(side="left", padx=4)
        ttk.Label(yolo_controls, text="Confidence:").pack(side="left")
        ttk.Spinbox(yolo_controls, from_=0.05, to=1, increment=0.05, textvariable=self.confidence_var, width=5).pack(side="left")
        self.model_status = ttk.Label(yolo_controls, text="No model loaded")
        self.model_status.pack(side="left", padx=4)
        label_controls = ttk.Frame(controls)
        label_controls.grid(row=3, column=0, columnspan=11, sticky="ew", pady=4)
        ttk.Label(label_controls, text="Box label:").pack(side="left")
        ttk.Entry(label_controls, textvariable=self.label_var, width=60).pack(side="left", fill="x", expand=True)
        ttk.Button(label_controls, text="Apply", command=self.apply_label).pack(side="left")
        ttk.Label(controls, text="Placeholder: {name} {confidence:.2f} {colors} {roi} {id:03d} {color} {shape} {count} {x} {y} {w} {h} {barcode} {type}").grid(row=4, column=0, columnspan=11, sticky="w")

        self.video_label = ttk.Label(main, text="Camera stopped", anchor="center")
        self.video_label.grid(row=2, column=0, sticky="nsew", pady=8)

        self.video_label.bind("<ButtonPress-1>", self.roi_press)
        self.video_label.bind("<B1-Motion>", self.roi_motion)
        self.video_label.bind("<ButtonRelease-1>", self.roi_release)
        self.root.bind("<Escape>", lambda event: self.cancel_roi())

        result = ttk.LabelFrame(main, text="Result", padding=8)
        result.grid(row=3, column=0, sticky="ew")

        self.result_text = tk.Text(result, height=7, width=90, font=("Courier", 10))
        self.result_text.pack(fill="both", expand=True)

        self.barcode_status = ttk.Label(main, text="")
        self.barcode_status.grid(row=4, column=0, sticky="w", pady=(5, 0))

        self.info = ttk.Label(
            main,
            text="START > Add Color Sample > drag on object > name it. Enable profiles to detect across the full image."
        )
        self.info.grid(row=5, column=0, sticky="w")

    def toggle_yolo(self):
        self.generation += 1
        self.yolo_result = None
        if self.yolo_var.get() and self.model is None:
            self.yolo_var.set(False)
            messagebox.showinfo("YOLO Shapes", "Load a shape-trained model first. Add Object creates a matching rule; it does not train YOLO.")

    def choose_model(self):
        if self.yolo_future is not None and self.yolo_future.done():
            self.yolo_future = None
        if self.yolo_future is not None:
            messagebox.showinfo("Model", "Stop the camera and wait for the current inference to finish before loading a model.")
            return
        path = filedialog.askopenfilename(title="Select shape-trained YOLO weights", filetypes=[("YOLO weights", "*.pt")])
        if not path:
            return
        self.yolo_var.set(False)
        self.yolo_result = None
        self.model_status.config(text="Loading model...")
        future = self.executor.submit(vision_yolo.load_model, path)
        self.yolo_future = future
        def poll():
            if not future.done():
                self.root.after(100, poll)
                return
            self.yolo_future = None
            try:
                self.model = future.result()
                self.model_path = path
                self.model_status.config(text=Path(path).name)
                self.yolo_var.set(True)
                self.save_settings(False)
                names = self.model.names
                messagebox.showinfo("Model classes", ", ".join(names.values() if isinstance(names, dict) else names))
            except Exception as exc:
                self.model_status.config(text="Model load failed")
                messagebox.showerror("Model load failed", str(exc))
        poll()

    def add_object(self):
        self.edit_object()

    def show_objects(self):
        window = tk.Toplevel(self.root)
        window.title("Objects - Select to Edit")
        listing = tk.Listbox(window, width=85, height=15)
        listing.pack(fill="both", expand=True)
        for rule in self.object_rules:
            listing.insert(tk.END, f"{'[x]' if rule['enabled'] else '[ ]'} {rule['name']} | {rule['shape']} | {', '.join(rule['colors']) or 'Any color'}")
        def edit():
            if listing.curselection():
                index = listing.curselection()[0]
                window.destroy()
                self.edit_object(index)
        def delete():
            if listing.curselection() and messagebox.askyesno("Delete object", "Delete the selected object rule?", parent=window):
                del self.object_rules[listing.curselection()[0]]
                self.save_settings(False)
                window.destroy()
                self.show_objects()
        ttk.Button(window, text="Edit / Enable", command=edit).pack(side="left")
        ttk.Button(window, text="Delete", command=delete).pack(side="left")

    def edit_object(self, index=None):
        rule = self.object_rules[index] if index is not None else dict(name="", shape="Any", colors=[], label="{name} | {shape} | {colors} | {confidence:.2f}", enabled=True)
        dialog = tk.Toplevel(self.root)
        dialog.title("Edit Object" if index is not None else "Add Object")
        dialog.transient(self.root)
        dialog.grab_set()
        variables = {key: tk.StringVar(value=rule[key]) for key in ('name', 'shape', 'label')}
        names = self.model.names if self.model is not None else {}
        classes = list(names.values()) if isinstance(names, dict) else list(names)
        for row, key in enumerate(('name', 'shape', 'label')):
            ttk.Label(dialog, text={'name':'Object name', 'shape':'YOLO class (exact name)', 'label':'Custom label (blank = global)'}[key]).grid(row=row, column=0, padx=8, pady=6)
            if key == 'shape':
                ttk.Combobox(dialog, textvariable=variables[key], values=['Any'] + classes, width=48).grid(row=row, column=1)
            else:
                ttk.Entry(dialog, textvariable=variables[key], width=50).grid(row=row, column=1)
        ttk.Label(dialog, text="Required colors (select multiple; none = any):").grid(row=3, column=0, columnspan=2)
        colors = tk.Listbox(dialog, selectmode='multiple', exportselection=False, height=8)
        colors.grid(row=4, column=0, columnspan=2, sticky='ew', padx=8)
        color_names = list(dict.fromkeys([p['name'] for p in self.rois] + rule['colors']))
        for i, name in enumerate(color_names):
            colors.insert(tk.END, name)
            if name in rule['colors']:
                colors.selection_set(i)
        enabled = tk.BooleanVar(value=rule['enabled'])
        ttk.Checkbutton(dialog, text="Enabled", variable=enabled).grid(row=5, column=0)
        ttk.Label(dialog, text="All selected colors must match. First matching object rule wins.\nThis does not train a new shape class.").grid(row=6, column=0, columnspan=2, padx=8, pady=8)
        def save():
            item = {key: value.get().strip() for key, value in variables.items()}
            item.update(enabled=enabled.get(), colors=[color_names[i] for i in colors.curselection()])
            proposed = list(self.object_rules)
            if index is None:
                proposed.append(item)
            else:
                proposed[index] = item
            try:
                vision_yolo.validate_objects(proposed, format_label)
                if classes and item['shape'] not in ['Any'] + classes:
                    raise ValueError('Shape class is not present in the loaded model')
            except (ValueError, TypeError, KeyError) as exc:
                messagebox.showerror("Invalid object", str(exc), parent=dialog)
                return
            self.object_rules = proposed
            self.save_settings(False)
            dialog.destroy()
        ttk.Button(dialog, text="Save", command=save).grid(row=7, column=0, pady=8)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).grid(row=7, column=1)

    def yolo_detections(self, frame):
        if self.yolo_future is not None and self.yolo_future.done():
            try:
                result = self.yolo_future.result()
                if self.future_generation == self.generation:
                    self.yolo_result = result
            except Exception as exc:
                self.yolo_var.set(False)
                self.yolo_result = None
                messagebox.showerror("YOLO inference failed", str(exc))
            self.yolo_future = None
        if self.yolo_future is None and self.yolo_var.get() and not self.drawing_roi:
            try:
                confidence = float(self.confidence_var.get())
                if not 0 < confidence <= 1:
                    raise ValueError()
            except ValueError:
                confidence = 0.5
            self.future_generation = self.generation
            self.yolo_future = self.executor.submit(vision_yolo.predict, self.model, frame.copy(), confidence)
        return self.yolo_result or (frame, [])

    def draw_yolo(self, frame, detections, display, use_color, use_shape, use_count):
        records = []
        for detection in detections:
            x,y,w,h = detection['bbox']
            if w*h < self.valid_min_area():
                continue
            crop = frame[y:y+h, x:x+w]
            hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
            support = detection['mask']
            support = np.full((h,w), 255, np.uint8) if support is None else support
            denominator = max(1, np.count_nonzero(support))
            colors = [p['name'] for p in self.rois if p['enabled'] and
                      np.count_nonzero(cv2.bitwise_and(profile_mask(hsv,p),support))/denominator >= 0.08]
            rule = vision_yolo.match_objects(detection['shape'], colors, self.object_rules)
            records.append((detection, colors, rule))
        lines = [f"YOLO objects: {len(records)}"] if use_count else []
        for i, (det, colors, rule) in enumerate(records, 1):
            x,y,w,h = det['bbox']
            name = rule['name'] if rule else 'Unknown'
            template = rule['label'] if rule and rule['label'] else self.label_template
            color_text = ', '.join(colors) or 'Unknown'
            label = format_label(template, name=name, roi=name, id=i, shape=det['shape'] if use_shape else '-',
                                 colors=color_text if use_color else '-', color=color_text if use_color else '-',
                                 confidence=det['confidence'], count=len(records), x=x,y=y,w=w,h=h,barcode='-',type='-')
            cv2.rectangle(display,(x,y),(x+w,y+h),(0,255,0),2)
            cv2.putText(display,label,(x,max(20,y-8)),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1,cv2.LINE_AA)
            lines.append(label)
        return lines

    def show_profiles(self):
        if hasattr(self, "profile_window") and self.profile_window.winfo_exists():
            self.profile_window.lift()
            return
        self.profile_window = tk.Toplevel(self.root)
        self.profile_window.title("Color Profiles - Enable / Edit / Resample")
        self.profile_window.geometry("760x520")
        body = ttk.Frame(self.profile_window, padding=8)
        body.pack(fill="both", expand=True)
        self.profile_tree = ttk.Treeview(body, columns=("enabled", "name", "low", "high"), show="headings", height=15, selectmode="browse")
        for key, title, width in (("enabled", "Enabled", 65), ("name", "Object Name", 230), ("low", "HSV Min", 160), ("high", "HSV Max", 160)):
            self.profile_tree.heading(key, text=title)
            self.profile_tree.column(key, width=width)
        scroll = ttk.Scrollbar(body, orient="vertical", command=self.profile_tree.yview)
        self.profile_tree.configure(yscrollcommand=scroll.set)
        self.profile_tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.profile_tree.bind("<ButtonRelease-1>", self.profile_click)
        buttons = ttk.Frame(self.profile_window, padding=8)
        buttons.pack(fill="x")
        for title, command in (("Edit", self.rename_roi), ("Resample", self.resample_roi),
                               ("Delete", self.delete_roi), ("Enable All", lambda: self.enable_all(True)),
                               ("Disable All", lambda: self.enable_all(False))):
            ttk.Button(buttons, text=title, command=command).pack(side="left", padx=3)
        ttk.Label(self.profile_window, text="Click [x]/[ ] to toggle. Changes save automatically. Earlier profiles have priority for overlapping colors.", wraplength=730).pack(padx=8, pady=8)
        self.refresh_rois()

    def refresh_rois(self):
        if not hasattr(self, "profile_tree") or not self.profile_tree.winfo_exists():
            return
        self.profile_tree.delete(*self.profile_tree.get_children())
        for i, profile in enumerate(self.rois):
            self.profile_tree.insert("", "end", iid=str(i), values=("[x]" if profile["enabled"] else "[ ]", profile["name"], str(profile["low"]), str(profile["high"])))

    def selected_profile(self):
        if hasattr(self, "profile_tree") and self.profile_tree.winfo_exists():
            selected = self.profile_tree.selection()
            if selected:
                return int(selected[0])
        return -1

    def profile_click(self, event):
        row = self.profile_tree.identify_row(event.y)
        if row and self.profile_tree.identify_column(event.x) == "#1":
            index = int(row)
            self.rois[index]["enabled"] = not self.rois[index]["enabled"]
            self.refresh_rois()
            self.save_settings(False)

    def enable_all(self, enabled):
        for profile in self.rois:
            profile["enabled"] = enabled
        self.refresh_rois()
        self.save_settings(False)

    def resample_roi(self):
        index = self.selected_profile()
        if index >= 0:
            self.begin_roi()
            if self.drawing_roi:
                self.resample_index = index
                self.root.lift()

    def apply_label(self):
        try:
            preview = format_label(self.label_var.get())
        except (ValueError, KeyError, IndexError, TypeError) as exc:
            messagebox.showerror("Label format", str(exc))
            return False
        self.label_template = self.label_var.get()
        self.info.config(text="Label preview: " + preview)
        return True

    def save_settings(self, apply=True):
        if apply and not self.apply_label():
            return
        try:
            CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
            temporary = CONFIG_PATH.with_suffix(".tmp")
            temporary.write_text(json.dumps(dict(version=3, profiles=self.rois, objects=self.object_rules, model_path=self.model_path, label=self.label_template), indent=2), encoding="utf-8")
            temporary.replace(CONFIG_PATH)
            self.info.config(text="Color profiles and label settings saved: " + str(CONFIG_PATH))
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))

    def load_settings(self):
        if not CONFIG_PATH.exists():
            return
        try:
            data = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
            if data.get("version") not in (2, 3):
                raise ValueError("Unsupported settings version")
            rois = data["profiles"]
            validate_profiles(rois)
            format_label(data["label"])
            rules = data.get("objects", [])
            vision_yolo.validate_objects(rules, format_label)
            self.object_rules = rules
            self.model_path = data.get("model_path", "")
            if self.model_path:
                self.model_status.config(text="Saved model: " + Path(self.model_path).name + " (load to use)")
            self.rois = rois
            self.label_template = data["label"]
            self.label_var.set(self.label_template)
            self.refresh_rois()
        except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
            messagebox.showwarning("Settings could not be loaded", str(exc))

    def begin_roi(self):
        if not self.running or self.display_size is None:
            messagebox.showinfo("ROI", "Click START and wait for the camera image first.")
            return
        self.resample_index = None
        self.sample_frame = self.latest_frame.copy()
        self.drawing_roi = True
        self.info.config(text="Drag a rectangle on the camera image, then enter a name. Esc cancels.")

    def cancel_roi(self):
        self.drawing_roi = False
        self.sample_frame = None
        self.drag_start = self.drag_end = None

    def image_point(self, event):
        if self.display_size is None:
            return None
        w, h = self.display_size
        x = event.x - (self.video_label.winfo_width() - w) / 2
        y = event.y - (self.video_label.winfo_height() - h) / 2
        return (max(0, min(1, x/w)), max(0, min(1, y/h)))

    def roi_press(self, event):
        if self.drawing_roi:
            self.drag_start = self.drag_end = self.image_point(event)

    def roi_motion(self, event):
        if self.drawing_roi and self.drag_start:
            self.drag_end = self.image_point(event)

    def roi_release(self, event):
        if not self.drawing_roi or self.drag_start is None:
            return
        end = self.image_point(event)
        x1, x2 = sorted((self.drag_start[0], end[0]))
        y1, y2 = sorted((self.drag_start[1], end[1]))
        sample_frame = self.sample_frame
        index = self.resample_index
        self.cancel_roi()
        if (x2-x1)*self.display_size[0] < 10 or (y2-y1)*self.display_size[1] < 10:
            self.info.config(text="Sample is too small. Draw at least 10 x 10 pixels.")
            return
        x, y, ex, ey = roi_pixels(dict(rect=[x1, y1, x2, y2]), sample_frame.shape[1], sample_frame.shape[0])
        low, high = learn_hsv(sample_frame[y:ey, x:ex])
        if index is not None:
            self.rois[index].update(low=low, high=high)
            self.refresh_rois()
            self.save_settings(False)
        else:
            self.edit_profile(dict(name="", low=low, high=high, enabled=True))

    def rename_roi(self):
        index = self.selected_profile()
        if index >= 0:
            self.edit_profile(dict(self.rois[index]), index)

    def edit_profile(self, profile, index=None):
        dialog = tk.Toplevel(self.root)
        dialog.title("Edit Color Profile" if index is not None else "New Color Profile")
        dialog.transient(self.root)
        dialog.grab_set()
        name = tk.StringVar(value=profile["name"])
        enabled = tk.BooleanVar(value=profile["enabled"])
        ttk.Label(dialog, text="Object name:").grid(row=0, column=0, padx=8, pady=8)
        entry = ttk.Entry(dialog, textvariable=name, width=32)
        entry.grid(row=0, column=1, columnspan=2, padx=8)
        entry.focus_set()
        variables = {}
        for row, channel in enumerate(("H", "S", "V"), 2):
            ttk.Label(dialog, text=channel + " min / max:").grid(row=row, column=0, padx=8)
            for col, key in enumerate(("low", "high"), 1):
                var = tk.StringVar(value=str(profile[key][row-2]))
                variables[key, row-2] = var
                ttk.Spinbox(dialog, from_=0, to=179 if channel == "H" else 255, textvariable=var, width=8).grid(row=row, column=col, padx=8, pady=4)
        ttk.Checkbutton(dialog, text="Enabled", variable=enabled).grid(row=5, column=0, padx=8)
        ttk.Label(dialog, text="H min > H max wraps through red (179/0).", wraplength=350).grid(row=6, column=0, columnspan=3, padx=8, pady=8)
        def commit():
            try:
                item = dict(name=name.get().strip(), enabled=enabled.get(),
                            low=[int(variables["low", i].get()) for i in range(3)],
                            high=[int(variables["high", i].get()) for i in range(3)])
                proposed = list(self.rois)
                if index is None:
                    proposed.append(item)
                else:
                    proposed[index] = item
                validate_profiles(proposed)
            except (ValueError, TypeError, KeyError) as exc:
                messagebox.showerror("Invalid profile", str(exc), parent=dialog)
                return
            if index is not None:
                old_name = self.rois[index]['name']
                for rule in self.object_rules:
                    rule['colors'] = [item['name'] if c == old_name else c for c in rule['colors']]
            self.rois = proposed
            self.refresh_rois()
            self.save_settings(False)
            dialog.destroy()
            self.show_profiles()
        ttk.Button(dialog, text="Save", command=commit).grid(row=7, column=1, pady=8)
        ttk.Button(dialog, text="Cancel", command=dialog.destroy).grid(row=7, column=2)
        dialog.bind("<Return>", lambda event: commit())
        dialog.bind("<Escape>", lambda event: dialog.destroy())

    def delete_roi(self):
        index = self.selected_profile()
        if index >= 0 and messagebox.askyesno("Delete profile", "Delete " + self.rois[index]["name"] + "?"):
            del self.rois[index]
            self.refresh_rois()
            self.save_settings(False)

    def selected_features(self):
        return (
            self.color_var.get(),
            self.shape_var.get(),
            self.count_var.get(),
            self.barcode_var.get(),
        )

    def start_camera(self):
        if self.running:
            return

        if not any(self.selected_features()):
            messagebox.showwarning(
                "No feature selected",
                "Select at least one feature: Color, Shape, Count, or Barcode."
            )
            return

        try:
            index = self.camera_index.get()
            if index < 0 or self.min_area_var.get() <= 0:
                raise ValueError()
        except (tk.TclError, ValueError):
            messagebox.showerror("Settings", "Camera must be >= 0 and Min area must be > 0.")
            return

        # V4L2 is usually appropriate on Raspberry Pi OS
        self.cap = cv2.VideoCapture(index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            # Fallback
            self.cap.release()
            self.cap = cv2.VideoCapture(index)

        if not self.cap.isOpened():
            messagebox.showerror(
                "Camera Error",
                f"Cannot open camera {index}. Check the USB connection and /dev/video*, and close other apps using the camera."
            )
            return

        # Moderate resolution for Raspberry Pi performance
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 20)


        self.read_failures = 0
        self.running = True
        self.update_frame()

    def stop_camera(self):
        self.generation += 1
        self.yolo_result = None
        self.cancel_roi()
        self.display_size = None
        self.running = False
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.video_label.configure(image="", text="Camera stopped")
        self.video_label.image = None

    def update_frame(self):
        self.after_id = None
        if not self.running or self.cap is None:
            return

        ok, frame = self.cap.read()

        if not ok:
            self.read_failures += 1
            if self.read_failures >= 20:
                self.stop_camera()
                messagebox.showerror("Camera Error", "No frames received. Check the camera connection.")
                return
            self.after_id = self.root.after(100, self.update_frame)
            return
        self.read_failures = 0

        self.latest_frame = frame.copy()
        if self.drawing_roi and self.sample_frame is not None:
            frame = self.sample_frame
        display = frame.copy()

        use_color = self.color_var.get()
        use_shape = self.shape_var.get()
        use_count = self.count_var.get()
        use_barcode = self.barcode_var.get()

        use_yolo = self.yolo_var.get() and self.model is not None and (use_color or use_shape or use_count) and not self.drawing_roi
        yolo_detections = []
        if use_yolo:
            frame, yolo_detections = self.yolo_detections(frame)
            display = frame.copy()
        result_lines = []
        found = detect_profiles(frame, self.rois, self.valid_min_area()) if (use_color or use_shape or use_count) and not use_yolo else []
        if not self.rois and (use_color or use_shape or use_count):
            result_lines.append("Add a color sample to start object detection. No background calibration needed.")
        if use_count:
            result_lines.append(f"Total objects: {len(found)}")
        for profile in self.rois:
            if profile["enabled"] and use_count:
                result_lines.append(f"{profile['name']}: {sum(name == profile['name'] for name, _ in found)}")
        for i, (name, cnt) in enumerate(found, 1):
            x, y, w, h = cv2.boundingRect(cnt)
            mask = np.zeros((h, w), np.uint8)
            cv2.drawContours(mask, [cnt - np.array([x, y])], -1, 255, -1)
            color = classify_color(frame[y:y+h, x:x+w], mask) if use_color else "-"
            shape = (shape_from_contour(cnt) or "Unknown") if use_shape else "-"
            label = format_label(self.label_template, roi=name, id=i, color=color, shape=shape,
                                 count=sum(n == name for n, _ in found), x=x, y=y, w=w, h=h, barcode="-", type="-")
            cv2.rectangle(display, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(display, label, (x, max(20, y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
            result_lines.append(label)
        if use_yolo:
            result_lines = self.draw_yolo(frame, yolo_detections, display, use_color, use_shape, use_count)
        if use_barcode:
            codes = self.barcode_reader.read(frame)
            result_lines.append(f"Barcodes: {len(codes)}")
            for i, (value, btype, poly) in enumerate(codes, 1):
                x, y, w, h = 0, 25*i, 0, 0
                if poly is not None:
                    pts = np.array(poly, dtype=np.int32).reshape((-1, 1, 2))
                    x, y, w, h = cv2.boundingRect(pts)
                    cv2.polylines(display, [pts], True, (255, 0, 255), 2)
                label = format_label(self.label_template, roi="Barcode", id=i, color="-", shape="-",
                                     count=len(codes), x=x, y=y, w=w, h=h, barcode=value, type=btype)
                cv2.putText(display, label, (x, max(20, y-8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA)
                result_lines.append(label + f" [{btype}: {value}]")
        self.write_result(result_lines)
        if self.drag_start and self.drag_end:
            h, w = frame.shape[:2]
            a = tuple(int(v*s) for v, s in zip(self.drag_start, (w, h)))
            b = tuple(int(v*s) for v, s in zip(self.drag_end, (w, h)))
            cv2.rectangle(display, a, b, (255, 255, 0), 2)

        # Add small status overlay
        active = []
        if use_color:
            active.append("COLOR")
        if use_shape:
            active.append("SHAPE")
        if use_count:
            active.append("COUNT")
        if use_barcode:
            active.append("BARCODE")

        cv2.putText(
            display,
            "ACTIVE: " + ", ".join(active),
            (10, display.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA
        )

        display = resize_keep_ratio(display, 900, 480)
        self.display_size = (display.shape[1], display.shape[0])

        # Convert OpenCV BGR -> Tkinter-compatible RGB image
        rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)

        # PIL is imported here so startup errors are easier to diagnose
        from PIL import Image, ImageTk

        img = Image.fromarray(rgb)
        photo = ImageTk.PhotoImage(image=img)

        self.video_label.configure(image=photo, text="")
        self.video_label.image = photo

        # ~25 FPS target
        self.after_id = self.root.after(40, self.update_frame)

    def valid_min_area(self):
        try:
            return max(1, self.min_area_var.get())
        except tk.TclError:
            return 1200

    def write_result(self, lines):
        self.result_text.delete("1.0", tk.END)
        self.result_text.insert(tk.END, "\n".join(lines))

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
        self.stop_camera()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="Raspberry Pi Vision: color, shape, count, barcode")
    parser.add_argument("--camera", type=int, default=0, help="USB camera index (default: 0)")
    parser.add_argument("--check", action="store_true", help="Check dependencies and devices without opening the GUI")
    args = parser.parse_args()
    if args.check:
        from glob import glob
        print(f"OpenCV: {cv2.__version__}; Tkinter/Pillow: OK")
        print(f"Barcode OpenCV: {HAS_CV_BARCODE}; pyzbar: {HAS_PYZBAR}")
        print("V4L2 cameras:", glob("/dev/video*") or "not detected")
        return
    try:
        root = tk.Tk()
    except tk.TclError as exc:
        parser.exit(1, f"GUI unavailable: {exc}\nRun from a Raspberry Pi desktop or VNC terminal.\n")
    app = App(root, camera=args.camera)
    try:
        root.mainloop()
    finally:
        if app.cap is not None:
            app.cap.release()


if __name__ == "__main__":
    main()
