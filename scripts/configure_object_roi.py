#!/usr/bin/env python3
"""Select a detection ROI on the Raspberry Pi display and save it to YAML."""

import argparse
from pathlib import Path
import re
import sys

import cv2


def update_yaml(path, roi):
    text = path.read_text(encoding='utf-8')
    values = dict(zip(('roi_x', 'roi_y', 'roi_width', 'roi_height'), roi))
    for key, value in values.items():
        pattern = rf'(?m)^(\s*{re.escape(key)}:\s*).*$'
        text, count = re.subn(pattern, rf'\g<1>{int(value)}', text, count=1)
        if count != 1:
            raise RuntimeError(f'Parameter {key} was not found in {path}')
    path.write_text(text, encoding='utf-8')


def main():
    workspace = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description='Interactive object detection ROI setup')
    parser.add_argument('--device', default='/dev/video0', help='RGB camera device')
    parser.add_argument(
        '--config', type=Path,
        default=workspace / 'src/studica_control/config/object_detection.yaml')
    args = parser.parse_args()

    if not args.config.is_file():
        print(f'ERROR: configuration not found: {args.config}', file=sys.stderr)
        return 1

    camera = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
    camera.set(cv2.CAP_PROP_FPS, 30)
    if not camera.isOpened():
        print(f'ERROR: cannot open {args.device}. Stop the ROS camera first.', file=sys.stderr)
        return 1

    frame = None
    for _ in range(20):
        ok, candidate = camera.read()
        if ok:
            frame = candidate
    camera.release()
    if frame is None:
        print(f'ERROR: no RGB frame received from {args.device}', file=sys.stderr)
        return 1

    window = 'Select Detection ROI'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 960, 540)
    print('Drag ROI with the mouse, then press ENTER or SPACE.')
    print('Press C to cancel and use the complete image.')
    selected = cv2.selectROI(window, frame, showCrosshair=True, fromCenter=False)
    cv2.destroyAllWindows()

    x, y, width, height = (int(value) for value in selected)
    if width == 0 or height == 0:
        roi = (0, 0, 0, 0)
        print('Full-frame automatic ROI selected.')
    else:
        roi = (x, y, width, height)
        print(f'ROI selected: x={x}, y={y}, width={width}, height={height}')
    update_yaml(args.config, roi)
    print(f'Saved to {args.config}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
