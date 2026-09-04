# Gemini E Object Detection Quick Start

This workspace provides a lightweight RGB-D detector for competition objects.
It uses depth geometry and configurable physical-size rules, so no training
dataset is required for the initial setup.

## Start everything

Connect the Orbbec Gemini E and run:

```bash
cd /home/vmx/studica_ws
./scripts/start_orbbec_foxglove.sh
```

For interactive ROI selection on the Raspberry Pi desktop, stop any running
camera node and run:

```bash
./scripts/select_roi_and_start.sh
```

Drag a rectangle with the mouse and press Enter or Space. Press C to use the
whole image. The selected ROI is saved to the detector YAML before the ROS
camera and Foxglove bridge start automatically.

In VS Code, forward port `8765`. Connect Foxglove on the laptop to
`ws://localhost:8765`.

Add an Image panel and select:

```text
/object_detection/debug_image/compressed
```

Machine-readable detections are published as JSON on:

```text
/object_detection/results
```

Each detection includes the object number, geometric class, dominant RGB color,
distance, position, and estimated dimensions. The debug bounding-box color also
matches the detected object color. Bounding boxes and labels are overlaid on the
live RGB camera image; depth is used internally for segmentation and measurement.

## Configuration

Edit `src/studica_control/config/object_detection.yaml` to adjust the depth
range, background separation, minimum contour area, processing rate, or object
dimensions. Set `background_depth_mm` to the measured table/wall distance for
the most repeatable segmentation. Leave it at `0` to estimate the background
automatically.

The current classifier contains the published size ranges for Objects 1-3.
Object 4 is reported as `unknown` until its specification is released.

## Important setup notes

- Mount the camera rigidly and keep its pose unchanged after calibration.
- Keep the objects inside the configured 0.2-2.5 m working range.
- Avoid reflective surfaces in the depth camera field of view.
- The reported dimensions are an initial image-plane estimate. Calibrate and
  validate them against the real competition objects before relying on them.
- Press Ctrl+C (keyboard shortcut, not a typed command) to stop all processes.
