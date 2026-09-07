# Multiple color detection baseline

Run on the Raspberry Pi desktop, with the previous camera application stopped:

```bash
cd /home/vmx/studica_ws
./scripts/start_color_roi_tracker.sh multiple_colors:=true
```

The window detects red, green and blue regions immediately, including multiple
separate regions of the same color. Press M to toggle the live binary mask and
Q/Esc to close the viewer. Stop the launch terminal with Ctrl+C to stop the camera.
The SSH launcher requires an active, accessible Raspberry Pi desktop session.

This mode uses HSV ranges, morphological opening/closing and contour area
filtering. It is an independently implemented baseline using the approach in
[the requested example](https://www.geeksforgeeks.org/python/multiple-color-detection-in-real-time-using-python-opencv/)
and [OpenCV's official inRange tutorial](https://docs.opencv.org/4.x/da/d97/tutorial_threshold_inRange.html).
Thresholds are starting values, not calibrated measurements. Adjust red/green/blue
low/high arrays in `src/studica_control/config/color_roi_tracker.yaml`, then restart.
OpenCV H ranges from 0 to 179; S and V from 0 to 255. Low H greater than high H
means a wrapped range through red at 179/0.

Outputs:

- `/color_tracker/image/compressed`: RGB with color boxes and pixel centers.
- `/color_tracker/mask/compressed`: PNG of all accepted color regions.
- `/color_tracker/result`: JSON with `mode`, `count` and `objects`.

Each object includes `color`, `bbox` [x,y,width,height], `pixel_x`, `pixel_y`,
`distance_m`, `x_m`, and `y_m`. Pixels start at the top-left. Metric coordinates
are camera optical coordinates (right/down/forward); distance is axial depth,
not Euclidean range. Depth must be aligned to RGB; the supplied camera launch
requests registration. Verify alignment on a real target before using these
estimates for grasping. Stale or unavailable depth produces null metric values.
The software rejects depth older than 0.5 s or over 0.15 s apart from RGB.

This is per-frame color-region detection, not object identity tracking. Two
touching same-color objects may merge. Colors do not imply competition Object
1/2/3/4; that requires separate shape/object classification. ROI teaching remains
available with the original command without `multiple_colors:=true`.

Performance defaults: the detector resizes RGB to width 320 (320x180 for the
640x360 input) and processes at most 15 FPS. The capture/depth registration profile
is unchanged. Incoming image queues keep only the latest sample; JPEG/PNG encoding
only runs when subscribers exist. Pixel coordinates and boxes refer to the smaller
image; JSON includes `image_width` and `image_height`. Depth lookup rescales those
pixels into the depth image. The minimum area threshold is scaled automatically.
Set `processing_width: 640` to restore image detail. Restart after YAML changes.

Validation: synthetic red/green/blue regions, two red regions, hue wrap at 179/0,
small-noise rejection and empty scene. Real lighting and desktop display still
need verification on the Raspberry Pi.
