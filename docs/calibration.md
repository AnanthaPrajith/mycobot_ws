# Calibration and HSV tools

[Return to the run instructions](../README.md).

The active `vision_lab9` runs with two marker centres from
`src/lab9_pick_place/config/lab9.yaml`. The eight-corner calibrated extension is
separate and currently fails the configured geometry checks. These tools are for
image-only work; leave the brain/controller stopped during calibration.

## Active vision: tune HSV

Edit `hsv.sat_min`, `hsv.val_min` and colour intervals under `hsv.hue` in the
vision YAML, then restart `vision_lab9`. OpenCV hue is 0–179. Red wraps across
zero, so it uses two intervals. Keep both markers visible, separate touching
objects and compare detections under at least two lighting conditions. Use `s`
in the vision window to save raw and annotated evidence.

## Inspect ArUco markers

From the workspace root, with the ROS overlay/domain sourced and only the camera
publisher running:

```bash
python3 scripts/inspect_aruco_markers.py
python3 scripts/check_aruco_setup.py
```

These regenerate `calibration_images/aruco_raw.png`,
`calibration_images/aruco_inspected.png`, `config/aruco_inspection.json` and
`config/aruco_setup_diagnostic.json`. Diagnostic yaw/pose estimates are fitted
from the same image; they do not independently verify robot-frame geometry.
Do not replace measured camera position with a fitted value to force a pass.

## Separate eight-corner extension

`config/aruco_fresh.json` uses **millimetres**. Measure marker size, both decoded
marker poses, board/marker Z, camera XY/height, cube height and workspace bounds.
Match intrinsics to the actual camera resolution, focus and zoom. Resolve the
existing image/geometry inconsistency before setting the verification flag.

With independently verified measurements and `robot_measurements_verified: true`:

```bash
PYTHONPATH=src/mycobot_vision python3 scripts/refresh_aruco_reference.py \
  --config config/aruco_fresh.json --samples 30 --write
```

This fits all eight reference corners, checks stability/reprojection/camera
position, and stores the fixed pose's reference pixels. Re-capture if camera or
board moves. Both configured markers must remain visible during operation.

Run only one vision service at a time:

```bash
ros2 run mycobot_vision vision_calibrated --ros-args \
  -p calibration_file:="$PWD/config/aruco_fresh.json"
```

It returns base-frame metres/radians through `/cube_coordinates`; missing,
ambiguous, stale or invalid targets return `[1,1,1,1,1,1]`. It does not supply
`/get_object`, so it cannot be substituted into `brain_lab9` without integration.

## Intrinsics if the supplied camera model cannot be validated

Print `calibration_patterns/chessboard_9x6_20mm_A4.pdf` at Actual size / 100%,
and verify the 200 × 140 mm board dimensions. Disable autofocus only after the
board is sharp, and keep camera settings fixed. Stop the ROS camera publisher
before using direct camera capture:

```bash
python3 scripts/capture_calibration_images.py --camera /dev/video0 \
  --columns 9 --rows 6 --width 640 --height 480
```

Save at least 12 sharp, varied views using `s`; quit with `q`. Counts are **inner
corners**, and `--square-mm` must equal the measured printed square side.

```bash
python3 scripts/calibrate_camera_intrinsics.py calibration_images/chessboard_*.png \
  --columns 9 --rows 6 --square-mm 20 --output config/intrinsics_fresh.json
```

Review per-view and total reprojection error. Copy camera matrix, distortion and
resolution into the calibration JSON only after checking their provenance.

## HSV mask comparison for the calibrated extension

Use a raw photo under each lighting condition:

```bash
python3 scripts/tune_hsv.py workspace.png --color red --interval 0
python3 scripts/tune_hsv.py workspace.png --color red --interval 1
python3 scripts/tune_hsv.py workspace.png --color yellow
python3 scripts/tune_hsv.py workspace.png --color blue
```

`s` saves `config/hsv_ranges.json` and raw/cleaned masks; `q` exits. Use separate
`--output` paths to retain evidence from different lighting conditions. Load this
JSON only in `vision_calibrated`:

```bash
ros2 run mycobot_vision vision_calibrated --ros-args \
  -p calibration_file:="$PWD/config/aruco_fresh.json" \
  -p hsv_file:="$PWD/config/hsv_ranges.json"
```

## Independent coordinate validation

Create a JSON list with at least five independently measured cube-centre points
spread across the workspace. Each entry has `name`, `color`, and numeric
`xy_mm: [x, y]` values. With the calibrated vision service running:

```bash
python3 scripts/validate_fresh_workspace.py config/fresh_points.json \
  --max-error-mm 10 --output config/fresh_validation.json
```

10 mm is an example limit, not an established acceptance criterion for every
nozzle/cube. Record mean/max XY error, per-point spread, reprojection errors,
repeated-capture consistency, and false/missed detections under two lighting
conditions. Test cube removal, camera stopping and marker occlusion. Low
reprojection error alone does not validate robot-frame coordinates.
