# Template corrections — 5 October 2026

This patch changes the assignment `mycobot_brain/brain.py` and
`mycobot_vision/vision.py`. It keeps the Brain class, manipulation function names,
existing ROS service, actions, and topics. It does not change the separate
YOLO/direct-motion pipeline or notebook execution results.

## Brain behavior

- Service coordinates mean the cube **top surface**, in `g_base`, metres/radians.
  Scene boxes use top Z minus half the cube size for their centers.
- Pickup descends the full 100 mm hover clearance before suction. Stacking places
  the suction point one carried-cube height above the supporting top. The original
  red top is cached, so raised green/blue tops are not projected onto vision's
  single-height plane again. Equal cube size is assumed.
- Missing replies, invalid poses, rejected goals, incomplete Cartesian paths, and
  failed executions stop the sequence. MoveGroup success is 1; trajectory success
  is 0. Pose and Cartesian requests both use `g_base`.
- Failure after pickup leaves suction commanded on and stops further task commands.
  Shutdown does not deliberately release a held object. This state reflects commands,
  not a vacuum sensor. ROS timeouts do **not** prove the hardware has stopped;
  inspect/cancel execution and recover manually before restarting.
- Sorting retains color selection and adds `all` for one pass over the configured
  three colors. Red routes to its own bin; the other two share a destination.
  Multiple cubes of the same color remain unsupported by the existing service.
- Menus use loops and offer `exit`. The template's automatic rejoice movement is
  no longer called after a task.

Configure `config/pick_place_brain.yaml`: set measured cube size, your three colors,
verified bin release poses, and then `bin_poses_verified: true`. The default
non-red colors are green/blue until specified otherwise. The all-zero poses are
placeholders and sorting refuses them. Do not paste SDK flange coordinates into
these pump-head targets or simply divide the flange Z by 1000.

From the workspace root, after rebuilding and sourcing the appropriate ROS workspace:

```bash
ros2 run mycobot_brain brain --ros-args --params-file config/pick_place_brain.yaml
```

The template still homes when its control menu starts. Offline tests never launch
this node. Home pose, 40 mm scene pump offset versus the assumed 68 mm nozzle,
pump release-valve wiring, and controller trajectory execution require validation
on the physical setup. Existing scene attachment/collision workarounds remain;
this patch does not certify collision behavior or physical grasp success.

## Vision behavior

The measured ArUco JSON can now contain `board_surface_z_mm` separately from
`marker_plane_z_mm`. Cube tops use board Z + cube height. For compatibility, if
board Z is absent, the marker plane is used as before; set it explicitly when
holders elevate the markers.

Optional `camera_height_above_marker_mm` enables a check against the independently
measured camera height. `camera_height_tolerance_mm` defaults to 10 mm and should
be justified from measurement uncertainty. Debug view shows detected corners in
green, reprojected corners in magenta, and mean/maximum reprojection error. Color
segmentation uses the original image before these annotations.

```bash
ros2 run mycobot_vision vision --ros-args \
  -p calibration_file:="$PWD/config/aruco_red_cube.json"
```

That calibration JSON is still missing and must contain actual measurements.
Your confirmed black-square size is 25 mm; the 5 mm board-edge gap puts marker
centers 17.5 mm inward along each adjacent board edge. Robot-base coordinates and
marker orientation still require the A–D records and decoded marker geometry.
No synthetic geometry is installed as physical calibration.

## Offline regression checks

```bash
PYTHONPATH=src/mycobot_vision PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  -p no:cacheprovider tests/test_pick_place_offline.py \
  src/mycobot_vision/test/test_aruco_geometry.py
```

The tests exercise actual template methods with ROS doubles and synthetic images:
contact heights, failure before/after suction, cached stack geometry, color routing,
Cartesian rejection/error handling, marker-height checks, and disappeared detections.
They do not establish ROS integration, calibration accuracy, or robot execution.
Shape classification and the extended color/shape/ID service are later work.

`pick_and_place_review.md` describes the pre-patch findings at commit `7519a2f`;
this document records which findings this patch addresses.
