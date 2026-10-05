# Robot PC: ordered pick-and-place setup and calibration

Updated 6 October 2026. Use this guide from `~/mycobot_ws` on the robot PC.
The corrected application uses **mycobot_vision → mycobot_brain → MoveIt →
mycobot_controller**. Its first demonstrations are two-bin color sorting and
stacking equal-size cubes. Shape recognition is later work.

**Read all steps before running hardware.** Steps 1–8 can be completed without
starting the Brain. The Brain automatically commands home when its menu starts;
launching it is not a read-only check. Complete the physical checks in step 9
before the execution commands in step 10.

## 1. Update the source without losing robot-PC measurements

Stop existing Brain/motion programs first. Check for local changes:

```bash
cd ~/mycobot_ws
git status --short
git log -3 --oneline
```

If there are local edits or untracked calibration files, preserve them before
pulling. Copy measurements outside the repository, for example:

```bash
mkdir -p ~/mycobot_calibration_backup
cp -a config ~/mycobot_calibration_backup/config-before-update
```

Commit intentional source edits, or use `git stash push -u` and restore them with
`git stash pop` after updating. Resolve any conflicts rather than discarding
measurements. Ignored files are not included by `stash -u`.

When the working tree is clean:

```bash
git pull --ff-only origin main
git log -3 --oneline
```

The source must include commit `a6ae0c3` (pickup/stacking corrections). If the
fast-forward fails because branches diverged, stop and inspect the local commits;
do not reset or force-push to work around it.

## 2. Select the existing ROS environment

Use the same installed ROS distribution for all communicating nodes. The course
uses Jazzy; this repository previously used Humble too. Open a fresh terminal and
source the distribution actually installed on this PC:

```bash
ls /opt/ros
# Choose ONE of these, according to this PC:
source /opt/ros/jazzy/setup.bash
# source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

Use the robot's established Python environment for pymycobot and GPIO. If an
existing compatible virtual environment is used, activate it before building.
Do not install the pinned YOLO requirements into the ROS Python environment for
this ArUco application. Do not mix ROS distributions or copy another PC's
`build/`, `install/`, or `log/` directories.

Check dependencies without opening devices:

```bash
python3 - <<'PY'
import numpy, scipy, cv2, rclpy
from cv_bridge import CvBridge
from pymycobot.mycobot280 import MyCobot280
import RPi.GPIO
print('OpenCV:', cv2.__version__)
print('ArUco available:', hasattr(cv2, 'aruco'))
assert hasattr(cv2, 'aruco'), 'Install compatible OpenCV with ArUco support'
print('Required imports succeeded; no devices opened')
PY
```

If an import fails, resolve that dependency for this ROS/Python environment before
continuing. The main README contains fresh-machine dependency setup. Avoid mixing
multiple pip/apt OpenCV installations without checking `cv_bridge` compatibility.

## 3. Rebuild the application and its dependencies

```bash
cd ~/mycobot_ws
colcon build --symlink-install --packages-up-to \
  mycobot_brain mycobot_vision mycobot_controller mycobot_280pi mycobot_280_moveit2
source install/setup.bash
ros2 pkg executables mycobot_brain
ros2 pkg executables mycobot_vision
```

Expected executables include `mycobot_brain brain` and `mycobot_vision vision`.
Use `vision`, not the registered but absent `vision2` module. If switching ROS
distributions, use a clean build rather than reusing the old installation.
Repeat the chosen ROS source and `source ~/mycobot_ws/install/setup.bash` in each
new terminal.

## 4. Run the offline regression checks

These tests use synthetic images and ROS doubles and send no hardware commands:

```bash
cd ~/mycobot_ws
PYTHONPATH=src/mycobot_vision PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q \
  -p no:cacheprovider tests/test_pick_place_offline.py \
  src/mycobot_vision/test/test_aruco_geometry.py
```

The corrected version passed **39 tests** on the development PC. A passing result
checks software behavior; it does not verify the robot's tool geometry or camera
measurements. Install compatible pytest if it is missing.

## 5. Recover your already recorded board and bin measurements

```bash
ls -l config
```

Find the board-corner recorder output, normally
`config/board_teaching_points.jsonl`. It was **not present in the published source**
when the corrections were made. Locate it on this robot PC or restore its backup.
Do not repeat the measurements merely because another PC lacked this file.

The existing `workspace_calibration_points.json` and
`red_workspace_calibration_points.json` contain yellow/red cube-centering samples;
they are not the new board A–D measurements.

If A–D truly need recording, `scripts/teach_board_points.py` provides an interactive
hand-guiding recorder. First read its instructions and stop every other serial
controller. It releases servos only after explicit input; support the arm while
released. Its A/B/C/D labels refer to a reference photograph: document your own
corner correspondence clearly. Do not assume alphabetical order runs clockwise.

Record or recover:

| Measurement | Required convention |
|---|---|
| Board corners | Corrected suction-tip XY in robot base; retain original poses/orientations |
| Board surface Z | Top of white board in robot-base millimetres |
| Marker surface Z | Printed black-square plane; include holder elevation |
| Tool geometry | Flange-to-suction-tip transform and SDK readback definition |
| Bin release poses | Target `pump_head` poses in `g_base`, metres/radians |
| Cube size | Actual equal cube side/height, not an assumed template value |

Hovering over a corner does **not** directly establish board Z. Convert the readback
to the actual tip position, then subtract known hover clearance, or independently
measure the board plane. Confirm whether SDK coordinates denote flange or a
configured tool before applying an offset.

The general tool relation is `p_tip = p_flange + R_flange @ t_flange_tip`.
Euler sequence and tool-vector direction must be verified. The saved affine files
use conventions that differ from other scripts.

The direct-motion node contains taught bin XY values, but its release heights are
SDK-specific and cannot be pasted directly into Brain poses. Do not change the
separate YOLO/direct-motion pipeline just to run this assignment pipeline.

## 6. Calculate marker geometry from the board measurements

Confirmed physical dimensions:

- Each marker's **black square** is **25 × 25 mm**.
- The black square is **5 mm from each adjacent board edge**.
- Thus each marker center is **17.5 mm inward from both adjacent edges**.
- There are two markers: four corners per marker, eight correspondences total.

For a corrected board corner `P` with inward unit vectors `ex` and `ey` along its
two edges, the adjacent marker center is:

```text
center = P + 17.5 * ex + 17.5 * ey     (millimetres)
```

At the opposite corner the inward directions reverse. Derive them from the actual
corner labeling. A tilted board requires adapting the current horizontal-plane
model; the current code uses constant marker Z.

Determine marker dictionary and IDs from the printed marker source or verified
camera detection. Their IDs cannot be inferred reliably from the photograph alone.
`top_edge_yaw_deg` is the angle in robot-base XY of the **decoded top-left →
top-right** edge, measured from base +X toward +Y. Rotating the printed marker can
change this angle even if its square looks unchanged.

The code generates each marker's corners in decoded TL, TR, BR, BL order.
No additional robot teaching of all eight corners is necessary if the measured
board geometry, marker offsets, and orientations define them accurately.

## 7. Obtain intrinsics and create the measured ArUco JSON

Use camera matrix K and distortion coefficients calibrated for the actual camera,
focus, and resolution. If these cannot be verified, perform a multi-view
checkerboard/ChArUco intrinsic calibration first. A single image of two stationary
markers is not a complete intrinsic calibration dataset.

Create `config/aruco_red_cube.json`. The following is a **schema skeleton**;
`null` fields must be replaced with measurements. Do not run it as calibration or
set the verification flag just to bypass validation:

```json
{
  "units": "mm",
  "robot_measurements_verified": false,
  "dictionary": null,
  "intrinsics_resolution": null,
  "camera_matrix": null,
  "dist_coeffs": null,
  "marker_size_mm": 25.0,
  "marker_plane_z_mm": null,
  "board_surface_z_mm": null,
  "cube_height_mm": null,
  "camera_height_above_marker_mm": null,
  "camera_height_tolerance_mm": null,
  "max_reprojection_error_px": null,
  "workspace_xy_mm": {"x": null, "y": null},
  "markers": [
    {"id": null, "center_xy_mm": null, "top_edge_yaw_deg": null},
    {"id": null, "center_xy_mm": null, "top_edge_yaw_deg": null}
  ]
}
```

`dictionary` must be an OpenCV name such as `DICT_4X4_50`, but use the verified
printed dictionary, not this example by default. Resolution is `[width,height]`;
K is 3×3; distortion is an OpenCV coefficient vector. Workspace bounds are
`[min,max]` in base XY. Choose and justify positive height/error tolerances.
The rectangle is a filter, not proof of reachability or collision clearance.

Both markers must be on the shared plane and use the configured shared size.
Set `robot_measurements_verified` true only after checking the physical geometry.

Validate the geometry without opening devices:

```bash
PYTHONPATH=src/mycobot_vision python3 - <<'PY'
import json
from mycobot_vision.aruco_geometry import validate_config
with open('config/aruco_red_cube.json') as f:
    config = json.load(f)
points, matrix, distortion = validate_config(config)
print('Marker corners in robot-base mm:')
print(points)
assert len(points) == 8
PY
```

Also confirm board surface, object height, and camera-height values are finite and
physically meaningful. The node checks the optional height test while processing
frames. Matching cube height in Brain is `cube_height_mm / 1000`.

## 8. Start camera and vision only; validate positions

Check your camera device index. For example `num:=2` selects `/dev/video2`; use the
actual device. Do not let the capture utility and ROS camera open it simultaneously.
In two prepared terminals:

```bash
# Terminal A: camera only
ros2 run mycobot_280pi opencv_camera --ros-args -p num:=2
```

```bash
# Terminal B: from ~/mycobot_ws
ros2 run mycobot_vision vision --ros-args \
  -p calibration_file:="$PWD/config/aruco_red_cube.json"
```

The camera publisher does not explicitly set resolution, so verify the resolution
matches the intrinsic calibration. It opens the camera per callback and lacks
robust frame handling; fix camera read failures before proceeding.
Use `-p show_debug:=false` for a headless session; arrange recorded image evidence
separately if no debug window is available.

In another prepared terminal:

```bash
ros2 topic hz /camera/image
ros2 service call /cube_coordinates mycobot_interfaces/srv/GetCubeCoords "{color: red}"
```

The response is `[x,y,z,roll,pitch,yaw]`: position in metres, angles in radians.
Z denotes the cube top. Zero angles are suction placeholders, not a measured full
object orientation. `[1,1,1,1,1,1]` means unavailable. One ambiguous color with
multiple detected candidates is also unavailable.

Check:

1. Both markers are detected; green observed corners and magenta reprojected
   corners agree. Record mean/max pixel errors across several frames.
2. Estimated camera height agrees with the independent measurement.
3. Move a cube among several independently measured positions, including edges.
   Record measured versus detected XY and their Euclidean errors in mm.
4. Remove the cube: its service response becomes unavailable.
5. Hide a marker: coordinates become unavailable, rather than reusing old data.
6. Test the selected colors under at least two lighting conditions. Capture mask
   examples separately; the current debug view does not save masks automatically.

The debug overlay does not save an annotated image automatically. Save screenshots
or add an image-saving function to collect the submission evidence.
The optional `capture_aruco_red_cube.py` opens the camera directly; its separate
geometry path predates the board/marker-height separation. Use the ROS Vision
service for validation when holders raise markers above the board. The capture
utility is not a replacement for this validation.

## 9. Configure Brain and resolve physical integration checks

Edit `config/pick_place_brain.yaml`:

- Set `cube_size_m` to the measured equal cube size, matching Vision.
- Set `sort_colors` to red and the two other colors used. Defaults are green/blue.
- Fill `red_bin_pose` and `other_bin_pose` with measured, converted `pump_head`
  release poses in `g_base`: `[x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad]`.
- Set `bin_poses_verified: true` after validating those poses.

Keep these measurements separate from the old per-color affine corrections.
Do not apply the old corrections to ArUco coordinates without new validation.

**The following checks remain unresolved in the published source.** Complete them
on this setup before starting Brain:

- The scene URDF has a 40 mm flange-to-`pump_head` offset, while previous direct
  code assumes a 68 mm nozzle. Verify which point the model actually represents
  and align it with the real suction contact point.
- Verify Brain's existing home pose `[0.16,-0.06,0.32,0,1.57,0]` in `g_base` is
  reachable and clear. The menu automatically commands it.
- `mycobot_controller` controls GPIO 20 only. Previous two-pin wiring also controls
  release valve GPIO 21. Verify release behavior and correct the controller if
  that wiring requires it.
- The controller assumes waypoint velocities are present, maps them to SDK speed,
  and reports success without measured final-pose verification. Validate/correct
  this before relying on its success reports.
- Validate planning-scene cube attachment and contact collision allowances in RViz.
  Existing template workarounds are still present.

Review `docs/pick_and_place_review.md` and `docs/pick_place_corrections.md` for
context. If a check fails, fix and test that component first; do not compensate by
inventing coordinates or disabling calibration verification.

## 10. Start motion components for a supervised trial

Proceed only after step 9. Keep camera and Vision running from step 8. Prepare the
robot with no unsupported payload, arm clear, and a usable stop procedure.
Run one serial controller only: stop `motion_node`, direct SDK recorders, GUIs,
notebook serial connections, and other competing serial users.

In separate prepared terminals, start:

```bash
# Hardware controller: opens robot serial/GPIO
ros2 run mycobot_controller controller
```

```bash
# MoveIt planning stack
ros2 launch mycobot_280_moveit2 move_group.launch.py
```

Use your validated robot-state/TF launch alongside MoveIt if required by the
installed configuration. `move_group.launch.py` does not by itself establish all
hardware state/TF publishers. Verify before starting Brain:

```bash
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo g_base pump_head
ros2 service list
ros2 action list
```

Required interfaces include `/cube_coordinates`, `/compute_cartesian_path`,
`/move_action`, and `/arm_group_controller/follow_joint_trajectory`. Check the
joint state is fresh, joint names match the MoveIt model, and TF agrees with the
physical tool. RViz visualization, if configured, helps inspect the scene.
Do not run a simulated trajectory server together with the hardware controller.

Now, **this command can move the robot to home immediately**:

```bash
cd ~/mycobot_ws
ros2 run mycobot_brain brain --ros-args --params-file config/pick_place_brain.yaml
```

Avoid the old `brain.launch.py` shortcut: it does not pass the new measured Vision
calibration or Brain parameters, and it does not start the hardware controller.

## 11. Demonstrate sorting, then stacking

First verify predicted pickup XY with supervised hover trials using the established
robot jogging procedure. The Brain has no separate hover-only menu item.
Then trial one cube and one release before sorting all three.

**Sorting:** choose menu `2`. Select a configured color (or `r`, `g`, `b`, `y`).
The bin is chosen automatically: red to the red bin, the configured non-red colors
to the other bin. `all` makes one pass through the three colors; absent colors are
skipped. `exit` returns to the main menu. Have at most one cube of each color in
the detection workspace and keep bins outside that detection region.

**Stacking:** begin with separate, equal-size red/green/blue cubes on the board.
Choose menu `1`. Red is the base; green is placed on red, followed by blue. If green
is absent, blue is placed on red. The base position and height are cached for the
sequence. Do not start this mode on an already raised stack; Vision assumes a
single cube top plane and cannot correctly localize arbitrary stacked tops.

If a motion fails, the task stops issuing subsequent commands. If an object may
be held, suction is intentionally left commanded on. Support/recover it manually
before restarting. A ROS timeout or Ctrl+C is not a physical emergency stop and
does not guarantee that an accepted trajectory has stopped.

## 12. Record evidence and preserve measurements

For submission retain:

- Updated `vision.py`, `aruco_geometry.py`, and `brain.py`.
- Matching `GetCubeCoords.srv` and dependency/run instructions.
- Measured ArUco JSON, Brain parameters, coordinate frames and units.
- Detected/reprojected-marker image; mean/max reprojection errors.
- Independent measured-versus-detected position table (mean/max mm errors).
- Detection results under two lighting conditions and after object removal.
- Sorting and stacking results, failures, and relevant limitations.

Later shape/ID extensions must update both server and client and rebuild the
interface package. Current code does not distinguish cubes from cylinders.

Back up the new robot-PC measurements outside the repo. Review intended changes
with `git status` and `git diff` before committing them; do not include generated
build files. Preserve calibration files when updating either PC.

## Instructions for a coding assistant on the robot PC

Read this file and the correction notes first. Inspect local state and preserve
robot-PC measurements before updating. Retain the course template's functions and
ROS interfaces unless the requested next task needs an interface change. Work
through the steps in order, record completed checks, and distinguish software
regression tests from physical validation. Do not infer missing A–D points, marker
IDs, heights, camera intrinsics, or bin/tool poses. Do not launch Brain, release
servos, change GPIO, or command robot motion as part of a read-only check. Carry
out hardware actions only within the user's explicitly requested robot trial.
