# Lab 9 on the real robot: course ROS pipeline + our own packages (2026-10-05) — **final implementation**

The course system (camera → vision → brain → MoveIt → controller) on the real robot, with the values that worked in
[`solutions/lab9_simple/`](../lab9_simple/). **All our code is in two own packages; the provided course packages are
unchanged** (`git diff 04c715b -- pp_moveit_ws/src/mycobot_*` is empty). Details of what replaces what:
[COURSE_PACKAGE_CHANGES.md](COURSE_PACKAGE_CHANGES.md). Old guide for Tasks A–C: [`old_tasks_ABC/`](old_tasks_ABC/).

**Decision (2026-10-05 17:45): this package version (`lab9_interfaces` + `lab9_pick_place`) is our final Lab 9
implementation.** The alternative of new standalone scripts with the course controller (`LAB9_PLAN.md` §2, option B) is
not pursued; the PDF plan there still applies for Part 0 and the evaluations.

**Status (2026-10-05 17:45):** built on the Lab PC (rebuilt 16:48 without the venv); end-to-end simulation test passed.
On the robot: controller (`controller_lab9`, 20 Hz joint states), MoveIt, camera and `vision_lab9` ran; at 17:16 vision
labelled `blue_cube_1`, `red_cube_1` and a yellow cube as `yellow_unknown_1` (`pp_moveit_ws/src/lab9_pick_place/frames/lab9_annotated_20261005_171624.png`);
the workspace box no longer matches the plate edges → re-measure the marker centres. **Not yet verified on the robot:** a
complete pick and drop (suction contact, §7, 8 mm).

```
Lab PC (ROS_DOMAIN_ID 47)                                                    Pi cobot-pi1
opencv_camera ─camera/image─▶ vision_lab9 ─/get_object─▶ brain_lab9 ─/pump_controller──────▶ controller_lab9
 (course)                     markers, HSV, shape        menu, scene ─follow_joint_trajectory─▶  serial ttyAMA0
                              (/cube_coordinates too)        │                                  GPIO 20 pump, 21 valve
                                                             └─▶ move_group (lab9_moveit.launch.py) ◀─/joint_states─┘
```

## 0. Where the files are

| Package / file (`pp_moveit_ws/src/…`) | Runs on | What |
|---|---|---|
| **`lab9_interfaces`** `/srv/GetObject.srv` | both (built on the Lab PC) | request `color`, `shape`, `id` → `found`, `id`, `shape`, `coords`, `height`, `matches` |
| **`lab9_pick_place`** `/lab9_pick_place/vision_lab9.py` | Lab PC | calibration from 2 markers, HSV masks per colour, shape features, object list, `/get_object` (+ course `/cube_coordinates`) |
| `lab9_pick_place/lab9_pick_place/brain_lab9.py` | Lab PC | menu, MoveIt goals, BOX/CYLINDER scene objects by id, pick / drop / stack, joint-space home |
| `lab9_pick_place/lab9_pick_place/controller_lab9.py` | **Pi** | copy of the course controller + valve GPIO 21 + smooth execution (replaces `mycobot_controller controller`) |
| `lab9_pick_place/launch/lab9_moveit.launch.py` | Lab PC | course `move_group` + longer execution time limits (× 4 + 5 s) |
| `lab9_pick_place/launch/lab9_real.launch.py` | Lab PC | tabs: MoveIt (above), camera (course `opencv_camera`), `vision_lab9` |
| `lab9_pick_place/config/lab9.yaml` | Lab PC | every value (markers, heights, HSV, shape, bins, home, speed); edit and restart the node, no rebuild |

Used unchanged from the course: `mycobot_280_moveit2` (MoveIt config, URDF scene), `mycobot_280pi` (`opencv_camera`),
`mycobot_interfaces` (`GetCubeCoords`), `mycobot_description`.

## 1. Lab tasks covered (2 of 5)

| PDF task | Where | Done |
|---|---|---|
| **Object shape detection** | `vision_lab9.py` `detect()`, `classify()` | HSV mask per colour, opening + closing, size filter (30–75 mm); features: area, perimeter, centre (moments), `minAreaRect`, circularity `C = 4πA/P²`, `approxPolyDP` vertices, fill = A / enclosing-circle area; label `cube` / `cylinder` / `unknown`; several objects per colour; a new list every frame (nothing older than 1 s is returned); contours + labels in the window |
| **Extension of the object interface** | `GetObject.srv`, `brain_lab9.py` | request by colour + shape (+ id), explicit `found`, unique id (`red_cube_1`), all `matches`; `get_object()`, `spawn_object()` (BOX / CYLINDER, 35 mm), ids used for add / attach / detach / remove; menu asks colour + shape and lists matches; menu is a loop |
| Calibration (needed, **not** graded Task A) | `PlaneMap` in `vision_lab9.py` | 2 robot-measured marker centres → scale, rotation, shift on the plate + correction for the object height. Task A asks for K + `solvePnP` with 8 corners; works here because the camera looks straight down (workspace box on the plate within ~5 px) |

Shape rule (`lab9.yaml` → `shape`): cylinder if fill ≥ 0.75 and C ≥ 0.75; cube if fill ≤ 0.72, C ≤ 0.85 and 4–6 vertices;
else `unknown`. Fill is the main criterion (square 64 %, circle 100 % of the enclosing circle); circularity is lowered by
pixel noise (10-02 frame: cylinder C 0.82).

## 2. Values (`lab9_pick_place/config/lab9.yaml`)

| Value | Now | How to get it |
|---|---|---|
| `markers.centers` | id 1 (0.135, −0.060), id 2 (0.255, 0.060) | ruler from the robot centre to each marker centre (plate moved +45 mm on 10-05) |
| `objects.surface_z_m` | 0.012 | lowered until the suction made contact (pick worked at flange z 115 mm with pymycobot) |
| `objects.cylinder_h_m` | 0.035 | **MEASURE** (assumed = cube) |
| `camera.height_m` | 0.47 | **MEASURE** lens → plate (estimate) |
| `robot.bins` | red (0.065, 0.170), yellow (0.200, 0.170), blue (0.185, 0.170) | check that each bin circle is over its bin |
| `robot.drop_tip_z_m` | 0.100 | clears a cube already in the bin by ~25 mm |
| `robot.home_joints_deg` | [109.86, 1.58, −93.51, 1.14, 0.08, 20.03] | home posture (pymycobot `get_angles` at [2.7, 178.8, 199.9, 179.22, −0.18, −0.17]); same home as `lab9_simple` |
| `robot.velocity_scaling` | 1.0 | course value (0.3 was ~3× slower) |

Frames: `g_base` (MoveIt) = pymycobot base frame. `pump_head` = flange + 40 mm, real nozzle tip = flange + 68 mm →
contact at `pump_head z = object top + 0.028`. `lab9_simple/config.py` holds the same values in mm (keep both in sync
while `lab9_simple` is used).

## 3. One-time setup (after `git pull`)

```bash
# Lab PC — NOT inside the (mycobot) venv: it would put the venv Python (numpy 2) into the node scripts
deactivate 2>/dev/null
cd ~/mycobot-project/pp_moveit_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select lab9_interfaces lab9_pick_place
head -1 install/lab9_pick_place/lib/lab9_pick_place/vision_lab9     # must print #!/usr/bin/python3
```
```bash
# [Pi]  (GitHub token for the pull after a reboot); nothing to build
cd ~/mycobot-project && git pull
```
The Pi runs only `controller_lab9.py`, started directly with `python3` (§4 step 1). `colcon build` of `lab9_pick_place`
does **not** work on the Pi: its `package.xml` depends on course packages that are not built there (MoveIt is not
installed on the Pi), 10-05. Both machines: `source ~/rosenv9.sh` in
**every** terminal (ROS Jazzy + `pp_moveit_ws/install` + `ROS_DOMAIN_ID=47`), and leave the `(mycobot)` venv first
(`deactivate`): the nodes run with the system Python.

## 4. Run the full pipeline (order matters)

Before: no Jupyter kernel and no `robot_pi.py` on the Pi (one program per serial port), markers uncovered, objects on the
plate between x ≈ 135 and 250 mm, arm **not** in the old 410 mm pose (see §8, −10).

| # | Where | Command | Check / what it does |
|---|---|---|---|
| 1 | [Pi] terminal 1 | `source ~/rosenv9.sh && python3 ~/mycobot-project/pp_moveit_ws/src/lab9_pick_place/lab9_pick_place/controller_lab9.py` | `Pump ready!`, `FJT action server ready!`. Robot driver: publishes `/joint_states` (20 Hz), executes trajectories, pump + valve. Moves nothing by itself |
| 2 | Lab PC terminal 1 | `source ~/rosenv9.sh && ros2 topic hz /joint_states` | ~20 Hz = network + controller OK; Ctrl+C |
| 3 | Lab PC terminal 1 (desktop) | `ros2 launch lab9_pick_place lab9_real.launch.py` | 3 tabs: **MoveIt** (`You can start planning now!`), **Camera**, **Vision** (`calibrated: … 1.0 mm/px, rotation ~89 deg`) |
| 4 | Lab PC | look at the `vision_lab9` window | yellow box on the plate, magenta circle on the robot base, each object outlined + `red_cube_1 (x, y) mm`; log `objects: …` on every change |
| 5 | Lab PC terminal 2 | `source ~/rosenv9.sh && ros2 service call /get_object lab9_interfaces/srv/GetObject "{color: red, shape: cube}"` | `found=True`, coords (m), `matches` = what the brain will get |
| 6 | Lab PC terminal 2 **[MOVES]** | `ros2 run lab9_pick_place brain_lab9` | arm goes to the home posture, then the menu |

Menu: `1` pick one object (colour r/y/g/b, shape c = cube / z = cylinder, choose among matches, bin: Enter = its colour),
`2` auto sort (all objects into their colour's bin, red → yellow → blue, ≤ 6 per colour), `3` stack cubes (red bottom,
yellow, blue), `q` quit (pump off). Vision keys: `c` re-read the markers, `s` save raw + annotated frame to `pp_moveit_ws/src/lab9_pick_place/frames/` (next to `LAB9_REPORT.md`).

Stop: `q` in the brain → Ctrl+C in the 3 tabs → Ctrl+C in the controller (last). End of day on the shared Pi:
`git credential-cache exit`.

Without a desktop (instead of step 3), each in its own terminal with `source ~/rosenv9.sh`:
`ros2 launch lab9_pick_place lab9_moveit.launch.py`, `ros2 run mycobot_280pi opencv_camera`,
`ros2 run lab9_pick_place vision_lab9`.

## 5. Part 0 baseline (original system, as delivered)

[Pi] `ros2 run mycobot_controller controller` (the course controller), then on the Lab PC
`ros2 launch mycobot_brain brain.launch.py` (course brain, MoveIt, camera, `vision`). Expected and worth documenting
(run guide `solutions/Lab8_Lab9_run_guide.md` §6d): `vision` finds nothing or wrong positions with this camera (fixed pixel
map of the old camera, 50–150 px filter, blue thresholds), 40 mm boxes, one object per colour; on the robot slow
stop-and-go motion, `TIMED_OUT` after most moves while the arm still finishes, objects may stay on the pump (no valve).
Never run both controllers at the same time.

## 6. Evaluation (deliverables)

**Shape detection**
1. Scenes: cubes and cylinders of several colours, two objects of the same colour, one cube rotated ~30°; `s` in each.
2. Per scene: labels and centres from the vision log (`objects: red_cube_1 (150, 20) C 0.80 fill 0.65 v 4, …`) against a
   ruler; C, fill and v (vertices) justify each label.
3. One uncertain case, e.g. a cylinder touching a cube of the same colour (merged blob → `unknown` or wrong shape).

**Object interface**
```bash
ros2 service call /get_object lab9_interfaces/srv/GetObject "{color: red, shape: cube}"       # existing
ros2 service call /get_object lab9_interfaces/srv/GetObject "{color: red, shape: cylinder}"   # absent -> found False
ros2 service call /get_object lab9_interfaces/srv/GetObject "{color: blue, shape: ''}"        # several -> matches
```
Then menu `1` with two objects of the same colour (selection list) and menu `2`. Per pick: id, position, success, miss in mm.

## 7. Limits and safety (what the tests showed)

- **Reach:** MoveIt plans hover + vertical descent for x ≈ 0.135–0.250 m (simulation). Near edge (x 0.120, centre) and
  far edge (x 0.265) fail planning → the brain stops (pump off, home) without moving there.
- **Yaw:** suction needs no yaw; MoveIt's IK fails at random for some yaws → 6 yaw candidates (2 rounds), retried only on
  planning errors (−1, −2, −31) where nothing moved. **Not** on −6 (TIMED_OUT): the arm was moving.
- Descent and lift: straight lines without collision checks against the target; free-space moves keep full checking.
- **Height offset (open):** MoveIt's model puts the flange 7.8 mm lower than pymycobot for the same joints (home posture).
  If picks stop ~8 mm above the object: `robot.tip_below_pump_head_m` 0.028 → 0.020.
- Scene changes: the brain waits 0.5 s before planning (error −2 otherwise). Bins are **not** in the MoveIt scene.
- `opencv_camera` (course) reopens the camera for every frame: ~10 Hz at best.

## 8. Troubleshooting

| Symptom | Cause → fix |
|---|---|
| `Package 'lab9_pick_place' not found` | Lab PC terminal not sourced: `deactivate; source ~/rosenv9.sh` (on the Pi the package is not built: start the controller with `python3`, §4 step 1) |
| no vision window; `ros2 run lab9_pick_place vision_lab9` → `Segmentation fault` (numpy 1.x / 2 message) | package built inside the `(mycobot)` venv: node scripts start with the venv Python (numpy 2) and `cv_bridge` crashes → `deactivate`, rebuild (§3), check `head -1` (10-05) |
| controller exits with `TypeError: 'int' object is not iterable` (`Current coordinates: -1`) | the robot did not answer the first serial reads after start; restart the controller (worked the second time, 10-05). The course controller has the same weakness |
| brain waits for `follow_joint_trajectory` | `controller_lab9` not running on the Pi, or domain / firewall: step 2 |
| controller: GPIO busy / serial errors | another program holds the robot (Jupyter kernel, `robot_pi.py`, the course controller): stop it |
| vision: `marker(s) [..] not visible` | uncover the markers, press `c` |
| objects not outlined / wrong shape | `hsv` / `shape` in `lab9.yaml`; restart `vision_lab9` |
| `no plan to pick … (reach limit?)` | object outside x 0.135–0.250 m: move it towards the plate centre |
| nozzle above / pressing into the object | `objects.surface_z_m` (±2 mm) or `robot.tip_below_pump_head_m` (§7) |
| MoveIt error −10 (`START_STATE_IN_COLLISION`) | arm pose collides in MoveIt's scene (old `lab9_simple` home at 410 mm is inside the scene's `env_camera` box): stop the controller, park the arm (command below), restart the controller |
| MoveIt error −6 (`TIMED_OUT`) | execution slower than MoveIt allows; the controller cannot be cancelled, the arm finishes. Use `lab9_moveit.launch.py` (not the course `move_group.launch.py`) and restart MoveIt |

Park the arm (Pi, controller stopped, **MOVES**, speed 30): the Lab 8 intermediate pose, collision-free in MoveIt.
```bash
PY=~/venvs/mycobot/bin/python
$PY -c "import sys,time;from pymycobot.mycobot import MyCobot as M;c=[float(v) for v in sys.argv[1:7]];s=int(sys.argv[7]);m=M('/dev/serial0',1000000);time.sleep(.3);m.send_coords(c,s,0);time.sleep(6);print('angles',m.get_angles());print('coords',m.get_coords())" 155 -20 245 180 0 0 30
```
