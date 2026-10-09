# Lab 9 on the real robot: step-by-step guide (Part 0, Tasks A, B, C)

This guide takes you through the final lab (`pdfs/Pick_And_Place-2.pdf`, Sep 23) with the **real camera, robot and pump only**. Every step says what it is for, what to type on which machine, what you should see, and how the code behind it works, down to the syntax, with small worked examples.

- Old reference solution (algorithms tested on synthetic images): `solutions/lab9_old_synthetic/`. Not needed here.
- Setup, start order and stopping of the whole system: `solutions/Lab8_Lab9_run_guide.md` (this guide links to its steps A0–A7, B1–B6, 6c, D2–D3 and does not repeat them).
- Code for this guide: this folder. Written 2026-10-02. Tested on the home laptop on generated images only (to catch bugs); **not yet run in the lab**.

Marks:
- **[MOVES]**: the arm moves. A person runs it, workspace clear, hand at the robot's power switch.
- **[check]**: not verified yet; the step says how to verify it.
- **⚠**: warning: something that can damage, mislead or waste a lab session.
- **[record]**: write this down or keep this file: it goes into a deliverable.
- `brain.py:337` means line 337 of `pp_moveit_ws/src/mycobot_brain/mycobot_brain/brain.py` (similarly `vision.py`, `controller.py`).

Contents: 0 How to use this guide · 1 What you must hand in · 2 The system in plain words · 3 Safety · 4 Part 0 (required) · 5 Recording real frames · 6 Task A · 7 Task B · 8 Task C · 9 Final evaluation · 10 Hand-in and git · 11 Troubleshooting

---

## 0. How to use this guide

### 0.1 Choose your approach
The PDF asks for Part 0 plus **two** of five tasks. This guide implements three of them, so you can take either pair:

| | **Approach 1: A + B** | **Approach 2: B + C** |
|---|---|---|
| Choose it when | the two ArUco markers are fixed at known positions **and** you have a checkerboard (printed, or shown full-screen on a laptop/tablet) or K from the TAs | markers or checkerboard are missing or unclear |
| What improves | pick **position accuracy** (A) and detection robustness (B) | detection robustness (B) and how thresholds are found (C) |
| Order | §4 Part 0 → §5 frames → §6 Task A → §7 Task B → §9 | §4 Part 0 → §5 frames → §8 Task C → §7 Task B → §9 |
| Your `vision.py` | `vision_AB.py` (calibrated mapping + robust detection) | `vision_B.py` (robust detection, original mapping) |
| Lab sessions | 1: Part 0, record frames, measure the setup · home: TA1–TA3, TB tuning · 2: install, TA6, picks | 1: Part 0, record frames · home: Task C, TB tuning · 2: install, live checks, picks |

⚠ Approach 2 keeps the original pixel → robot constants. If Part 0 shows position errors above ~1 cm, picks will still miss after B + C. That is not a failure of B or C (they are about detection), but say it in the report; for picks you can use the run guide's marked workaround (re-fit the constants, D3 (b)).

### 0.2 The pattern of every step
**Goal** (what the step proves) → **Why** → **Do** (machine named) → **Expect** → **If not** → **Record** → **Understand** (the code, line by line, with the syntax explained and a worked example).

### 0.3 Machines and folders
| Machine | Repo | Used for |
|---|---|---|
| Lab PC `CoRobot2` | `~/mycobot-project` (branch `myCobot-lab`) | camera, vision, MoveIt, brain, all scripts of this folder |
| Robot Pi `cobot-pi1` (`ssh cobot`) | `~/mycobot-project` | only the controller (run guide B1) |
| Home laptop | `~/TUD/cobots` | the OpenCV-only scripts on recorded frames (TA1–TA3, Task C, labelling) |

All commands below are run **from the repo root** (`cd ~/mycobot-project` on the Lab PC). Results go to `lab9/` in the repo (`lab9/frames/`, `lab9/calib/`, `lab9/tables/`, `lab9/hsv/`, `lab9/eval_B/`, `lab9/eval_C/`); the scripts create these folders.
Scripts that talk to ROS (`record_frames.py`, `position_errors.py`, `evaluate_detection.py run|sequence`) need `source ~/rosenv9.sh` first. The others need only Python 3 with OpenCV, NumPy and PyYAML (system Python on both machines; OpenCV 4.6).

### 0.4 Before the first lab session
- Run guide A0–A6 are marked done; **A7 (network test) is still TODO**. Do A7 first; if it fails nothing else works.
- Read §1 and §2 of this guide and the run guide §0 (safety surprises).

---

## 1. What you must hand in

| Part | Required | Deliverable (PDF) | Where it comes from |
|---|---|---|---|
| **0** Explore | yes | short report: initial cube positions, menu choices, bins, outcomes, ≥ 2 limitations; diagram or ordered list of the data flow camera image → finished pick. It is the **baseline** for your improvements. | §4 |
| **A** Calibration | choice | updated `vision.py`; calibration parameters with frame and unit conventions; annotated image with detected and reprojected marker corners; table of validation errors (reprojection px mean/max; position mm mean/max at measured points incl. edges; consistency over several images) | §6: `lab9/calib/camera_calibration.yaml`, `annotated_*.png`, `extrinsics_report.md`, `lab9/tables/position_errors_*.csv` |
| **B** Robust HSV | choice | updated `vision.py`; evaluation: several positions and orientations, ≥ 2 lightings, shadows/reflections, a frame with a removed cube; original vs improved masks; false and missed detections; reasons for your thresholds and filters | §7: `lab9/eval_B/` |
| **C** HSV from examples | choice | the program; evaluation: calibrate on 1 image, test under ≥ 2 lightings, which pixels are wrongly in/out, compare with the fixed `vision.py` thresholds, does median ± 2σ need a minimum width | §8: `hsv_calibrate.py`, `lab9/hsv/`, `lab9/eval_C/` |
| Both tasks | yes | "document and evaluate", compared with the Part 0 baseline | §9 |

Checklist (tick as you go):
- [ ] Part 0 report + data-flow list + baseline tables
- [ ] Task 1 code + evaluation
- [ ] Task 2 code + evaluation
- [ ] Before/after comparison (§9)

---

## 2. The system in plain words

### 2.1 Five programs, one data flow
```
Lab PC                                                                                   Pi
camera node ── camera/image (topic, ~10 Hz) ──> vision ──┐
(opencv_camera)                                          │ /cube_coordinates (service: colour -> x, y, z, roll, pitch, yaw)
                                                         v
                    brain (menu) ── /move_action (action) ─────────> MoveIt move_group ── /arm_group_controller/
                       │        ── /compute_cartesian_path (service) ─>      │             follow_joint_trajectory ──> controller ─ serial ─> arm
                       │        ── /arm_group_controller/follow_joint_trajectory (action, for straight lines) ───────> controller
                       │        ── /collision_object, /attached_collision_object (topics) ─> MoveIt planning scene
                       └─────── /pump_controller (topic: "on"/"off") ─────────────────────────────────────────────> controller ─ GPIO 20 ─> pump
                                                                    MoveIt <── /joint_states (topic, 20 Hz) ──────── controller
```
The PDF uses partly different names; the code is what runs:

| PDF | Code |
|---|---|
| `/image` | `camera/image` (`opencv_camera.py`, `vision.py`) |
| `/get_cube_coords` | `/cube_coordinates` (`vision.py`, `brain.py:25`) |
| `/joint_state_publisher` | `/joint_states` (`controller.py:19`) |

### 2.2 Words used below
| Word | Meaning here |
|---|---|
| node | one running ROS program (vision, brain, ...) |
| topic | a one-way stream of messages (images, pump commands). Anyone can publish or listen. |
| service | a request with exactly one answer (brain asks vision "where is red?") |
| action | a long request with feedback and a final result (move the arm along a trajectory) |
| planning scene | MoveIt's model of the world: robot, table, bins, cubes as boxes; plans avoid collisions with it |
| HSV | hue (which colour, 0–179 in OpenCV), saturation (how strong, 0–255), value (how bright, 0–255) |
| mask | a black/white image: white (255) = pixel accepted, black (0) = rejected |
| contour | the outline of one white region of a mask, as a list of pixel points |
| intrinsics K, d | how the camera turns a direction into a pixel (focal length, image centre) and how the lens bends it (distortion) |
| extrinsics R, t | where the camera is and how it is turned, relative to the robot base frame B |
| ArUco marker | a black-and-white square with an id; OpenCV finds its 4 corners to sub-pixel accuracy |
| reprojection error | distance in pixels between a detected point and where the camera model predicts it |

### 2.3 One sort cycle, as the code does it
1. Menu `2` → `choose_pick()` (`brain.py:413`) removes and re-adds all cubes in the planning scene: each `spawn_cube` asks vision once (`get_cube_coords`, `brain.py:56`).
2. You type a colour → `pick()` (`brain.py:327`) asks vision again for that colour.
3. Hover: goal = (x, y, **z + 0.10**), pump pointing down (`brain.py:337–339`), planned by MoveIt (`send_goal_pose`).
4. Descend 5 cm in a straight line (`brain.py:342`, `send_cartesian_path`): the target for the link `pump_head` is z + 0.05.
5. Pump on (`brain.py:346`), the cube is attached to `pump_head` in the planning scene.
6. Retract 10 cm, then home `[0.16, -0.06, 0.32]` (`brain.py:322`).
7. You type a bin → `drop()` (`brain.py:393`): hover over the bin at 0.15 m, pump off, the cube falls in, home.
8. `exit` → pump off, home, menu again (`choose_pick` calls `control_menu()` itself, `brain.py:413–449`).
   **Rejoice** (`brain.py:406`, `controller.py:94–100`: all joints to 0 = arm straight up, then back) runs after a **stacking** run (`brain.py:522`). The sort-mode call (`brain.py:526`) is never reached by the code as written, because the menus call themselves instead of returning **[code reading; check by watching]**. Keep the space above the arm clear in both modes anyway (the run guide expects rejoice after every run).

Vision answers once per request; there is no feedback while the arm moves (the PDF calls this single-measurement, not closed-loop, visual servoing).

---

## 3. Safety

Read the run guide §0 (eight surprises) once. The ones that matter most here:
- ⚠ **Starting `brain` moves the arm** to home before the menu appears (`brain.py:509`). Start it last.
- ⚠ **After a stacking run (and possibly after sorting, see §2.3 step 8) the arm goes straight up (all-zero angles) and back** ("rejoice", `controller.py:94–100`). Keep the space above the arm clear. Never publish on `/rejoice` yourself.
- ⚠ **Ctrl+C in brain does not stop a motion already sent**; the controller has no cancel. **The power switch is the stop.**
- ⚠ **Stacking with a missing cube can press one cube onto another** **[code reading, check by observation with the hand at the switch]**: if green was picked but red is not detected, `place()` returns without releasing (`brain.py:365–366`), so green stays on the pump; the next `pick("blue")` then descends onto blue with green still attached. Before menu `1`, check in the vision window that red, green **and** blue all have a rectangle.
- ⚠ A cube still on the pump when a stacking run ends is carried through rejoice and released when the menu restarts (`control_menu` switches the pump off first, `brain.py:508`): it can fall from high up **[code reading]**.
- The scripts in this folder never move the robot: they only read images, call `/cube_coordinates`, or work on files.

---

## 4. Part 0: explore the existing system (required, the baseline)

Do all of Part 0 with the **unchanged course code**. Your improvements are judged against what you record here, so record it carefully and keep the same cube positions for later.

### P0. Choose and mark five test positions
**Goal:** five positions you can reproduce in every later test.
**Do:** mark them on the table with small pieces of tape **next to** the cube footprint (tape under a cube changes its colour in the image). Suggested positions in the base frame B (x forward from the robot base centre, y to the robot's left), in mm: `(150, 0)`, `(100, -50)`, `(200, 50)`, `(220, -70)` (edge), `(90, 70)` (edge). Adjust to your tray, then never change them.
**How to measure in B:** stick one tape line on the table from the base centre straight forward (that is the x axis); measure x along it and y at right angles with a set square. **[check]** that the base frame origin `g_base` is at the centre of the base (the run guide states it, the URDF was not checked).
**Record:** the five positions, a photo of the setup.

### P1. Prepare the setup (PDF item 1)
**Do:** run guide 6b steps **B1–B5** (controller on the Pi → `/joint_states` check → MoveIt → camera → vision). No brain yet, no motion yet. Then run guide **D2** (image is 640×480).
**Expect:** the vision window shows a rectangle on each cube and the HSV values under the mouse.
**If not:** a cube without a rectangle is not detected; blue often fails because the original code needs S and V ≥ 200 (`vision.py`, colours dictionary). Use red for the first tests. Lighting is Task B's topic; note it as a limitation.
**Record:** which colours are detected, in which light.

### P1b. Baseline accuracy of the vision service (no motion)
**Goal:** how far the original pixel → robot mapping is off, in mm. This is the number Task A must beat, and it tells you whether picks can work at all.
**Do (Lab PC, new terminal):**
```bash
cd ~/mycobot-project && source ~/rosenv9.sh
python3 solutions/lab9_real_hardware/position_errors.py baseline --color red
```
For each of your five positions: put the red cube there, type the measured `x y` in mm (e.g. `150 0`, `220 -70 edge`), Enter. Empty line = done.
**Expect:** one line per position with vision's answer and the error; at the end a table with mean and max error.
**If not:** "`/cube_coordinates` not available" → vision is not running or `rosenv9.sh` was not sourced in this terminal.
**Record:** `lab9/tables/position_errors_baseline.csv` and the printed table. Mean dx/dy far from zero = the whole mapping is shifted (camera moved); errors growing towards the edges = wrong scale.
⚠ If the errors are larger than about 10 mm, the pump will land off the cube top. Run P2 only with a cube at the position with the smallest error, or note that picks are expected to fail.

**Understand: `position_errors.py`**
```python
client = node.create_client(GetCubeCoords, "/cube_coordinates")   # a service client, like brain.py:25
request = GetCubeCoords.Request(); request.color = color           # the request has one field: color (GetCubeCoords.srv)
future = client.call_async(request)                                 # send, do not wait yet
rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)     # let ROS work until the answer arrived (or 3 s)
coords = list(future.result().coords)                               # [x, y, z, roll, pitch, yaw] in m and rad
```
- `call_async` returns a *future*: a placeholder that will hold the answer. `spin_until_future_complete` runs the node's message handling until it is filled. brain uses exactly this pattern (`brain.py:56–62`).
- Vision answers `[1.0]*6` when the colour is not detected; the script records that as "not detected".
- `dx, dy = xr - xm, yr - ym` and `np.hypot(dx, dy)` = √(dx² + dy²), the distance on the table in mm. Example: vision (158, 4), ruler (150, 0) → dx = 8, dy = 4, error = 8.9 mm.

### P2. Sorting mode (PDF item 2) [MOVES]
**Do:** run guide **B6** (start brain last; it moves home first). Menu `2` → `r` → wait until the cube is lifted → bin `A` → then another cube, e.g. green → bin `C`. `exit` returns to the menu (pump off, home). Use at least two cubes and two different bins. Then do one pick at **each of the five positions** with the red cube (same order as P1b), bin A.
**Watch:** hover, descent, pump sound, lift, transport, release height, return home. Where does the pump touch the cube (centre, edge, miss)?
**Record (per pick):**

| # | colour | position (mm) | bin | picked? | dropped in bin? | where the pump touched | notes |
|---|---|---|---|---|---|---|---|

### P3. Stacking mode (PDF item 3) [MOVES]
**Do:** red, green, blue on the table, all three with a rectangle in the vision window. Menu `1`.
**Expect (code, `brain.py:476–503`):** green is picked and placed on red, then blue is picked and placed **on green** (yellow is commented out). The PDF says blue goes on red if green could not be stacked; the code always targets green: compare what you see with both.
**Then (carefully):** test one missing cube the safe way: remove **blue** (not red) and run menu `1` again: `pick("blue")` returns `False` and nothing is held. ⚠ Do not remove red (§3: green would stay on the pump and be pressed onto blue).
**Record:** order of picks, success, what happened with the missing cube (log lines in the brain terminal, e.g. `blue cube not detected!`).

### P4. Trace the information flow (PDF item 4)
**Do (Lab PC, all nodes running, brain waiting in the menu; nothing here moves):**
```bash
source ~/rosenv9.sh
ros2 node list                     # expect: /vision /image_publisher /mycobot_brain /move_group /controller (+ MoveIt helpers)
ros2 topic list -t                 # name [type]: camera/image [sensor_msgs/msg/Image], /pump_controller [std_msgs/msg/String], ...
ros2 service list -t | grep -v parameter     # /cube_coordinates [mycobot_interfaces/srv/GetCubeCoords], /compute_cartesian_path ...
ros2 action list -t                # /move_action [moveit_msgs/action/MoveGroup], /arm_group_controller/follow_joint_trajectory ...
ros2 node info /mycobot_brain      # which topics it publishes, which services and actions it uses
ros2 topic hz /joint_states        # ~20 Hz from the Pi (Ctrl+C to stop)
ros2 topic hz /camera/image        # ~10 Hz expected [check]: the camera node reopens the camera for every frame (opencv_camera.py)
```
During one pick (P2), in another terminal: `ros2 topic echo /pump_controller` shows `data: on`, later `data: off`.
**Record:** the data-flow list for one pick (the deliverable). Template:
1. `opencv_camera` publishes a BGR image on `camera/image` (10 Hz).
2. `vision` converts to HSV, makes one mask per colour, finds square contours, keeps the centre `(cx, cy)` and angle per colour.
3. `brain` asks `/cube_coordinates` with a colour; `vision` maps `(cx, cy)` to `(x, y)` with fixed constants (`vision.py:44–49`) and answers `[x, y, 0.01, 0, 0, yaw]`.
4. `brain` adds the cubes to MoveIt's planning scene (`/collision_object`), then sends the hover pose as a goal on `/move_action`; MoveIt plans and sends the trajectory to the controller (`follow_joint_trajectory`).
5. `brain` gets a straight-line descent from `/compute_cartesian_path` and sends it to the controller itself (same action).
6. `brain` publishes `on` on `/pump_controller`; the controller sets GPIO 20 low; the cube is attached to `pump_head` (`/attached_collision_object`).
7. Retract, home, bin, `off`, home (as 3–6). The controller publishes `/joint_states` at 20 Hz all the time; MoveIt plans from it.

### P5. Document (PDF item 5)
Report skeleton: setup photo + five positions · P1b table · P2 and P3 tables · data-flow list · **at least two limitations**. Candidates (confirm each by what you saw, do not just copy):
- fixed pixel → robot constants from an old camera pose (P1b errors) → Task A;
- fixed z = 0.01 for every object; one object per colour; the id is the colour;
- detections stay after a cube is removed (`vision.py:133` only adds) → Task B;
- thresholds: blue needs S, V ≥ 200; red's hue upper limit 200 is outside OpenCV's 0–179 → Tasks B, C;
- rejoice (all-zero angles) after a stacking run; recursive menus (`choose_pick` and `control_menu` call themselves, so the sort-mode rejoice at `brain.py:526` is never reached and every choice adds a stack frame);
- stacking does not release a held cube when the next step fails (§3);
- open-loop: one measurement, no correction while descending.
More: run guide §11.

---

## 5. Recording real frames (both approaches)

**Goal:** a set of real images you can analyse at home, repeat, and show in the report.
**Do (Lab PC, camera node running, run guide B4):**
```bash
cd ~/mycobot-project && source ~/rosenv9.sh
python3 solutions/lab9_real_hardware/record_frames.py light1_cubes      # s = save, q = quit
```
Record these folders (the name is the argument):

| Folder | What is in the picture | Frames | For |
|---|---|---|---|
| `checkerboard` | the checkerboard in many places: centre, all four corners of the image, tilted ±30° in both directions, nearer and farther | 15–20 | A (TA1) |
| `markers` | both ArUco markers fully visible, nothing on them, the camera in its final position | 5 | A (TA3, TA6) |
| `light1_cubes` | room light; the four cubes at your five positions, different orientations, some near the edges | 8–10 | B, C |
| `light2_lamp` | a second lighting: a desk lamp from one side, or room lights dimmed | 8–10 | B, C |
| `shadow_reflection` | a hand shadow across a cube; a phone torch reflecting on a cube; a cable or another coloured object in view | 5 | B |
| `removed_cube` | same scene: 3 frames with the red cube, then take it away, 3 more frames (in this order) | 6 | B (TB4) |

⚠ **The camera must not move** after the `markers` frames. If it is bumped, record `markers` again and redo TA3. 
⚠ **Autofocus** changes the intrinsics. For Task A, switch it off before `checkerboard` and keep it off **[check the control names]**:
```bash
v4l2-ctl -d /dev/video0 --list-ctrls | grep -i focus            # look for focus_automatic_continuous (or focus_auto) and focus_absolute
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0 -c focus_absolute=0
```
**Record:** the folder names and the lighting of each (it goes into the B/C evaluation).

**Understand: `record_frames.py`**
```python
self.create_subscription(Image, "camera/image", self.on_image, 1)   # same topic and queue size as vision.py
self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")                 # ROS Image -> NumPy array (rows, cols, 3), BGR order
rclpy.spin_once(node, timeout_sec=0.05)                             # handle at most one incoming message, then return
key = cv2.waitKey(1) & 0xFF                                         # also lets OpenCV draw the window; & 0xFF keeps the key code
cv2.imwrite(str(path), node.frame)                                  # save the original frame, not the copy with text
```
`spin_once` in a `while` loop (instead of `rclpy.spin`) is how one program both listens to ROS and handles a window with key presses.

---

## 6. Task A: camera-to-robot calibration with two ArUco markers (approach 1)

**What the PDF asks (pp. 14–16):** replace the fixed constants in `send_cube_coords()` by a calibrated camera model: (1) intrinsics K, d; (2) marker corner coordinates in B; (3) camera pose with `cv2.solvePnP` from the 8 corners, inverted, checked against the camera height; (4) pixel → ray → plane intersection; (5) review z and yaw; (6) validate (reprojection, position errors, consistency).

**The idea in one picture:** the camera turns a 3D point into a pixel. If you know *how* (K, d) and *from where* (R, t), you can go back: a pixel gives a ray from the camera; where that ray hits the plane of the cube tops is the cube's position.
```
pixel (u, v) --K, d--> ray in camera frame --R_BC--> ray in robot frame --hits z = z_top--> (x, y, z_top)
```

### TA1. Intrinsics K and d
**Goal:** the camera's focal length, image centre and lens distortion, for exactly the 640×480 images vision gets.
**Why:** a pixel is only a direction once you know K. Distortion bends straight lines near the image edges; ignoring it gives errors of millimetres to centimetres at the edges.
**Do (laptop or Lab PC, after recording `checkerboard`):**
```bash
python3 solutions/lab9_real_hardware/camera_intrinsics.py lab9/frames/checkerboard/*.png --cols 9 --rows 6 --square 0.025
```
`--cols/--rows` = **inner** corners (a board of 10 × 7 squares has 9 × 6); `--square` = side of one square in metres, measured on the print or screen.
**Expect:** `used 15 images, RMS reprojection error 0.3 px` (< 0.5 good, < 1.0 acceptable); `K` with fx ≈ fy (a few hundred pixels) and cx ≈ 320, cy ≈ 240; `lab9/calib/intrinsics.yaml`; drawings of the found corners in `lab9/calib/intrinsics_check/`; `undistorted_example.png` with straight edges.
**If not:** "board not found" for many images → wrong `--cols/--rows`, board too small or blurred. One image with a high error → delete it and run again. Fewer than 10 usable images → record more.
**No checkerboard at all:** `--from-fov 90` (the C930e's diagonal field of view) gives a rough K without distortion. ⚠ Write "approximate intrinsics" in the report: the edge errors will be larger.
**Record:** `intrinsics.yaml` (K in pixels, d dimensionless, OpenCV model `[k1, k2, p1, p2, k3]`), RMS.

**Understand: `camera_intrinsics.py`**
The pinhole model, in code (what K means):
```python
u = fx * X / Z + cx      # X, Y, Z: point in the camera frame (Z = distance along the optical axis)
v = fy * Y / Z + cy      # example: fx = 550, point 0.1 m right at 0.5 m depth -> u = 550 * 0.2 + 320 = 430
```
```python
found, corners = cv2.findChessboardCorners(gray, (9, 6), flags)    # (54, 1, 2) pixel corners, row by row
corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)   # refine each to ~0.1 px in an 11x11 window
pts[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2) * 0.025          # the same corners on the flat board, in metres
```
- `np.mgrid[0:9, 0:6]` makes two 9×6 grids of the column and row indices; `.T.reshape(-1, 2)` turns them into 54 pairs `(0,0), (1,0), (2,0) ... (8,5)`, in the order the detector returns the corners; `* 0.025` converts to metres. z stays 0 (the board is flat).
- `rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, (640, 480), None, None)` finds K, d **and** the board pose of every image so that projected board corners land on the detected ones. `rms` is the remaining mismatch in pixels.
- Python syntax: `a, b, c = f()` unpacks a tuple; `for o, i, r, t in zip(...)` walks four lists in step.

### TA2. Marker geometry in frame B
**Goal:** the 3D coordinates in B of all 8 marker corners, in the order OpenCV reports them.
**Do:** fill in `solutions/lab9_real_hardware/markers.yaml` (every `TODO`): dictionary (e.g. `DICT_4X4_50`), side length of the black square, each marker's id, centre `x, y` in B, and `yaw_deg`; the marker plane height `plane_z` and the camera height `camera_height` above that plane (tape measure from the lens to the table).
How `yaw_deg` is defined (seen from above, pattern readable):
```
            +x_B (away from the robot)                     yaw_deg = 0:   marker x (TL->TR) along +x_B,
               ^                                                          marker "up" (top edge) towards +y_B
               |                                           yaw_deg = 90:  marker x along +y_B,
   +y_B <------+  robot base (origin of B)                                marker "up" towards -x_B (to the robot)
```
**Expect:** later, in TA3, the annotated image shows the red reprojected crosses on the green detected corners. If they are swapped or rotated, an id or a yaw is wrong.
**Record:** `markers.yaml` with real values (it is part of "calibration parameters with explicit conventions").

**Understand: `marker_corners_B()` in `aruco_extrinsics.py`**
```python
s = size / 2
local = np.array([[-s, s], [s, s], [s, -s], [-s, -s]])          # TL, TR, BR, BL around the marker centre
Rz = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])  # 2D rotation by the yaw a (radians)
xy = local @ Rz.T + [marker["x"], marker["y"]]                    # rotate every corner, then shift to the centre
return np.column_stack([xy, np.full(4, plane_z)])                 # add z = plane height -> (4, 3)
```
Worked example: marker at (0.10, 0.12), size 0.04, yaw 0 → corners `(0.08, 0.14)`, `(0.12, 0.14)`, `(0.12, 0.10)`, `(0.08, 0.10)`. With yaw 90° the same marker gives TL = `(0.08, 0.10)`: the pattern's top edge now faces the robot.
- `@` is matrix multiplication; `local @ Rz.T` rotates all 4 rows at once (each row `p` becomes `Rz p`).
- `+ [x, y]` is added to every row (NumPy *broadcasting*).
- The PDF's note: two marker **centres** are not enough to fix the camera's tilt; 8 corners are.

### TA3. Camera pose (extrinsics) from the markers
**Goal:** R_BC, t_BC: how the camera is turned and where it is, in B.
**Why:** this is what changed since the course constants were measured (PDF p. 15).
**Do:** record `markers` (§5), then:
```bash
python3 solutions/lab9_real_hardware/aruco_extrinsics.py lab9/frames/markers/*.png
```
**Expect:** a table per image (reprojection mean/max px, camera position in mm, angle to the final pose), then
`Final ... reprojection mean 0.3 px`, `camera at (x, y, z) mm in B`, a spread of a few mm at most, and `Height check: estimated 452 mm ..., measured 450 mm, difference 2 mm`. Files: `lab9/calib/camera_calibration.yaml`, `annotated_*.png`, `extrinsics_report.md`.
Open an annotated image: green circles (detected) and red crosses (reprojected) should coincide; the red `x_B` arrow must point away from the robot, the green `y_B` arrow to the robot's left; the blue frame is your workspace at cube-top height and should cover the tray, not the bins.
**If not:**
- `marker(s) [..] not found` → dictionary wrong, marker covered or too small: the image is skipped. **This is also the answer to "how the node behaves if a marker is missing":** the calibration is done offline from frames with both markers; an image with one marker is skipped, and if no image is usable nothing is written and the previous `camera_calibration.yaml` stays. The vision node only loads that file, so a cube covering a marker during a run does not matter.
- height difference > 1 cm or reprojection max > 2 px (warnings are printed) → wrong marker size, wrong id/yaw/position in `markers.yaml`, or the camera moved between frames.
- arrows point the wrong way → the B axes in your measurements do not match the robot's: recheck P0.
**Record:** `camera_calibration.yaml`, one annotated image, `extrinsics_report.md` (reprojection + consistency part of the TA6 table).

**Understand: `aruco_extrinsics.py`**
Detection, written for both OpenCV versions (4.6 on Ubuntu 24.04 has the old functions):
```python
dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, "DICT_4X4_50"))   # getattr: constant from its name
if hasattr(cv2.aruco, "ArucoDetector"):                                             # OpenCV >= 4.7
    corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
else:                                                                               # OpenCV 4.6
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
return {int(i): c.reshape(4, 2) for i, c in zip(ids.ravel(), corners)}            # dict comprehension: id -> 4 corners
```
Pose: `solvePnP` finds R_CB, t_CB with `p_C = R_CB p_B + t_CB` (B → camera) so that the 8 known corners project onto the 8 detected ones.
```python
_, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
```
- All 8 corners lie in one plane. For a flat target there are usually **two** poses that explain the image almost equally well (seen from above-left or mirrored). IPPE returns both; the code keeps the one with the camera **above** the plane, looking **down**, and with the height closest to your measured `camera_height` (PDF: "select a pose consistent with the physical camera placement").
- `cv2.solvePnPRefineLM` then polishes that pose.
- `rvec` is a rotation written as an axis times an angle (3 numbers); `cv2.Rodrigues(rvec)` turns it into the 3×3 matrix R_CB.

Inversion (PDF formulas), in code:
```python
R_BC = R_CB.T                       # a rotation's inverse is its transpose
t_BC = -R_BC @ t_CB                 # the camera centre in B (the point that maps to p_C = 0)
height = t_BC[2] - plane_z          # compare with your tape measure
looks_down = (R_BC @ [0, 0, 1])[2] < 0    # the camera's optical axis, written in B, must point downwards
```
Example: camera straight above (0.15, 0, 0.45) looking down → `t_BC = (0.15, 0, 0.45)`, height 0.45 m.
Final pose: the camera and markers did not move between the frames, so all corners of all frames are stacked and solved once (averages detection noise). The per-image poses are kept only for the consistency table.

### TA4. From pixel to robot coordinates (the new `send_cube_coords`)
**Goal:** replace `vision.py:44–49` by a function that uses the calibration.
**Why the cube-top plane:** the camera sees the cube's **top face**, 4 cm above the table. Intersecting with the table plane instead would shift every position away from the camera's foot point (the PDF warns about this).
**Understand: `pixel_to_base()` in `vision_AB.py`**
```python
xn, yn = cv2.undistortPoints(pixel, K, dist)[0, 0]   # 1. remove distortion and divide by f: normalised coordinates
r_C = np.array([xn, yn, 1.0])                         # 2. ray direction in the camera frame (z = forward)
r_B = R_BC @ r_C                                      # 3. the same direction in robot axes
lam = (z_plane - t_BC[2]) / r_B[2]                    # 4. how far along the ray the plane z = z_plane is
if lam <= 0: return None                              #    plane behind the camera: reject
return t_BC + lam * r_B                               # 5. the point: camera centre + lam * direction
```
Worked example (camera straight down at (0.15, 0, 0.45), image right = robot's right, f = 550, centre (320, 240), cube tops at z = 0.04):
pixel (430, 240) → `xn = (430 - 320)/550 = 0.2`, `yn = 0` → `r_B = (0, -0.2, -1)` → `lam = (0.04 - 0.45)/(-1) = 0.41` → point `(0.15, -0.082, 0.04)`: 110 px to the right is 8.2 cm to the robot's right.
- `undistortPoints` needs the shape `(1, 1, 2)` (a list of points), so the code builds `np.array([[[u, v]]])` and takes `[0, 0]` from the result.
- **Distortion (PDF TA1 question):** it is removed per point by `undistortPoints` before the ray is formed; the image itself is not undistorted (cheaper, same result for the centre).
- **Rejections:** `lam <= 0` (behind the camera) or a point outside the workspace rectangle / beyond 0.28 m reach (`in_workspace`, limits in `markers.yaml`) → the service answers "not detected" (`[1.0]*6`), so brain does not move to a wrong place.

### TA5. Review z and yaw
**z (PDF: "is the fixed z appropriate?"):** brain adds +0.10 for the hover and −0.05 for the descent to whatever z vision returns (`brain.py:337, 342`). The course tuned this chain with z = 0.01: hover at 0.11, descent target for `pump_head` at 0.06. Returning the physical top height (0.04) without changing brain would stop the pump 3 cm higher. Decision in `vision_AB.py`: **x and y from the true top plane, returned z stays `Z_BRAIN = 0.01`**, with this reason in the report. **[check]** in the lab: at the first pick after installing, watch whether the pump reaches the cube top exactly as before.
**yaw:** `cv2.minAreaRect` gives an angle in **pixel** axes; the robot needs it in **B** axes. The code maps the centre and a point 20 px along one rectangle side into B and takes the direction:
```python
yaw = np.arctan2(p1[1] - p0[1], p1[0] - p0[0])          # direction of the side in B, radians
return (yaw + np.pi / 4) % (np.pi / 2) - np.pi / 4      # fold into [-45 deg, 45 deg)
```
A square looks the same after 90°, so 70° and −20° are the same cube orientation; the fold picks the smaller wrist turn. Examples: 70° → −20°, −50° → 40°, 20° → 20°. (Python's `%` always returns a value with the sign of the divisor: `-5 % 90 == 85`.)

### TA-install. Put the calibrated vision into the package
**Learn mode (recommended):** open the course file and the reference side by side and type the changes yourself:
```bash
cd ~/mycobot-project
diff -u pp_moveit_ws/src/mycobot_vision/mycobot_vision/vision.py solutions/lab9_real_hardware/vision_AB.py | less
```
**Fast mode:** copy the whole reference:
```bash
cp solutions/lab9_real_hardware/vision_AB.py pp_moveit_ws/src/mycobot_vision/mycobot_vision/vision.py
git diff --stat                     # vision.py shows as changed; `git checkout -- <file>` restores the course version
```
No rebuild should be needed, because the workspace was built with `--symlink-install` (run guide A2) **[check]**: `ls -l pp_moveit_ws/build/mycobot_vision/mycobot_vision/vision.py` should be a link to `src`. If not: `colcon build --symlink-install --packages-select mycobot_vision`.
Restart vision (Ctrl+C in its terminal):
```bash
ros2 run mycobot_vision vision                     # loads ~/mycobot-project/lab9/calib/camera_calibration.yaml
ros2 run mycobot_vision vision --ros-args -p calib_file:=/path/to/camera_calibration.yaml   # another file
```
**Expect:** log `Camera calibration loaded from ...`; in the window each cube is labelled with `x=.. y=.. mm`; the white workspace polygon.
**If not:** `No camera calibration (...)` → wrong path; every request answers "not detected" (safe). `Image is (w, h), calibration is for ...` → the camera resolution changed: recalibrate.

### TA6. Validate
**Do (no motion):** with the calibrated vision running, the same five positions as P1b:
```bash
python3 solutions/lab9_real_hardware/position_errors.py calibrated --color red
```
Add 3 more positions near the edges of the workspace (PDF: "including positions near its edges"), with the note `edge`.
**Expect:** mean error a few mm, clearly below the baseline. As a rough target (my estimate, not from the PDF): ≤ 5 mm mean.
**If not:** a constant offset (mean dx or dy far from 0) → B origin or marker positions measured wrong; errors growing towards the edges → intrinsics (redo TA1 with more edge images); one bad point → measurement or detection error at that point.
**Record — the validation table (deliverable):**

| | baseline (Part 0) | calibrated |
|---|---|---|
| reprojection mean / max px (8 corners) | — | from `extrinsics_report.md` |
| position error mean / max mm (5 + 3 edge points) | `position_errors_baseline.csv` | `position_errors_calibrated.csv` |
| consistency: spread of the camera position over 5 frames (mm) | — | from `extrinsics_report.md` |
| camera height: estimated vs measured | — | from `extrinsics_report.md` |

Picks with the calibrated vision: §9.

---

## 7. Task B: robust HSV cube detection (both approaches)

**What the PDF asks (pp. 16–18):** (1) check and calibrate the thresholds (hue only 0–179); (2) clean each mask with morphology, justify the kernel; (3) reject unreliable candidates (area, filled fraction, workspace), keeping the size and aspect checks; (4) no stale detections. Evaluate on several positions, ≥ 2 lightings, shadows/reflections, a removed cube; show original vs improved masks.

All changes are in the block `# ---- Task B` of `vision_B.py` / `vision_AB.py` (identical in both) plus two lines in `img_callback`. The detection is a plain function `detect_cubes(bgr)` without ROS, so the node and `evaluate_detection.py` run the same code.

### TB1. Thresholds
**Goal:** HSV limits that accept your cubes and reject the table, in your light.
**Why:** the course limits were set elsewhere. Two are plainly wrong: red's second range goes up to hue **200**, but OpenCV hue stops at 179 (8-bit images store degrees / 2); blue needs S and V ≥ 200, which normal room light rarely gives.
**Do:** two ways, use either:
- *Mouse overlay (approach 1):* in the vision window, move the mouse over each cube (centre, edges, shadowed side) and over the table; the bar shows `H S V` and the pixel. Write down min/max per colour.
- *Task C program (approach 2):* §8 gives the limits directly from an outlined cube.
Then edit the `COLORS` dictionary in your `vision.py`. Rule of thumb: hue ±8 around the measured centre, S and V lower limit ~30 below the lowest value you saw on the cube, but above the table's values.
**Record:** a small table per colour: H, S, V measured on the cube (light 1 and light 2) and on the table; the chosen limits.

**Understand:**
```python
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)          # BGR -> HSV; H 0..179, S 0..255, V 0..255
mask = cv2.inRange(hsv, lower, upper)               # 255 where lower <= pixel <= upper in all three channels
mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower2, upper2))   # red: two hue ranges, 0..15 and 170..179
```
Pure colours in OpenCV hue: red 0, yellow 30, green 60, blue 120 (hue = degrees / 2). The course's blue range 81–110 suggests its "blue" cubes are a light, cyan-ish blue: measure yours.
`COLORS["red"]["ranges"]` is a list of `(lower, upper)` tuples; `color_mask()` ORs all of them.

### TB2. Clean the masks
**Goal:** remove isolated noise pixels and close small holes before looking for contours.
**Why:** a single noise pixel next to a cube changes its outline; a hole from a reflection splits it; noise blobs create false candidates.
**Understand (opening and closing on tiny masks, 1 = white, 3×3 kernel):**
```
opening removes a speck:          closing fills a hole:
0 0 0 0 0      0 0 0 0 0          1 1 1 1 1      1 1 1 1 1
0 0 1 0 0  ->  0 0 0 0 0          1 1 0 1 1  ->  1 1 1 1 1
0 0 0 0 0      0 0 0 0 0          1 1 1 1 1      1 1 1 1 1
```
Erode = a pixel stays white only if the whole kernel around it is white (shrinks regions, specks vanish). Dilate = a pixel becomes white if any pixel under the kernel is white (grows regions, holes close). Opening = erode then dilate (big regions get their size back, specks do not come back); closing = dilate then erode.
```python
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))   # a 5x5 disc
mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)            # anything thinner than ~5 px disappears
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)           # gaps narrower than ~5 px close
```
**Why 5×5 and an ellipse:** a cube top is 50–150 px wide, so a 5 px disc removes noise but shifts the outline by under a pixel and the centre not at all; the round shape treats all directions equally (a square kernel rounds corners less evenly). Open first so noise is removed before closing could join it to a cube.
**Do (experiment):** `ros2 run mycobot_vision vision --ros-args -p show_masks:=true` opens a second window with the cleaned masks. Try `(3, 3)`, `(5, 5)`, `(9, 9)` in `KERNEL` (restart vision each time): watch whether the cube stays one solid blob and whether its centre moves.
**Record:** kernel choice and why (with a mask screenshot).

### TB3. Reject unreliable candidates
**Goal:** only solid, square regions inside the tray count as cubes.
**Understand: `is_cube()`**
```python
(cx, cy), (w, h), angle = rect                      # cv2.minAreaRect: centre, side lengths, angle (tuple unpacking)
if not (SIDE_MIN <= w <= SIDE_MAX and ...): return False    # original: 50..150 px
if min(w, h) / max(w, h) < ASPECT_MIN: return False         # original: square-ish, >= 0.75
area = cv2.contourArea(contour)                     # area enclosed by the outline
if area / (w * h) < FILL_MIN: return False          # new: the outline fills its rectangle (>= 0.80)
inside = np.zeros_like(mask)
cv2.drawContours(inside, [contour], -1, 255, -1)    # the outline, filled (-1 = fill)
if cv2.countNonZero(cv2.bitwise_and(mask, inside)) / area < SOLID_MIN: return False   # new: really solid (>= 0.85)
if workspace_px is not None and cv2.pointPolygonTest(workspace_px, (cx, cy), False) < 0: return False   # new
```
- **Fill** catches L-shapes, notched or merged blobs: a 60 × 60 square has area 3600 in a 3600 rectangle → 1.0; the same square with a 30 × 30 corner missing has 2700 / 3600 = 0.75 → rejected (limit 0.80).
- **Solidity** is needed because `cv2.contourArea` counts a **hole as area**: a red frame 90 × 90 px with a 4 px border has fill ≈ 1.0, but only ≈ 1400 of its 8100 px are white → solidity 0.17 → rejected. (A large glare hole on a real cube can also lower it: if a cube is lost under a lamp, look here first.)
- **Workspace:** `pointPolygonTest` returns +1 inside, 0 on the edge, −1 outside. Approach 1: the polygon is your `markers.yaml` workspace projected into the image (`workspace_polygon_px`, the blue/white frame you saw). Approach 2: read 4 corners of the tray with the overlay (it now shows `px=(x,y)`) and set `WORKSPACE_PX = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], np.int32)`. Keep the bins **outside**, or a sorted cube is detected again.
- The PDF also mentions a minimum contour area: with `SIDE_MIN = 50` and `FILL_MIN = 0.8` the area is at least 0.8 · 50² = 2000 px² anyway, so a separate area limit would never act.

### TB4. No stale detections
**Goal:** a cube that is taken away is reported as "not detected" at the next frame.
**Why:** the original only **adds** to the dictionary (`vision.py:133`); a removed red cube stays at its old position forever, and brain would try to pick air.
```python
d = {}
d["red"] = (300, 200)    # frame 1: red seen
                         # frame 2: red removed, nothing assigned -> d still contains red   (original behaviour)
found = {}               # improved: a NEW dictionary for every frame ...
self.detected_cubes = found   # ... replaces the old one after all colours were checked -> red is gone
```
**Do (live check, no motion):** vision running, red cube in view: `ros2 service call /cube_coordinates mycobot_interfaces/srv/GetCubeCoords "{color: red}"` → coordinates. Remove the cube, call again → `coords=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0]`.
**Record:** both answers (original vision: the second call still returns coordinates).

### TB-install
Approach 2: as TA-install, with `vision_B.py`. Approach 1: `vision_AB.py` already contains B. Then paste your TB1 thresholds into `COLORS` and, for approach 2, your `WORKSPACE_PX`.

### TB-eval. Evaluation (deliverable)
**Do:**
```bash
# 1. once per folder (any machine with a display): click each cube centre, press its colour key r/y/g/b; n = next frame
python3 solutions/lab9_real_hardware/evaluate_detection.py label lab9/frames/light1_cubes
python3 solutions/lab9_real_hardware/evaluate_detection.py label lab9/frames/light2_lamp       # also shadow_reflection, removed_cube
# 2. Lab PC (imports vision_B.py, needs ROS): original vs improved
source ~/rosenv9.sh
python3 solutions/lab9_real_hardware/evaluate_detection.py run lab9/frames/light1_cubes lab9/frames/light2_lamp lab9/frames/shadow_reflection
#    approach 1: add  --calib lab9/calib/camera_calibration.yaml  (workspace polygon from Task A)
# 3. stale detections over time
python3 solutions/lab9_real_hardware/evaluate_detection.py sequence lab9/frames/removed_cube
```
**Expect:** `lab9/eval_B/detection_table.md` with TP / FP / FN per frame and per folder for both detectors; `lab9/eval_B/<folder>/<frame>_compare.png` (top: detections, bottom: masks; original left, improved right); `sequence_removed_cube.md`: the original keeps reporting red after removal, the improved does not.
⚠ `evaluate_detection.py` uses the `COLORS` and `WORKSPACE_PX` **in `solutions/lab9_real_hardware/vision_B.py`**. Paste your tuned values there too (or copy your package `vision.py` over it), otherwise you evaluate the default values.
**Understand: the scoring** (`score()`): a detection counts as correct (TP) if a label of the same colour lies within half the detected side from its centre; otherwise FP. Labels without a detection are FN.
**Record in the report:** both tables, 2–3 compare images (one per lighting, the shadow/reflection case), every FP and FN with its reason, and why you chose each threshold and filter value.

---

## 8. Task C: HSV thresholds from an outlined object (approach 2)

**What the PDF asks (pp. 18–20):** a program that (1) shows an image and lets you click a polygon (left click adds, right click removes, Enter finishes); (2) masks the polygon, leaves out a band along its edge, extracts the HSV pixels; (3) reports median and standard deviation of S and V, thresholds = median ± 2σ clipped to 0–255; (4) handles hue as a circle, splitting an interval that crosses 0/179; (5) applies the thresholds to the whole image, shows original / polygon / mask / masked image, prints a `colors` entry for `vision.py`. Evaluate on ≥ 2 lightings and compare with the fixed thresholds.

### TC1–TC5. Run the program
**Do (any machine with a display, on recorded frames):**
```bash
python3 solutions/lab9_real_hardware/hsv_calibrate.py lab9/frames/light1_cubes/light1_cubes_000.png --name red
```
Click around the red cube's top face (5–8 points, slightly inside its edge), Enter. Repeat for yellow, green, blue.
**Expect:** in the terminal e.g. `2200 pixels   hue centre 178.4 spread 1.9   S median 205 std 9   V median 160 std 12`, the range(s), and a block to paste; a 2 × 2 window; files `lab9/hsv/red.yaml`, `lab9/hsv/red_panels.png`.
**If not:** `no pixels left inside the polygon` → polygon too small for the band (`--band 3`). Mask also lights up the table → the cube region was mixed with background (click further inside) or the colour is weak (Task B's thresholds will need a manual S/V lower limit).
**Record:** the four `*_panels.png`, the printed statistics and ranges.

**Understand: `hsv_calibrate.py`**
TC1, the polygon:
```python
def on_mouse(event, x, y, flags, param):            # OpenCV calls this for every mouse event in the window
    if event == cv2.EVENT_LBUTTONDOWN: points.append((x, y))
    elif event == cv2.EVENT_RBUTTONDOWN and points: points.pop()
view = img.copy()                                   # draw on a fresh copy each time, the original stays clean
cv2.polylines(view, [pts], False, (255, 255, 255), 1)   # False = open polyline while clicking
key = cv2.waitKey(20) & 0xFF                        # 13 = Enter, 27 = Esc
```
`on_mouse` is defined inside `select_polygon` and changes `points` from there (a *closure*): `append` and `pop` modify the same list object, so no `global` is needed.
TC2, the region:
```python
mask = np.zeros(shape[:2], np.uint8); cv2.fillPoly(mask, [polygon], 255)   # 255 inside the polygon
mask = cv2.erode(mask, np.ones((7, 7), np.uint8))   # shrink by ~3 px on every side: edge pixels mix cube and table
pixels = hsv[mask > 0]                              # boolean indexing -> (N, 3) array of the HSV values inside
```
TC3, S and V:
```python
s_lo, s_hi = np.clip([np.median(s) - 2 * s.std(), np.median(s) + 2 * s.std()], 0, 255)
```
The median is not pulled by a few glare or shadow pixels the way the mean is. σ is the **standard deviation** (same unit as the pixel values), not the variance (squared units, PDF remark).
TC4, hue as a circle:
```python
ang = np.deg2rad(h * 2.0)                                            # hue 0..179 -> angle 0..358 deg
centre = np.rad2deg(np.arctan2(np.sin(ang).mean(), np.cos(ang).mean())) / 2 % 180   # mean DIRECTION, back to hue
diff = (h - centre + 90) % 180 - 90                                  # signed distance to the centre, -90..90
```
Example: hues 178 and 2. The plain mean is 90 (cyan: wrong). As angles 356° and 4°, the mean direction is 0° → centre 0, distances −2 and +2 → spread 2. An interval `centre ± 2·spread` = −4..4 crosses 0, so it becomes two ranges, `0..4` and `176..179`, and `apply_thresholds` ORs their masks.
TC5: `cv2.bitwise_and(img, img, mask=mask)` keeps the colour only where the mask is white: you see at once what the thresholds accept.

### TC-eval. Evaluation (deliverable)
**Do:**
```bash
python3 solutions/lab9_real_hardware/evaluate_thresholds.py red lab9/frames/light1_cubes/*.png lab9/frames/light2_lamp/*.png
```
Outline the red cube once per image (saved in `lab9/eval_C/gt/`, reused next time).
**Expect:** `lab9/eval_C/red_table.md`: per image, for three threshold sets, "object kept %" (high is good) and "background accepted px" (low is good); `*_red_masks.png` with the three masks side by side.
- `calibrated` = your ± 2σ thresholds; `original` = the course `vision.py`; `min-width` = the same statistics with each half-width at least H 8, S 40, V 40 (`--min-half` changes this).
**What to look for (this answers the PDF's questions):** ± 2σ from one image is often **too narrow** for another lighting: V moves most when the light changes, so "object kept" drops in light 2. The `min-width` column shows whether a minimum width fixes that without accepting the table. Which pixels are wrongly excluded (the shadowed side, glare spots) or included (similar-coloured background) is visible in the mask images.
**Record:** the table for each colour, 2 mask images, your conclusion on the minimum width, and the final thresholds you put into `vision_B.py` (TB1).

---

## 9. Final evaluation against the Part 0 baseline [MOVES]

**Do:** install your final `vision.py` (TA-install / TB-install), restart vision, then:
1. Position accuracy (no motion, approach 1): `position_errors.py calibrated` (TA6). Approach 2: optional, the mapping is unchanged.
2. Detection (no motion): the TB-eval tables, plus a live check in both lightings.
3. Picks: brain (run guide B6) → menu `2`, the red cube at each of the **five P0 positions**, bin A, in the same order as in P2. Same table as P2.

**Record — the comparison table:**

| | Part 0 (original) | after Task 1 + Task 2 |
|---|---|---|
| position error mean / max (mm) | | |
| detections TP / FP / FN, light 1 | | |
| detections TP / FP / FN, light 2 | | |
| stale detection after removal | yes | no |
| successful picks (of 5) | | |

Be honest about what did not improve and why (e.g. approach 2 does not change positions).

---

## 10. Hand-in and git

You run git yourself. One commit per task (gameplan §7b). On the Lab PC:
```bash
cd ~/mycobot-project
git status
git add pp_moveit_ws/src/mycobot_vision/mycobot_vision/vision.py solutions/lab9_real_hardware/markers.yaml \
        lab9/calib lab9/tables lab9/eval_B lab9/eval_C lab9/hsv
git commit -m "Lab 9 Task A: calibrated pixel-to-robot mapping"     # one commit per task
git push origin myCobot-lab
```
Raw frames are about 0.5 MB each: commit the ones the report uses (or all of them if the size is fine for you).
To go back to the course `vision.py` at any time: `git checkout -- pp_moveit_ws/src/mycobot_vision/mycobot_vision/vision.py` (uncommitted change) or `git show <commit>^:pp_moveit_ws/src/mycobot_vision/mycobot_vision/vision.py > /tmp/vision_course.py`.

---

## 11. Troubleshooting (this guide's scripts)

| Symptom | Likely cause | What to do |
|---|---|---|
| `record_frames.py` window stays black / nothing happens | no images on `camera/image` | run guide B4; `ros2 topic hz /camera/image` |
| `ModuleNotFoundError: rclpy` or `mycobot_interfaces` | ROS not sourced in this terminal | `source ~/rosenv9.sh` (Lab PC) |
| `cannot import vision_B.py` in `evaluate_detection.py run` | as above, or run on the laptop | run steps 2 and 3 on the Lab PC; step 1 (label) works anywhere |
| `AttributeError: module 'cv2.aruco' ...` | OpenCV without the aruco module | `python3 -c "import cv2; print(cv2.__version__, hasattr(cv2, 'aruco'))"` should print `4.6.0 True` |
| `... is 1280x720, but K is for 640x480` | frames recorded at another size | record again at the vision size (run guide D2) |
| height check off by several cm | marker size or `camera_height` wrong, or plane height | measure the black square side again; `plane_z` |
| all cubes "outside the workspace" after TA-install | workspace limits in `markers.yaml` too small, or B axes swapped | look at the polygon in the window; recheck P0 axes and the annotated image arrows |
| a cube detected in light 1, lost in light 2 | V lower limit too high | TB1 / Task C `min-width`; `show_masks:=true` |
| a cube under a lamp is lost although the mask looks fine | glare hole lowers solidity | lower `SOLID_MIN` a little, or larger `KERNEL` |
| vision service answers but brain picks beside the cube | mapping or z chain | P1b / TA6 numbers; TA5 z note |
