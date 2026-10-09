# Lab 9 simple: camera + YOLO → pick and place (single shot, real hardware)

**Status, lab results and the open mapping fix: [STATUS.md](STATUS.md) (2026-10-02 17:25). Do not use `--jog` (collision).**

Written 2026-10-02. Independent of `pp_moveit_ws` / MoveIt and of the course `vision_node.py` (no course file changed, nothing to build).

```
Lab PC (camera)                                   Pi (robot)
vision_pc.py                                      robot_pi.py
 camera → YOLO pose (best.pt) → colour + top-centre pixel
 2 ArUco centres → pixel→robot mm (+ cube-height correction)
 stable 1 s + ARMED ──/simple_pp/target (PointStamped)──▶ workspace check → Lab 8 pick-and-place → home
                       frame_id = colour, point = top (m)    ignores targets while moving + 2 s
```

| File | Where | What |
|---|---|---|
| `config.py` | both | every value; **M1–M5 = what you measure** (step 3) |
| `vision_pc.py` | Lab PC | camera, YOLO, marker mapping, overlay window, sends targets |
| `robot_pi.py` | Pi | pick and place; `--jog` to measure M1–M3 |

Mapping: the 2 marker centres (M1, robot mm) fix scale, rotation and shift; the camera looks straight down (plate 157 × 158 px
in the 10-02 frame). A cube top 35 mm (`CUBE_MM`) above the plate is corrected towards the camera axis by `(H − 35) / H` (M4 = H).
Boxes whose longer side is outside `BOX_MM` (30–75 mm) are ignored (the white plate was once boxed as a red cube).

## ⚠ Warnings (read before step 3)

- **Both `robot_pi.py` modes move the arm immediately** (home first). Clear the area, keep a hand on the robot power switch.
  Ctrl+C stops after the current move and switches the pump off; for an emergency, switch the power off.
- **The marker positions in `config.py` are a guess.** `vision_pc.py` refuses to send (space = "locked") until M1 is
  measured and `MARKERS_MEASURED = True`. Never set it True without measuring: every pick goes where the mapping says.
- **Yellow bin (200, 170) is over the table** in the 10-02 camera view → measure M3 or remove yellow from `COLOR_ORDER`.
- **ARMED = every stable cube on the plate is picked.** First run: one cube only, `MOVE_SPEED = 30`, `MOTION_SLEEP = 2.5`.
- Jog near the plate with step 1 mm only (key `1`); z stops at −10 mm.
- The model knows red/green/yellow/cyan **cubes**; the blue cylinder is ignored. Single cubes only, no stacks.
- Untested: YOLO on this camera, the live loop, the robot side of this script. Lab 8 sequence and heights are unchanged.
- The Lab PC had 0.6 GB RAM free on 10-02: close Firefox and extra VS Code windows before step 4.

## 1. Once: install (Lab PC) and pull ([Pi])

```bash
# Lab PC: ~/venvs/mycobot already has torch 2.9.1+cpu, torchvision 0.24.1+cpu, rclpy works with its numpy 2.5.3
~/venvs/mycobot/bin/pip install ultralytics==8.4.8 opencv-python==4.13.0.90
```
The opencv pin: without it pip takes opencv-python 5.0.0 (untested with the ArUco call and ultralytics); 4.13.0.90 is the
course `requirements.txt` version. `best.pt` must be at `pp_yolo_ws/weights/best.pt` (from `Lab 09 PP.zip`, gitignored).

```bash
# [Pi]
ssh cobot
cd ~/mycobot-project && git pull
source /opt/ros/jazzy/setup.bash
python3 -c "import rclpy, pymycobot, RPi.GPIO; print('ok')"
~/venvs/mycobot/bin/python -c "import rclpy, pymycobot, RPi.GPIO; print('ok')"
```
Use the python that prints `ok` as `PY` (Lab 8 ran in the venv kernel). Folder not committed yet → from the Lab PC:
`scp -r ~/mycobot-project/solutions/lab9_simple cobot:~/mycobot-project/solutions/`

## 2. Every terminal (both machines)

```bash
source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=47; unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE
cd ~/mycobot-project/solutions/lab9_simple
```

## 3. Measure M1–M5 → `config.py` ([Pi] jog MOVES)

Jog keys: `w/s` x ±, `a/d` y ±, `r/f` z up/down, `1/2/3` step 1/5/20 mm, `p` print tip coordinates, `q` quit (goes home).
Copy the **actual tip** values that `p` prints.

| # | Do | Command ([Pi]) | Write into `config.py` |
|---|---|---|---|
| M1 | tip ~5 mm above the centre of marker 1, `p`; `q`; same for marker 2 | `$PY robot_pi.py --jog --start 93 -63 30` then `--start 205 55 30` | `MARKERS_MM = {1: (x, y), 2: (x, y)}`, `MARKERS_MEASURED = True` |
| M2 | over marker 1: key `1`, `f` until the tip just touches the plate, `p`, `r` | (same jog session as M1) | `SURFACE_Z_MM = z` |
| M3 | tip over the middle of each bin opening, `p` | `--start 125 170 120` (red/green), `--start -6.9 173.2 120` (cyan), yellow: find a real bin | `BIN_COORDS["red"] = (x, y)` … |
| M4 | tape: camera lens → plate surface | — | `CAMERA_HEIGHT_MM` |
| M5 | optional, ruler: edge of the black marker square | — | `MARKER_SIDE_MM` (adds a scale check) |

Edit on the Lab PC, then copy to the Pi: `scp config.py cobot:~/mycobot-project/solutions/lab9_simple/`

## 4. Run

Start order: Pi first (it goes home), then the Lab PC.
```bash
[Pi]     $PY robot_pi.py                               # home, then "waiting on /simple_pp/target"
[Lab PC] ~/venvs/mycobot/bin/python vision_pc.py      # window "lab9 simple", starts SAFE
```
Before pressing space, check the window (markers uncovered at start):
- `[calib]` printout: scale ≈ 0.95 mm/px, rotation ≈ 90° (10-02 setup); with M5 the marker side within ±10 %.
- magenta circle = robot base (0, 0) on the real base; red arrow = +x away from the robot; green arrow = +y.
- yellow rectangle = workspace on the plate; white circles = bins on the real bins.
- one cube on the plate: green box, label `x y yaw`.

Then **space = ARMED**: after 1 s still, the cube is sent; the Pi prints `[pick n] <colour> x y z_top` and picks.
Space again = SAFE (the current pick finishes). Stop: Ctrl+C on the Pi. `s` in the window saves raw + overlay to
`lab9/frames/` (for the report). Order with several cubes: red → yellow → green → cyan (`COLOR_ORDER`).

## If it fails

| Symptom | Cause → fix |
|---|---|
| space prints `locked` | `MARKERS_MEASURED = False` → step 3 M1 |
| `[calib] marker(s) [..] not visible` | marker covered or too small → clear it, `c`; or `FRAME_W, FRAME_H = 1280, 720` |
| base circle / rectangle not on the base / plate | M1 wrong (marker ids swapped?) → re-jog |
| no boxes on cubes | `YOLO_CONF = 0.3`; check the class names printed at start |
| `[send]` on the Lab PC, nothing on the Pi | `ROS_DOMAIN_ID` (both print a warning if not 47), firewall; `ros2 topic list` on the Pi must show `/simple_pp/target` |
| `[reject] … outside workspace` | cube off the plate, or M1 wrong |
| constant offset at every spot | M1 → re-jog |
| error grows away from the image centre | M4 `CAMERA_HEIGHT_MM` |
| nozzle stops above / presses into the cube | M2 `SURFACE_Z_MM` |
| cube not held / dropped | pump as in Lab 8 task 8; lower `MOVE_SPEED` |
| `cv2` / `aruco` error in the venv | opencv-python is not 4.13.0.90 → rerun the step 1 command |
