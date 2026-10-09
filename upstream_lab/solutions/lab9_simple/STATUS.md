# Lab 9 simple: status and next fix (2026-10-02 17:25, end of lab session)

> **2026-10-05: no longer the main approach.** Lab 9 now runs in the course ROS pipeline:
> [../lab9_real_hardware/README.md](../lab9_real_hardware/README.md). This folder is the single-shot fallback; its values
> (markers, heights, bins, home) are mirrored in `lab9_real_hardware/lab9.yaml`. Sections below are the 10-02 state.

Read with [README.md](README.md) (run order) and [config.py](config.py) (all values). This file = what happened in the lab,
what is still wrong, and the planned fix (robot-taught calibration) to implement at home.

## 1. State of the files (checked 2026-10-05 11:15)

| Where | State |
|---|---|
| `origin/myCobot-lab` | `44701f0` (10-04 23:09, Labs 3–7 from home) on top of `ffc595e` (10-02 17:25, all lab changes below + this file) and `9013a6c` (first version of this folder) |
| `origin/myCobot-personal` | `32237ac`, 3 commits behind `myCobot-lab` (home work went to `myCobot-lab` directly) |
| Lab PC `CoRobot2` | `44701f0`, clean |
| Pi `cobot-pi1` | `9013a6c` (2 behind). `lab9_simple/` modified + untracked `STATUS.md` = byte-identical to `ffc595e` (scp copies). Not from this work: `PickAndPlaceTemplate.ipynb` modified, `Lab9_PickAndPlace_MoveIt_solution.ipynb` untracked (neither touched by the new commits) |

[Pi] pull (credential cache is empty after every reboot → GitHub token as password):
```bash
cd ~/mycobot-project
git checkout -- solutions/lab9_simple        # drop the scp copies (identical to ffc595e)
rm solutions/lab9_simple/STATUS.md           # untracked copy, would block the pull
git pull                                      # username + token
git log --oneline -1                          # 44701f0
git status --short                            # only PickAndPlaceTemplate.ipynb + Lab9_PickAndPlace_MoveIt_solution.ipynb
```

## 2. What works (seen in the lab, 10-02)

- Install: `~/venvs/mycobot/bin/pip install ultralytics==8.4.8 opencv-python==4.13.0.90`; vision runs in `~/venvs/mycobot`.
  The `QFontDatabase … fonts` lines at start are harmless Qt warnings.
- Camera 848×480 MJPG, both ArUco markers found (`DICT_6X6_50`, ids 1, 2), scale 0.95 mm/px.
- YOLO `best.pt` boxes the 35 mm red cubes (conf 0.58–0.86), keypoints on the top face.
- ROS link Lab PC → Pi (`/simple_pp/target`, domain 47): targets arrive, the Pi ran the Lab 8 sequence (`[pick 5] red x 168.6
  y 8.7 z_top 54.5 mm` → hover, down, lift, bin, drop, home).
- Pi python: `$PY` must be set by hand (`PY=~/venvs/mycobot/bin/python` or `/usr/bin/python3`, whichever imports
  `rclpy, pymycobot, RPi.GPIO`); `$PY robot_pi.py` without it → `robot_pi.py: command not found`.

## 3. Changes made in the lab (all in config.py / vision_pc.py)

| Time | Change | Why |
|---|---|---|
| 16:4x | `MARKERS_MM` 1: (140, −60), 2: (250, 60), `MARKERS_MEASURED = True` | hand-guided `read_tip.py` (servos released), rounded. Tip z 15 / 24 |
| 16:5x | `MARKERS_MM` y +20 → 1: (140, −40), 2: (250, 80) | first picks missed in y by "−20 mm". **Sign assumed** (robot stopped on the −y side); not confirmed |
| 16:5x | `BOX_MM` filter in `detect_cubes()` | YOLO boxed the empty **white plate as "red 0.86"** at x190 y−2 (inside the workspace) |
| 17:0x | `CUBE_MM` 40 → **35**; `SURFACE_Z_MM` 19.5 (13 for one run) | lab cubes are 35 × 35 mm; explains "nozzle 5–8 mm too high" |
| 17:1x | `BOX_MM = (30, 75)` | 35 mm cube incl. sides ≈ 40–60 mm in the box (58 measured); plate ≈ 150 |
| ~17:25 | by Mano: `MARKERS_MM` back to 1: (140, −60), 2: (250, 60) (y +20 undone); `SURFACE_Z_MM` 16 | after the 17:2x run |

Current key values (17:30): `MARKERS_MM {1: (140, −60), 2: (250, 60)}`, `SURFACE_Z_MM 16`, `CUBE_MM 35`, `CAMERA_HEIGHT_MM 470`
(estimate, not measured), `WS_X (75, 225)`, `WS_Y (−75, 75)`, `BIN_COORDS` = Lab 8 values (not re-measured).
Pick: flange z = 16 + 35 + 68 = 119, hover 189, drop 148. The `MARKERS_MM` / `SURFACE_Z_MM` comments in config.py still
describe the y +20 and 19.5 values (stale).

## 4. Incidents

- **`robot_pi.py --jog` collided the arm with itself** (start (93, −63, 30) next to the base, tool down; first `w` swung the
  arm through itself). Cause: `send_coords` near the base picks a contorted joint solution. **Do not use `--jog`;** remove
  it from `robot_pi.py` and the README (still in both).
- Markers were then measured by hand-guiding with released servos (`release_all_servos()` one-liner, `read_tip.py` in
  `~/` on the Pi, `power_on()` to lock again). Not reliable, see §5.

## 5. Open problem: the camera → robot mapping is wrong

Evidence (screenshots 16:5x and 17:2x):

1. The yellow box (= `WS_X × WS_Y`, centre **(150, 0)** mm from the J1 axis) is drawn from config, not detected. With the
   markers of the 17:2x run id 2 (250, 80) is outside it by definition (x > 225, y > 75); with (250, 60) still x > 225. The plate is ≈ 150 mm = the Lab 8 box
   size, so the plate was probably laid out as the workspace → markers should be near (90, −60) / (210, 60). The measured
   y (±60) fits that, the measured x (140 / 250) is ~50 mm too large (fits a tilted hand-guided reading: 68 mm nozzle).
2. The magenta circle (robot (0, 0)) should be on the centre of the round robot base; at 17:2x it is ~35–45 mm off,
   upper left (estimate from the screenshot).
3. **Something moved between runs:** `[calib]` rotation 91.3° (reference frame 15:39) → 92.7° (16:5x) → **96.0°** (17:2x);
   camera-over (193, −16) → (221, 48) (+20 of that from the y edit). The cube lying in the red bin mapped to y 165 (+20 =
   185 expected) then **213**. If the plate moved relative to the robot, any marker values measured before are stale.
4. Result of `[pick 5]` (x and y miss) was not recorded.

Quick check, no motion: ruler from the centre of the robot base to the centre of marker 1:
≈ 145 mm → measured values right (then move `WS_X/WS_Y` onto the plate); ≈ 110 mm → plate = Lab 8 workspace, marker x wrong.

## 6. Planned fix: teach the mapping with the robot (to implement at home)

Idea: the robot itself puts the nozzle at 2 known points; a cube is slid under the nozzle; the camera records the cube's
top-centre pixel. Two pixel ↔ robot pairs at **cube-top height** define the mapping directly → marker values, ruler,
camera height and the parallax correction drop out (all picks are at the same cube-top height).

```
robot_pi.py --teach                     vision_pc.py
  Lab 8 home                              live window, 1 cube visible
  hover P1 (150, −50), down to top+3 mm   key t → pixel of cube top centre ↔ P1
  Enter (cube slid under the nozzle)      
  lift, hover P2 (200, 50), down          key t → pixel ↔ P2
  Enter, lift, home                       2 pairs → PlaneMap → lab9/calib/teach_map.json, used at once
```

Points: inside the Lab 8 limits, 112 mm apart (1 mm pixel error ≈ 0.5° rotation); same height as `[pick 5]` (+3 mm so
the cube can slide under; 16 + 35 + 3 = 54 mm tip z with the current values). The cube must sit square under the nozzle centre (look from the side in x and y).

### Code changes (sketch)

`config.py`
```python
TEACH_MM = [(150.0, -50.0), (200.0, 50.0)]          # robot points for --teach (inside WS, Lab 8 height)
TEACH_FILE = "~/mycobot-project/lab9/calib/teach_map.json"
TEACH_CLEARANCE_MM = 3.0                            # nozzle this far above the cube top while teaching
```

`robot_pi.py` (replaces `jog`)
```python
def teach():
    print("WARNING: robot moves to the teach points now")
    rb = Robot()
    rb.home()
    z = C.SURFACE_Z_MM + C.CUBE_MM + C.TEACH_CLEARANCE_MM
    for x, y in C.TEACH_MM:
        rb.tip(x, y, z + C.HOVER_MARGIN)
        rb.tip(x, y, z, 30, 3.0)
        input(f"P({x:.0f}, {y:.0f}): slide a cube under the nozzle, press t in the vision window, then Enter ")
        rb.tip(x, y, z + C.HOVER_MARGIN)
    rb.home()
    rb.close()
```

`vision_pc.py`
```python
class PlaneMap:
    def __init__(self, px, mm, frame_wh, h_ref=0.0):   # h_ref: height of the plane the pairs were taken at
        ...                                            # unchanged; store self.h_ref = h_ref
    def top_to_xy(self, u, v, h):
        x, y = self.to_robot(u, v)
        if h == self.h_ref:
            return x, y                                # teach map: already at cube-top height
        H = C.CAMERA_HEIGHT_MM - self.h_ref            # camera height above the mapped plane
        k = (H - (h - self.h_ref)) / H
        cx, cy = self.cam_xy
        return cx + (x - cx) * k, cy + (y - cy) * k

# in main(): load TEACH_FILE if it exists -> PlaneMap(px, mm, wh, h_ref=C.CUBE_MM), else calibrate() from markers
# key t: one detected cube -> teach_px.append((u, v)) of keypoint 4; after len(C.TEACH_MM) pairs:
#        pm = PlaneMap(dict(enumerate(teach_px)), dict(enumerate(C.TEACH_MM)), wh, h_ref=C.CUBE_MM)
#        json.dump({"px": teach_px, "mm": C.TEACH_MM, "wh": wh, "h_ref": C.CUBE_MM}, TEACH_FILE); print(pm.report(...))
# arming: allowed if a teach map is loaded OR MARKERS_MEASURED
```
`PlaneMap.__init__` takes the 2 lowest keys of `mm` today → works unchanged with keys 0, 1.

README: new step 3 "teach" (replaces M1 + jog), warning "repeat after the camera or the plate moves"; markers stay as a
fallback and for the overlay check. Remove `--jog` and the M1 jog text.

### Test in the lab (order)

1. Ruler check (§5). Note whether camera / plate were moved since 17:2x.
2. [Pi, MOVES] `$PY robot_pi.py --teach` + `vision_pc.py`, key `t` at both points → `teach_map.json`.
3. Overlay: magenta circle on the base centre, the box where expected. `[calib]` scale ≈ 0.95 × (H−35)/H ≈ 0.88 mm/px
   (cube-top plane is closer to the camera).
4. One cube at (150, 0): `[pick n]` → note the miss in x and y (mm, direction). Then 2 more spots (near each teach point).
5. Bins: put a cube in each real bin, read its `x y` in the overlay → `BIN_COORDS` (the red-bin cube read y 213 vs 170 in
   config; yellow (200, 170) is over the table).

## 7. Other open items

- `CAMERA_HEIGHT_MM` (470) never measured; irrelevant once the teach map is used (only for the marker fallback).
- `BOX_MM` filter and `CUBE_MM 35` not yet tested in a full pick.
- The y +20 marker edit: sign never confirmed. With the teach map it no longer matters.
- Blue cylinder is not a YOLO class (ignored). Stacking not supported.
- README still says `$PY` without defining it; add `PY=...` lines.
