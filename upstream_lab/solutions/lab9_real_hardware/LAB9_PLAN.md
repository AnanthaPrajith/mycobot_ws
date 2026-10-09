# Lab 9 plan along the PDF (`pdfs/Pick_And_Place-2.pdf`, Sep 23, pages 14–26)

Written 2026-10-05 16:10 (decision §2 added 16:30) from the PDF text, the course code (`pp_moveit_ws/src/mycobot_*`) and today's robot runs.
Nothing in this file is implemented yet unless marked ✓.

## 1. What the PDF requires (pages 14–26)

Structure: **Part 0 is required**, then **2 of 5 tasks**; document and evaluate both chosen tasks (page 2).

| Part (PDF page) | Required work | Deliverable / evaluation (PDF wording, shortened) | Notes and additional information in the PDF |
|---|---|---|---|
| **Part 0 Explore the existing system** (14) | 1 prepare setup, check vision detects cubes before motion; 2 sorting mode 2 with ≥ 2 cubes, different containers; 3 stacking mode 1, note missing cube / failed motion; 4 trace the information flow; 5 document positions, selections, containers, outcomes, ≥ 2 limitations | **Short report + diagram or ordered list of the data flow camera image → completed pick-and-place.** Used as the baseline for the later evaluation | run the **provided** nodes; first motions supervised |
| Task: Recalibrating camera → robot (14–16) | K + d; 8 marker corners in frame B; `solvePnP`; invert; ray–plane intersection at the object's top height; review z and yaw; validate | updated `vision.py`, calibration parameters with frame + unit conventions, annotated image with detected + reprojected corners, table of validation errors (mean/max px and mm) | **Note: use the corners of both markers; two centres do not determine the full pose** |
| Task: Robust HSV detection (16–18) | thresholds (hue 0–179), morphology, candidate rejection (area, fill, workspace), no stale detections | original vs improved masks, ≥ 2 lighting conditions, shadows/reflections, a removed cube; false/missed detections; **keep the existing `/cube_coordinates` service** | new dict per frame |
| Task: Interactive HSV calibration (18–20) | polygon tool, mask minus border band, median ± 2σ for S, V; circular hue; split interval at 0/179; print thresholds for `vision.py` | calibrate on one image, test ≥ 2 lighting conditions, compare with the original thresholds, discuss minimum width | σ = standard deviation, not variance |
| **Task: Extension of the object interface** (20–22) | 1 service: request `shape`, response `found` + unique ID + pose, rule for several matches; 2 client `get_object(color, shape)`; 3 `spawn_object()` BOX/CYLINDER with calibrated dims, IDs; 4 ID through pick/attach/detach/remove, review `place()` (0.04 step); 5 menu: colour + shape, list matches, choose; keep `choose_drop()`; 6 loops instead of recursion, `exit` leaves the menu | (no separate list) competence: *compare the completed pick-and-place behaviour with the original implementation* | edit `GetCubeCoords.srv`, `colcon build --packages-select mycobot_interfaces`, restart both nodes; **cylinder: radius, not diameter**; stable unique IDs; one ID + one pose = document the selection rule; example menu ends with **"Destination bin: A / B / C / D"**; *"the existing motion-planning methods can largely remain unchanged"* |
| **Task: Detecting object colour and shape** (22–25) | 1 mask per colour + cleanup + size filter; 2 area, perimeter, centre, `minAreaRect`, circularity `4πA/P²`, `approxPolyDP` vertices; explain noise + occlusion; 3 classify cube/cylinder, thresholds **from images of the actual camera**, `unknown` if uncertain; 4 list of records (several per colour), draw contour + label, remove vanished objects; 5 query by colour + shape (red cylinder must not return a red cube), rule for none / several | annotated image, detected labels and centres; queries: existing, absent, several matches; **≥ 1 uncertain shape case discussed** | no yaw for a cylinder (not observable from above) |

Competences (page 25) also name: explain how camera, Vision, Brain, MoveIt and **Controller** exchange data.

## 2. Central decision: "course packages as is" vs. what the PDF asks

The PDF tasks are written as **edits of the course files** (`vision.py`, `brain.py`, `GetCubeCoords.srv` in
`mycobot_interfaces`). It never asks to change the controller, MoveIt, the camera node or the URDF.

| | A. PDF-literal | B. Course packages as is + our scripts as interface |
|---|---|---|
| Task code | edit `vision.py`, `brain.py`, `GetCubeCoords.srv` (as the PDF says) | new files that use the course nodes through their topics/services/actions |
| Controller, MoveIt, camera, URDF | **as is** | **as is** |
| Part 0 baseline | git tag on the original state (`04c715b`) | course packages are the baseline anyway |
| Deliverable "updated `vision.py`" | literally | new file instead; must be explained (ask TA) |
| Risk | none with the graders | graders may expect the named files |

**Final decision (2026-10-05 17:45, Mano): the package version `lab9_pick_place` (+ `lab9_interfaces`) is the final
implementation** (run guide `README.md`). Option B below (standalone scripts + course controller) is **not pursued**;
the rest of this plan (Part 0, task evaluations, §6 order from step 2 on, TA questions) still applies.

*Superseded:* **Decision (2026-10-05 16:30, Mano): B.** Course packages untouched and run as is; the task code lives in new files that
use the course nodes through their interfaces. (Still worth confirming with the TA, §7 question 1, because the PDF names
`vision.py` as the deliverable.)

### Implementation B (decided)

| Runs | What | Source |
|---|---|---|
| Pi | `ros2 run mycobot_controller controller` | course, as is |
| Lab PC | `ros2 launch mycobot_280_moveit2 move_group.launch.py` | course, as is |
| Lab PC | `ros2 run mycobot_280pi opencv_camera` | course, as is |
| Lab PC | `python3 vision_tasks.py`: shape detection task + query by colour and shape (`/get_object`) | **new file** |
| Lab PC | `python3 brain_tasks.py`: object-interface task, written fresh (not copied from `brain.py`), client of the course MoveIt + controller | **new file** |
| Pi, only if the valve test fails | `python3 valve_pi.py` (GPIO 21) | new file |

New files in `solutions/lab9_tasks/` (outside the course packages):
- `vision_tasks.py`: marker map + HSV per colour + shape features/classification + fresh object list per frame + `/get_object`
  (reuses our own detection code from `vision_lab9`, no course code).
- `brain_tasks.py`: `get_object(color, shape)`, `spawn_object()` (BOX / CYLINDER with radius), ids through
  pick/attach/detach/remove; menu loops with `exit`; bins **A / B / C / D** (PDF example); motion = MoveIt plan only →
  thin the trajectory → course `follow_joint_trajectory` → wait for arrival (so the course controller works unchanged).
- `lab9_tasks.yaml`: all values (markers, heights, HSV, shape thresholds, bins A–D, home posture).
- `Lab9_Evaluation.ipynb`: evaluation cells (queries existing / absent / several, annotated images, tables).
- Service type: `lab9_interfaces/GetObject.srv` (new package, not a course package; a custom service must be built).

Differences to the fallback `pp_moveit_ws/src/lab9_pick_place` (stays as is, run guide `README.md`): no controller copy,
no MoveIt launch override, brain not copied from `brain.py`, bins A–D, `exit`.

## 3. How different we have been until now

| Area | PDF / course | What we did (10-02 … 10-05) | Verdict |
|---|---|---|---|
| Part 0 baseline | required first | **not done** (we went straight to new code) | do first |
| Robot motion route | course brain → MoveIt → course controller | `lab9_simple`: pymycobot + YOLO on a topic (Lab 8 style, no MoveIt) | outside the PDF; keep only as fallback |
| Controller | as delivered | `controller_lab9`: copy + valve pin 21 + waypoint thinning, speed 40–90, arrival wait | **not asked by the PDF → go back to the course controller** |
| MoveIt | as delivered | `lab9_moveit.launch.py`: execution limits × 4 + 5 s | not asked → avoid (see §4) |
| Calibration | Task: K + `solvePnP` with 8 corners + ray–plane | 2 marker **centres** → similarity + height correction | works (≈ 5 px), but the PDF note says this is not a full calibration → **not** our graded task |
| Robust HSV | Task: morphology, rejection, stale | done inside `vision_lab9` (open/close, size, workspace, new list per frame) | partly covered; not chosen |
| Interactive HSV tool | Task | old `old_tasks_ABC/hsv_calibrate.py` (not run) | not chosen |
| Object interface | Task: edit `GetCubeCoords.srv` + `brain.py` | new `GetObject.srv` (own package), `brain_lab9` = edited **copy** of `brain.py`, bins by colour, joint-space home, yaw retries, plan-only fixes | content matches items 1–6; form differs (copy, new srv, colour bins instead of A–D) |
| Shape detection | Task | `vision_lab9`: C, `approxPolyDP`, fill, `unknown`, several per colour, labels, stale removal | content matches items 1–5; thresholds need real-camera justification + evaluation |
| Object size | 0.04 m in the code | 35 mm measured; cylinder height assumed | keep 35 mm (PDF: "calibrated dimensions") |

## 4. The course controller as is: consequences and how to live with it

What it does (read from `controller.py`): publishes `/joint_states` (20 Hz, paused while executing); executes a
`FollowJointTrajectory` by sending **every** waypoint with `send_radians(positions, 100 × max|velocity|)` and sleeping;
cannot be cancelled; `/pump_controller` "on"/"off" switches **GPIO 20 only**.

| Effect seen on 10-05 | Cause | Without touching the controller |
|---|---|---|
| very slow, stop-and-go | many waypoints, low speed values | in **our brain**: plan only, thin the trajectory (one point per ≥ 0.25 s) and set the `velocities` field so `100 × max|v|` is 40–90, then send it to the course `follow_joint_trajectory` action — the course brain already sends Cartesian paths this way (`send_cartesian_path()`) |
| `TIMED_OUT` (−6), arm keeps moving | MoveIt executes and times out; controller ignores cancel | follows from the line above: MoveIt only **plans** (`plan_only`), so its execution timeout never applies; course `move_group.launch.py` as is |
| next plan starts before the arm arrived | controller returns after the last sleep | brain waits on `/joint_states` until the arm is within ~1.7° of the last point |
| object may stay on the pump (Lab 8 needed GPIO 21) | valve pin not switched | **test first** with the course controller (it may release well enough); if not: tiny extra script on the Pi that listens to `/pump_controller` and drives GPIO 21 only |
| start state in collision (−10) | arm parked inside the URDF's `env_camera` box | park the arm (Lab 8 intermediate pose) before starting; home as a joint posture |

For **Part 0** these effects are exactly the "limitations or unexpected behaviours" the report asks for: record them with
the original brain, before any fix.

## 5. Implementation plan (chosen tasks: shape detection + object interface)

Why these two: their content is already implemented and simulation-tested; they fit together (both extend the same
service); calibration via markers is still needed to make the robot reach the objects but is not graded.

### 5.1 Part 0 baseline (lab session 1, ~1.5 h)
1. Original system only: [Pi] `ros2 run mycobot_controller controller`; Lab PC `ros2 launch mycobot_brain brain.launch.py`.
2. Check the `Color Detection` window detects cubes (expected: mostly not — fixed 50–150 px filter, blue S/V ≥ 200, old pixel map).
3. Mode 2 with ≥ 2 cubes and two containers; mode 1 stacking; note missing cube / failed motion behaviour.
4. Record: initial positions (ruler), selections, bins, outcomes; screenshots; brain + MoveIt + controller logs.
5. Data-flow diagram: `opencv_camera` →`camera/image`→ `vision` →`/cube_coordinates`→ `brain` →`move_action`/`compute_cartesian_path`→
   `move_group` →`follow_joint_trajectory`→ `controller` (serial) ; `brain` →`/pump_controller`→ `controller` (GPIO 20);
   `controller` →`/joint_states`→ `move_group`; `brain` →`/collision_object`,`/attached_collision_object`→ `move_group`.
6. Limitations to report (expected from 10-05): detection with the new camera, positions off (old mapping), 4 cm boxes,
   slow stop-and-go execution, `TIMED_OUT` while moving, recursive menu, one object per colour.

### 5.2 Task: Detecting object colour and shape (PDF 22–25)
| PDF item | Implementation (exists in `vision_lab9.py`, reused) | Still to do |
|---|---|---|
| 1 masks + cleanup + size | per-colour `inRange` (red two intervals), open 3×3 + close 5×5, enclosing-circle size 30–75 mm, search area = workspace + 20 mm | justify kernel sizes in the report |
| 2 features | area, perimeter, centre (moments), `minAreaRect`, `C = 4πA/P²`, `approxPolyDP(0.04 P)`, fill = A/enclosing-circle | explain square vs circle, noise, occlusion |
| 3 classify | fill ≥ 0.75 & C ≥ 0.75 → cylinder; fill ≤ 0.72 & C ≤ 0.85 & 4–6 vertices → cube; else `unknown` | **thresholds from real-camera images**: record C/fill/v for every object type (log prints them) and set limits between the clusters |
| 4 records + display + removal | list of dicts per frame (id, colour, shape, centre, yaw), contour + label drawn, list replaced every frame, 1 s staleness | — |
| 5 query | service request colour + shape; `found`; nearest-first `matches` | define + document the selection rule (nearest to the base) |
| Evaluation | — | annotated images (key `s`): cubes + cylinders, several colours, **two of the same colour**; labels + centres vs ruler; queries existing / absent / several; **one uncertain case** (touching objects or partial occlusion) |

### 5.3 Task: Extension of the object interface (PDF 20–22)
| PDF item | Implementation | Still to do |
|---|---|---|
| 1 service | request `color`, `shape` (+ optional `id`); response `found`, `id`, `shape`, `coords` (x, y, z_top in m, frame `g_base`, + roll, pitch, yaw), `height`, `matches` | decide A/B (§2): extend `GetCubeCoords.srv` (PDF) or keep `GetObject.srv` |
| 2 client | `get_object(color, shape, id)` checks `found` | — |
| 3 scene | `spawn_object()` BOX 0.035³ / CYLINDER [h, **r = 0.0175**], placed by the top surface, id = object id | — |
| 4 identity | same id for add, attach (pump_head), detach, remove; `place()` step = 0.035 | — |
| 5 menu | colour + shape → list "1 - red cylinder at (0.14, 0.03)" → select | **bins A / B / C / D** as in the PDF example (map A–D to our measured bins; colour default optional) |
| 6 loops | `while` menus, `q`/`exit` returns | use `exit` as the PDF word |
| motion | course `send_goal_pose`/`send_cartesian_path` logic kept (PDF: "can largely remain unchanged"), plus plan-only + thinning + arrival wait (§4) so the course controller works | document as a necessary deviation |
| Evaluation | — | same scenes as Part 0 → compare success, positions, time with the baseline |

### 5.4 Calibration used (not graded)
2 marker centres (robot-measured) → pixel → `g_base` map + height correction (`PlaneMap`), values in `lab9.yaml`.
State in the report that it is a planar approximation (camera looks straight down, checked ≈ 5 px), not Task A.

## 6. Order of work

| # | What | Where | Status |
|---|---|---|---|
| 1 | Decide the implementation (§2) | — | ✓ final: `lab9_pick_place` package version (10-05 17:45); TA confirmation of new files vs edited `vision.py` open |
| 2 | Part 0 with the original system, report draft | lab | open |
| 3 | Course controller on the Pi for everything except the fallback (`lab9_pick_place` stays as backup) | Pi | ✓ (Pi on `5cda33d`, course files original) |
| 4 | ~~`solutions/lab9_tasks/` (option B)~~ not pursued; open in `lab9_pick_place` instead: bins A–D + `exit` in the brain menu (PDF example), re-measure the marker centres after the +45 mm plate move | `lab9_pick_place` | open |
| 5 | Valve test with the course controller; extra Pi script only if needed | lab | open |
| 6 | Shape thresholds from real images; evaluation scenes | lab | open |
| 7 | Object-interface evaluation + comparison with Part 0 | lab | open |
| 8 | Report: data flow, both tasks, tables, images | — | open |
| ✓ | Detection, service, scene objects, ids, menu loops implemented; simulation test passed | `lab9_pick_place` | done 10-05 |

## 7. Questions for the TA
1. May the task code live in new files (course packages untouched), or must `vision.py`, `brain.py` and
   `GetCubeCoords.srv` be edited as the PDF describes?
2. Is the 2-marker-centre calibration acceptable as a working calibration if Task A is not chosen?
3. The course controller streams every waypoint (slow, `TIMED_OUT`): is adapting the trajectory in the brain the intended
   way, or is a controller change allowed?
4. Bins: keep A–D (PDF example) or colour-named bins?
