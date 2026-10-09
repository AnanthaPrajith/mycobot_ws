# Pick-and-place review — 5 October 2026

Review of the local repository at commit `7519a2f` and the ten supplied course PDFs. This is a code and saved-output review, with limited offline numerical checks. It is not a physical robot validation. No robot or camera was connected or commanded.

## Assignment and implementation order

`Pick_And_Place-2.pdf`, page 2, explicitly asks for two of five improvements: ArUco camera calibration, robust HSV masking, interactive HSV threshold calibration, shape detection, and object-interface extension. Sorting and stacking are baseline application functions described on pages 10–12. They do not by themselves replace the two selected improvements. The two pick-and-place PDFs describe different pipelines.

Recommended first stage: ArUco calibration and robust HSV masking; demonstrate them through two-bin sorting, followed by cube stacking. Then add shape detection and the color/shape/ID interface together. Interactive HSV calibration is a separate fifth improvement. Confirm any different task-count instruction with the instructor.

Two-bin requirement: one destination for red, another shared by the two selected non-red colors. The colors and physical bin assignments are still to be specified. Route colors to bin identifiers and keep measured destination poses separately, rather than giving every color an independent bin.

## What was found in the repository

- `config/workspace_calibration_points.json`: five yellow-cube centering samples, including four workspace positions and a center point. Stored affine-fit RMSE: 4.24 mm.
- `config/red_workspace_calibration_points.json`: five red-cube centering samples. Stored affine-fit RMSE: 2.51 mm.
- These files contain published camera coordinates and robot poses; they are not the newly taught white-board A–D records or verified marker geometry. The center samples are included in the recorded fits, so those RMSE values are training residuals, not independent validation errors.
- `scripts/teach_board_points.py` records A–D to `config/board_teaching_points.jsonl`. That output file is absent locally. The script's labels refer to an unavailable reference photo: A near marker 2, D near marker 1, B/C adjacent corners. Those labels alone cannot establish the actual board axes or marker offsets.
- `config/aruco_red_cube.json`, the capture tool's default input, is absent. No measured ArUco configuration is included in the repository.
- `src/mycobot_motion_v1/mycobot_motion_v1/motion_node.py` stores bin XY in mm: red `(140.6, -124.2)`, yellow `(218.3, -137.6)`, green `(115.8, 177.3)`, blue/cyan `(-6.9, 173.2)`. Only red/yellow have pickup offsets and release-height entries; current target filtering excludes the other colors.
- The assignment Brain uses different, hardcoded four-bin poses. It does not load these direct-controller bin measurements. A pose read from the SDK must be identified as flange or configured tool pose and converted before being used as a MoveIt `pump_head` target.

## The three kinds of corners

1. Board corners A–D define board location, axes, and workspace extent. Three noncollinear corrected points can define a planar board frame, with the fourth used to check geometry; a fit using all four needs separate validation. Hover-only measurements supply XY after tool correction, but do not supply board-surface Z unless clearance is known.
2. ArUco corners are the four outer corners of each printed marker, eight correspondences for two markers. Their pixel coordinates come from detection, in decoded top-left, top-right, bottom-right, bottom-left order. Their robot-base coordinates can be calculated from measured marker size, center position, plane height, and decoded orientation. They need not all be manually taught if the markers' placement relative to the measured board is known.
3. Validation positions are additional independently measured object locations, away from fitting points. They are used to measure actual localization error, not to fit the same calibration again.

Without the earlier conversation, it is not possible to identify exactly which additional four points another Codex session requested. There is no basis here for asking you to repeat A–D automatically. First recover the recorded file and determine which measurement was missing.

Use `p_base_tip = p_base_flange + R_base_flange @ t_flange_tip` with verified tool geometry and Euler convention. The saved affine files use an uppercase `XYZ` Euler sequence and a negative local-Z nozzle vector. Other code uses different RPY conventions; do not assume these describe the same physical tool. Hovering over a corner is not touching the board plane. The 68 mm nozzle assumption also differs from the scene URDF's 40 mm pump-head offset.

## Immediate calibration sequence

1. Recover A–D records from the robot PC, including orientation, tool definition, and hover clearance. Check point labeling and physical board dimensions. Keep the robot-base frame consistent with the SDK and MoveIt model.
2. Verify camera matrix and distortion coefficients at the actual capture resolution and focus. Obtain a multi-view checkerboard/ChArUco intrinsic calibration if the existing intrinsics cannot be verified. Two fixed markers in a single frame are used here for extrinsics, not as a complete intrinsic calibration dataset.
3. Measure both printed marker black-square side lengths, IDs/dictionary, centers relative to the board, decoded top-edge orientations, and plane Z in the robot base. Measure camera height above that plane and cube/cylinder heights. If sizes or planes differ, the current shared-size/shared-plane configuration needs adapting.
4. Build a measured calibration JSON accepted by `validate_config`: units `mm`, verification flag, intrinsics resolution, camera matrix, distortion, marker size, marker-plane Z, cube height, reprojection limit, two marker records (`id`, `center_xy_mm`, `top_edge_yaw_deg`), dictionary, and workspace XY bounds. Never mark geometry verified just to bypass the check.
5. Capture images with both markers unobscured. Detect all eight corners and solve PnP. Invert robot-to-camera to recover camera-to-base; compare estimated camera height with the independent measurement. Current code checks that the camera is above the plane but does not enforce agreement with measured height.
6. Reproject the corners, save an annotated image, report mean and maximum pixel error, and repeat on several frames. The current code computes errors but its ROS debug display only reports their maximum; it does not save the required reprojection overlay/report.
7. Validate object XY at new center/edge positions and report mean/max millimetre error. Use the object top plane, not the board plane. The ArUco vision currently assumes one fixed cube height; stacked tops need an explicit known height or another height-estimation method.
8. Validate predicted positions with supervised hover moves, then one pick and release. Fix Brain motion and controller issues before stacking or unattended sorting. Rebuild the ROS packages on this PC after source updates; old `build/` and `install/` contents are not evidence that the new source is running.

## Code findings that prevent calling the implementation fully valid

### Assignment vision and Brain

- `mycobot_vision/vision.py` uses the correct general ray/plane approach and clears stale detections. It requires a measured calibration file; the existing launch supplies none, so it currently fails at startup.
- It performs two red intervals, 3×3 opening/closing, area/fill/aspect filtering, and workspace rejection. Physical lighting tests and calibrated thresholds are still required. There is no upper size limit or physical dimension consistency check.
- A circular cylinder top can pass the square tests: bounding-box aspect is near one and circle fill ratio is about 0.785, above the 0.7 threshold. Shape classification is absent.
- The service handles one unambiguous object per color and rejects multiple candidates. It has no shape, explicit found flag, or unique object ID. Returned zero angles are placeholders for suction, not a measured full 6D object pose.
- Vision returns cube TOP Z. Brain places a 40 mm collision box CENTER at that Z; the center should be top minus half measured height.
- `Brain.pick()` hovers 100 mm above the top, descends only 50 mm, and switches on suction while still 50 mm above it. It ignores motion return codes and can proceed after failure.
- `Brain.place()` adds `0.04 * number` plus 60 mm, then descends 50 mm. With `number=2`, the commanded suction point is 90 mm above the lower top. For equal 40 mm cubes it should contact the carried cube top 40 mm above that lower top, subject to a verified contact allowance.
- Stacking re-queries a color after it has moved onto a stack. Vision projects onto a single fixed top plane, so it cannot localize raised tops correctly. Maintain a validated stack-base pose and accumulated height instead.
- Cartesian execution does not check path fraction/service error, empty trajectories, or goal acceptance. Its failure logger uses `.val` on an integer action error code, producing another exception. Reject incomplete paths and propagate failure before pump transitions.
- Pose goals use `g_base`, Cartesian requests use `world`. The current scene fixes these frames with identity, but consistency should be explicit before using a changed scene.
- Menus recurse. Sorting is manual destination selection among four bins, not automatic two-bin color routing. Stacking lacks the PDF's fallback from failed green placement to blue-on-red.
- `mycobot_controller` operates GPIO 20 only, unlike the two-pin pump/valve wiring in the notebook and direct motion code. Verify release-valve control. Its trajectory execution assumes velocity arrays, maps velocity to an unchecked SDK speed, and reports success without final pose verification.
- `vision2` is registered in setup.py but the module is absent.

### Notebook review

| Notebook | Assessment |
| --- | --- |
| 01 Spatial transformations | Composition and interpolation code is coherent on inspection; widget/backend dependencies prevent a full rerun here. |
| 02 Forward kinematics | Includes DH/URDF comparison and acknowledges that a home-pose tool correction aligns only that pose. Do not infer general model equivalence from the home assertion. It also contains an unconditional zero-joint publish cell. |
| 03 Inverse kinematics | Solver comparison and DLS structure are coherent. LM/BFGS are unconstrained; wrapping angles does not enforce URDF joint limits. Publishing does not validate solver success/limits. Shared `rot_to_vec` is invalid at exact 180-degree errors. DLS reports the final pre-update history residual, which may differ from the returned q's residual. |
| 04 Differential kinematics | Saved end-frame Jacobian comparison matches. Direct matrix solve can fail near singularities; hardware command assumes joint-state ordering, computes velocity once, and lacks a guaranteed stop in `finally`. It is not continuously recomputed resolved-rate feedback. |
| 05 TCP calibration | Transform composition and SVD mean are coherent. Saved RMS is 1.2363 mm on the calibration samples. The estimated millimetre-scale SDK/model correction does not establish a 68 mm physical nozzle calibration. Independently verify what the recorded SDK TCP denotes. |
| 06 Trajectory planning | Offline forward, reverse, short/triangular, and zero-motion profile checks reached the expected endpoints with zero final velocity. Validate positive limits/dt and use the exact final timestamp rather than a sample beyond tf for reuse. Publishing streamed joint targets does not prove the controller executes the calculated velocity/acceleration profile. |
| 07 Serial communication | Protocol construction agrees with the supplied lab. Direct serial and ROS motion cells run without explicit motion enable; failed/missing readback is not consistently guarded. Serial close and velocity stop need exception-safe cleanup, name ordering, freshness, and limits. |
| 08 Pick and place | Based on the older YOLO/direct-motion PDF, not the ArUco/Brain extension. Hardware and pump test default to True; motion waits are timed rather than verified. Bin coordinates differ from the updated motion node. No implemented stacking or shape interface; several student-answer cells remain placeholders. |

The YOLO node's `Z_ONE_CUBE=0.36` is described as robot-base Z but actually serves as reference camera depth; its transformation yields about 30 mm base Z with a 390 mm camera height. Clarify this convention. Replacing only depth while keeping raw PnP XY is an empirical correction, not a calibrated ray/plane reconstruction. Do not chain the old per-color affine corrections onto a new ArUco mapping without independent evidence that they remain applicable.

## Verification performed

- Parsed 91 Python source files: no syntax errors.
- Parsed code in all eight notebooks after stripping standalone IPython magic lines: no syntax errors. No saved exception outputs were present. Saved execution counts do not demonstrate a clean top-to-bottom run.
- Existing ArUco geometry tests: 3 passed. They cover ideal synthetic pose recovery, unverified geometry rejection, and a behind-camera intersection.
- Reproduced the rotation-error bug: exact pi rotations around X, Y, and a diagonal axis return a zero rotation vector in `helperFunctions.rot_to_vec`, although magnitude should be pi.
- Checked selected trajectory functions offline for forward, reverse, triangular, and zero-motion cases.
- Full notebook/ROS integration runs were not performed: the active Python lacks roboticstoolbox, spatialmath, rclpy, and IPython. Physical geometry, lighting, vacuum wiring, and robot motions remain unverified.

## Submission package

For the newer assignment use `src/mycobot_vision/mycobot_vision/vision.py` and `src/mycobot_brain/mycobot_brain/brain.py` after addressing the findings. Include `aruco_geometry.py`, because vision imports it; include measured calibration JSON, matching service definition, dependencies/build instructions, and validation evidence. The PDF's ArUco deliverables additionally require coordinate/unit conventions, an annotated detected/reprojected-marker image, and an independent validation-error table. HSV evaluation requires mask examples and results under at least two lighting conditions, including removed-object behavior. Video/results of sorting and stacking support the demonstrations.

Shape/interface extensions also require the modified `.srv`/message definitions and rebuilt matching server/client. Submitting only two standalone files would omit required runtime dependencies and calibration evidence.

Reference: OpenCV's official ArUco tutorial explains decoded corner detection and PnP use with intrinsics and distortion: https://docs.opencv.org/4.10.0/d5/dae/tutorial_aruco_detection.html
