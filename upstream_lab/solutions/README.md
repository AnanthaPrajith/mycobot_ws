# Worked solutions

One notebook per lab, next to the templates in the repo root. **Use them to compare, not to copy**: solve the template yourself
first, then read the matching solution. The interview asks you to explain your own code.

Every task in a solution notebook has the same parts:

1. **Task**: what the template asks, in a line or two.
2. **Solution**: code that runs. In Labs 1–8 these are the template's own cells with only the blanks filled in
   (same order, same names, nothing extra); the few lines that differ from the template are marked with a comment.
   Labs 3–7 (rewritten 2026-10-04) add a short **Concept** paragraph before the syntax notes.
3. **How it works**: each piece of syntax and each maths step, line by line.
4. **Example**: short code blocks with their output, written as text (not as cells), that show the syntax on a toy case.

The saved outputs come from running each notebook top to bottom (plots as static images). Run them yourself for interactive plots
(`%matplotlib widget`). The PDFs' short questions are answered at the end of the notebooks that have them (Labs 5–8).

| Notebook | Template / PDF | Runs on | Status |
|---|---|---|---|
| `Lab1_SpatialTransformations_solution.ipynb` | `SpatialTransformationsTemplate.ipynb`, tasks 1–33 | laptop | fully executed |
| `Lab2_ForwardKinematics_solution.ipynb` | `ForwardKinematicsTemplate.ipynb`, tasks 1–29 | laptop (+ ROS from task 18) | tasks 1–17 executed; ROS cells not executed |
| `Lab3_InverseKinematics_solution.ipynb` | `InverseKinematicsTemplate.ipynb`, tasks 1–18 | laptop; task 18 with the robot | tasks 1–17 executed; task 18 **real robot by default** (no saved output), tested in simulation |
| `Lab4_DifferentialKinematics_solution.ipynb` | `DifferentialKinematicsTemplate.ipynb`, tasks 1–20 | laptop; tasks 19–20 with the robot | tasks 1–18 executed; tasks 19–20 **real robot by default**, tested in simulation |
| `Lab5_TCPCalibration_solution.ipynb` | `TCPCalibrationTemplate.ipynb`, tasks 1–18 | laptop | fully executed (recorded robot data, no robot cells) |
| `Lab6_TrajectoryPlanning_solution.ipynb` | `TrajectoryPlanningTemplate.ipynb`, tasks 1–19 | laptop; task 19 with the robot | tasks 1–18 executed; task 19 **real robot by default**, tested in simulation |
| `Lab7_SerialCommunication_solution.ipynb` | `SerialCommunicationRobotControlTemplate.ipynb`, tasks 1–29 | Pi | tasks 1–5 executed; Part 1 (serial) and Part 2 (ROS) **real robot by default**, both tested in simulation |
| `Lab8_PickAndPlace_solution.ipynb` | `PickAndPlaceTemplate.ipynb`, tasks 1–28 | Pi (real) or laptop (simulation) | executed against a **simulated** `MyCobot` and GPIO (`sim_hardware.py`) |
| `lab9_real_hardware/` (**use this for Lab 9**) | `pdfs/Pick_And_Place-2.pdf`: Part 0, Tasks A, B, C | Lab PC + Pi, real camera, robot and pump | step-by-step `GUIDE.md` (approaches A + B or B + C) and the scripts it uses; tested on generated images only, not yet in the lab |
| `lab9_old_synthetic/Lab9_PickAndPlace_MoveIt_solution.ipynb` (reference) | `pdfs/Pick_And_Place-2.pdf` (no template) | laptop | algorithms executed on **synthetic** images and data (`lab9_synthetic.py`); "Where this goes" notes map each task to lines of `vision.py` / `brain.py` |

**Running Labs 8 and 9 on the real system** (which machine runs which node, files needed on the Pi and on the Lab PC, SSH steps and one-time setup for the lab, start order, the values you can change and experiments): [`Lab8_Lab9_run_guide.md`](Lab8_Lab9_run_guide.md). It is the single run guide (it replaced the former `lab9_runbook.md`) and uses plain terminals, no tmux (end of section 4).

Helper files in this folder:
- `labpaths.py`: puts the repo root on `sys.path` (so `helperFunctions` and `serial_iface` import as in the templates) and finds the URDF.
- `sim_hardware.py`: stand-ins for `pymycobot` and `RPi.GPIO` plus a simulated clock, so the Lab 8 notebook runs on a laptop (`sim_hardware.install()`; not called on the Pi).
- `sim_robot.py`: a simulated myCobot 280 behind a virtual serial port (`python3 solutions/sim_robot.py` prints e.g. `/dev/pts/5`). It answers the robot's byte protocol, so the real `serial_iface.py` (Lab 7 Part 1) and the real `mycobot_control` node (Labs 3, 4, 6, Lab 7 Part 2, started with `-p port:=/dev/pts/5`) run unchanged. Used only by the optional 🧪 simulation blocks; run it with `ROS_DOMAIN_ID=99` and localhost-only discovery so it can never command the real arm.
- `lab9_real_hardware/`: the Lab 9 approach for the real system. `GUIDE.md` explains every step; scripts: `record_frames.py` (save camera frames), Task A `camera_intrinsics.py`, `markers.yaml`, `aruco_extrinsics.py`, `position_errors.py`; Task B `evaluate_detection.py`; Task C `hsv_calibrate.py`, `evaluate_thresholds.py`; reference `vision_AB.py` (A + B) and `vision_B.py` (B). None of them sends robot commands.
- `lab9_old_synthetic/` (kept for reference, not needed in the lab): the old Lab 9 notebook and its helpers `lab9_synthetic.py` (synthetic test images), `publish_test_image.py` (publishes a synthetic or saved image as `camera/image`), `hsv_calibrator.py` (the earlier HSV calibrator, replaced by `lab9_real_hardware/hsv_calibrate.py`).

## Safety
- **Labs 3, 4, 6, 7: real robot by default** (rewritten 2026-10-04). Running a cell marked **[MOVES]** moves the arm, as in
  the templates. Run them cell by cell (never "Run All"), workspace clear, hand at the power switch. Each motion cell has a
  short guard instead of an on/off flag: it reads the measured pose and refuses big steps (IK target ≤ 15° per joint,
  trajectory start ≤ 5° from the arm, velocity ≤ 0.5 rad/s and not near a singularity, serial target relative to the
  measured pose and inside the joint limits).
- **Labs 2 and 8** keep their earlier switches: motion cells default to **off** (`send_motion`, `publish_ros2`,
  `enable_motion`, ...), and Lab 8 runs against a simulated robot unless `USE_REAL_HARDWARE = True`.
- Targets are relative to the **measured** pose and small. Never send all-zero angles.
- Found while testing in simulation (read in `mycobot_control.py`, [check] on the robot): `/mycobot/joint_states` is the
  controller's estimate and lags one position command behind; after a velocity command ends with zeros the controller has
  sent STOP, and later position commands are ignored until a new velocity command resumes the robot. The notebooks
  explain both and work around them.
- The Lab 7 template's example target `[90, 0, 0, 90, 0, 0]` at speed 100 is a move of up to ~170° per joint at full speed. The solution uses joint 1 + 10° at speed 20 instead.

## The URDF
The labs load `mycobot_280_gazebo.urdf` from the repo root (added 2026-10-02). It names its meshes
`package://mycobot_description/...`, which the toolbox can only resolve when it knows where that ROS package is:
`labpaths.urdf_path()` registers `ros2_ws/src` for that, so the notebooks load it without a sourced ROS workspace.
Labs 3–7 were executed with the course URDF; Lab 5 still reproduces the tool offset and base correction of the controller's
launch file to six digits (as it did with the earlier fallback `ros2_ws/.../mycobot_280_pi.urdf`, which `urdf_path()`
uses only if the course file is missing). ⚠ Your own templates in the repo root load the same file: if that fails with
`Package not found: mycobot_description`, start Jupyter from a terminal where `ros2_ws/install/setup.bash` is sourced.

## Where the solutions deliberately differ from the templates
| Lab | Template says / does | Solution does, and why |
|---|---|---|
| 1 | task 26: the given `update_phi` uses `[phi, theta, psi]`, the task text asks for the reversed order for RPY | `[psi, theta, phi]` in the three update functions |
| 1 | `np.arange(0, 1.05, 0.05)` for the SLERP times | `np.linspace(0, 1, 21)`: `arange` with float steps can end above 1.0 and `Slerp` rejects that |
| 2 | random joint vector / all-zero start for the ROS tasks | start from the **measured** pose with a small step (asserted ≤ 15° per joint); `publish_ros2` / `send_motion` default to `False` |
| 3 | `least_squares` result used directly | wrap with `transform_to_pipi`: the raw answer can contain 59 rad and violates the joint limits |
| 3 | task 18 publishes the toolbox IK solution | that solution has joint 1 near 168°, a half-turn from any normal pose: task 18 instead solves for a target 2 cm above the **measured** pose, seeded with the measured angles, and refuses steps > 15° |
| 4 | task 12/14 sweep joint 2 with all other joints 0 | that pose is already singular (q3 = 0): the curve is flat at 0. Extra sweeps from a healthy pose show where the singularities are (q3 ≈ 0, q5 ≈ ±90°) |
| 4 | resolved-rate control with the plain inverse | the path runs into the elbow singularity (joint 3 jumps 57° in one step); damped inverse shown |
| 4 | task 20 from wherever the arm is | refused near a singularity (the parked pose has joint 5 ≈ 90°, wrist-singular) or above 0.5 rad/s; stop in `finally` |
| 6 | `np.arange(0, tf + dt, dt)` samples | the last sample can be after `tf`, where the deceleration formula turns the velocity negative: the sampler holds the goal |
| 6 | task 19 publishes a trajectory that starts at `q_start` | refused unless the arm is within 5° of `q_start`; the text shows how to plan from the measured pose |
| 7 | target `[90, 0, 0, 90, 0, 0]`, speed 100 | joint 1 + 10° from the measured pose, speed 20, inside the joint limits |
| 7 | Part 2: subscription queue 10, one `spin_once` | queue depth 1 (always the newest state), retry until the first state arrives, re-publish after a move (fresh estimate), a wake-up velocity message before task 24 (resume after STOP) |
| 8 | `execute_pick_and_place` without stop checks; `KeyError` for an unknown colour in the middle of the sequence | the template cell is kept; an extra example `execute_with_stop_checks` shows stop checks before every move (as the node does) and refuses an unknown colour before anything moves |
| 8 | `enable_motion = False`, `enable_pump_test = False` | `= SIMULATED`: `True` in simulation, to be set by a person on the real robot |

## Regenerating / editing
The notebooks are ordinary `.ipynb` files: edit them in Jupyter. Cells that need the robot or ROS have no saved output; everything else has.
