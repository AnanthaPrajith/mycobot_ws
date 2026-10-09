# Lab 8 and Lab 9: running the system, what runs where, why, and the values to experiment with

This is the single guide for the real system (it replaces the former `lab9_runbook.md`). The command steps and the real lab values (IP addresses, domain id, firewall, camera index, first pick-and-place check) come from the runbook written in the lab on 2026-10-01; the reasons, value tables, experiments and code problems were added around them.

How to read the marks in this guide:
- **[code]**: read in the source files of this repo (file and function named).
- **[check]**: not verified. Either it could not be verified from the home laptop (no access to the Pi), or it is a reading of mine. The command that answers it is given. Do not trust it until you ran the command.
- **[done]** / **[TODO]**: status copied from the runbook notes of 2026-10-01. I have not verified these myself.
- **[MOVES]**: this command or action moves the arm. A person runs it, with the workspace clear, the stop within reach, and the first move small and slow.

## The machines and the fixed values

| | Lab PC | Robot Pi | Home laptop |
|---|---|---|---|
| Name / address | `CoRobot2`, 129.217.130.101 | `cobot-pi1`, 129.217.130.85 (`ssh cobot`) | your own machine, `~/TUD/cobots` |
| Runs (Lab 9) | camera, vision, MoveIt `move_group`, brain, RViz | **only** the controller (robot serial + pump GPIO) | notebooks, simulation, `lab9_old_synthetic/publish_test_image.py`; no robot |
| Repo path | `~/mycobot-project` | `~/mycobot-project` | `~/TUD/cobots` |
| Branch | `myCobot-lab` | `myCobot-lab` | `myCobot-personal` (merge into `myCobot-lab` when it works) |

- **ROS domain 47** on both lab machines (change it in both `~/rosenv9.sh` files if the TAs give another number).
- Paste tip: if a pasted line starts with `^[[200~`, delete it and paste again with **Ctrl+Shift+V**.

**Status as noted on 2026-10-01:** A1–A6 of section 4 **[done]**, **A7 (network test) [TODO]**. Lab 7 Part 2 and Lab 8 were done on the robot (gameplan §12).

## Lab day in order

1. Lab PC: `ssh cobot 'hostname'` prints `cobot-pi1` without a password (A0).
2. Both machines: `git pull` on `myCobot-lab`; rebuild only if sources changed (A2, A4).
3. **A7 network test** if it was not done yet. If it fails, stop and ask the TAs (multicast blocked).
4. **Simulation first** on the Lab PC, controller **off** (6a).
5. Real robot, in this order (6b): controller on the Pi → `joint_states` check → `move_group` → camera → `vision` → service call check → **accuracy check with a ruler (6d, D3), before brain** → `brain` **last** (it moves the arm to home as soon as it starts).
6. First cycle: one cube near the centre, bin A, hand at the power switch; remember the all-zero "rejoice" move after the run (section 0, item 2).
7. Stop in the order of 6c; push your work from the Pi; `ssh cobot 'git credential-cache exit'`.
8. Ask the TAs when needed: where `mycobot_280_gazebo.urdf` is (gameplan §11 #3), whether you may remove `rejoice()` in your copy of `brain.py`, and (only if A7 fails) multicast.
Lab 8 notebook only: 5a (needs just the Pi and Jupyter).

Contents: 0 Safety and surprises · 1 The two systems · 2 Which machine runs what · 3 Files each machine needs · 4 One-time setup (A0–A7) and terminals without tmux ·
5 Lab 8 runbook · 6 Lab 9 runbook (simulation, real robot, stop, first pick and place) · 7 Why it is built this way · 8 Values you can change · 9 Experiments, safest first · 10 Troubleshooting · 11 Problems found in the code

---

## 0. Safety, and eight things the code does that you would not expect

Rules that apply to everything below: a person runs every command that moves the arm; workspace clear; know where the power switch is *before* the first run; first run in simulation where possible; small and slow first; only **one** program may hold `/dev/serial0` (`fuser -v /dev/serial0` shows who).

| # | What happens | Where [code] | What to do |
|---|---|---|---|
| 1 | **Starting `brain` moves the arm** to the home pose `[0.16, -0.06, 0.32, 0, 1.57, 0]` before the menu even appears. | `brain.py: control_menu()` calls `go_home()` first | Start `brain` last, workspace clear, hand at the power switch. |
| 2 | **After every sort or stack run the controller sends all-zero joint angles** (arm straight up) at speed 40, then `[0,-60,15,-45,0,0]`, then zeros again. `brain.rejoice()` publishes `True` on `/rejoice`; `controller.rejoice` runs three `send_angles` with fixed sleeps. | `brain.py: rejoice()`, `controller.py: rejoice()` | Clear the space above the arm. If you do not want it: remove the two `self.rejoice()` calls in *your copy* of `brain.py` (it is a course file: ask the TAs first). Never publish `True` on `/rejoice` yourself. |
| 3 | **Starting the Lab 8 `motion_node` moves the arm** to the intermediate pose, then home, at once. | `motion_node.py: __init__` ends with `self.go_home()` | Same as 1. |
| 4 | The Lab 8 stop key `s` is **disabled by default** (`DEBUG_STOP_ENABLE = False`). Ctrl+C in `brain` only ends the Python process; a trajectory already sent to the controller still runs to its end (the controller has no cancel handler). | `motion_node.py`, `controller.py: execute_trajectory` | Do not rely on software stops. **The power switch is the stop.** |
| 5 | Lab 8 drives **two** GPIO pins for the pump (20 = vacuum motor, 21 = release valve); the Lab 9 controller drives **only pin 20**. Pin 21 is never set up there, so the valve is whatever the hardware defaults to. | `motion_node.py: pump_on/pump_off`, `controller.py: control_pump` | **[check]** If suction is weak or the cube does not drop in Lab 9, compare with Lab 8's `pump_on` (both pins LOW) before blaming the vision. |
| 6 | The Lab 9 pump is only switched by a topic: `ros2 topic pub /pump_controller std_msgs/msg/String "{data: 'on'}"` runs the motor. | `controller.py: control_pump` | Test the pump with an object at the nozzle, motor on for seconds, then `off`. |
| 7 | The controller sends `speed = int(100 * max|velocity|)` for every trajectory point. MoveIt's joint velocity limit is 2.0 rad/s and `brain` asks for scaling 1.0, so the value can reach 200, but the robot API takes 0–100. | `controller.py`, `joint_limits.yaml`, `brain.py: send_goal_pose` | **[check]** What the library does with values above 100 and with 0 (short segments). Watch the controller log in the simulation first; experiment 9.6. |
| 8 | `brain.launch.py` and `brain_simulation.launch.py` open **gnome-terminal tabs**. They need a desktop session; over plain SSH or tmux they fail. They also do **not** start the controller. | `brain.launch.py` | On the Pi: start the nodes by hand (section 6). |

---

## 1. The two systems

### Lab 8: one process per job, direct robot library, no MoveIt
```
 USB camera ─► vision_node (YOLO pose + solvePnP) ──/detected_objects──► motion_node ──► pymycobot ──► arm
              (pp_yolo_ws/src/vision)           mycobot_msgs/DetectedObject   (pp_yolo_ws/src/mycobot_motion_v1)   └► GPIO 20/21 ─► pump
```
- `DetectedObject`: `float32 x, y, z` (**metres**, robot base frame), `string color` (+ joint fields `j1..j6`, unused by the vision node). **[code]**
- `motion_node` filters by workspace, waits 2 s collecting detections, picks the highest stack (then colour priority), and runs the 8-step sequence of the Lab 8 notebook, in **millimetres and degrees**. **[code]**
- The Lab 8 notebook (`PickAndPlaceTemplate.ipynb`) rebuilds the pieces of this by hand; it does **not** need ROS.

### Lab 9: five nodes, MoveIt plans, the controller executes
```
 USB camera                                                       ┌──────────── MoveIt move_group ─────────────┐
    │ camera/image (10 Hz)                                         │ /move_action (pose goals, plans + executes)│
    ▼                                                              │ /compute_cartesian_path (straight lines)   │
 opencv_camera ─► Vision ──/cube_coordinates (service)──► Brain ───┤ /collision_object, /attached_collision_object (planning scene)
 (mycobot_280pi)  (HSV, pixel→robot)   GetCubeCoords      (menu,   └───────────────┬────────────────────────────┘
                                                           sequences)              │ /arm_group_controller/follow_joint_trajectory (action)
                                                              │ /pump_controller   ▼
                                                              └──(String "on"/"off")──► Controller (Pi) ─► pymycobot ─► arm,   GPIO 20 ─► pump
                                            Controller ──/joint_states (20 Hz)──► MoveIt (current state)
```
- Free-space moves: `Brain → /move_action → MoveIt plans and executes → controller`. Straight-line moves (down onto the cube, back up): `Brain → /compute_cartesian_path → Brain sends the returned trajectory directly to the controller` (MoveIt has no action to execute Cartesian paths). **[code]**
- The RViz simulation (`demo.launch.py`) **replaces** the controller (it serves the same action). Never run both. **[code, MoveIt README]**
- Units: Vision/Brain/MoveIt in **metres and radians**, frame `g_base`. The controller converts to what pymycobot needs.

---

## 2. Which machine runs what

| Machine | Runs | Why |
|---|---|---|
| **Robot Pi** `cobot-pi1` | `controller` (Lab 9); the Lab 8 notebook and `motion_node` | Only the Pi has `/dev/serial0` (robot) and the GPIO header (pump). The controller README says it must run on the Pi. **[code]** |
| **Lab PC** `CoRobot2` (or the home laptop for the simulation) | MoveIt `move_group`, `brain` (interactive menu), `vision` (opens an OpenCV window), RViz, the camera node | The Pi has 1.8 GB RAM and no swap (gameplan §2); vision and RViz need a display; the menu needs a terminal. |
| **Lab 8 YOLO vision** | Lab PC or laptop | needs `torch` and `ultralytics` (hundreds of MB), which do not fit the Pi's RAM budget. `pp_yolo_ws/Vision_requirements/requirements.txt` pins torch 2.9.1. |

**Where the camera is plugged in.** It is on the **Lab PC**: the Logitech Webcam C930e is `/dev/video0`, so `opencv_camera` runs there with `num:=0` **[done, A3]**. That is the design the course code assumes, and it keeps the large image stream off the network.
- If the camera were on the **Pi** instead: run `opencv_camera` on the Pi (it needs `mycobot_280pi` and `cv_bridge` built there) and send `camera/image` over the network: 640×480×3 bytes × 10 Hz ≈ 9 MB/s ≈ 74 Mbit/s. Use a wired connection; with Wi-Fi expect dropped frames. Alternative: run `vision` on the Pi and look at its window through `ssh -X` (heavy for the Pi).
- Whatever you choose, `vision.py` assumes a **640×480** image (its pixel constants and the Lab 8 intrinsics are for that size). Check in D2.

**ROS networking between the machines.** Nodes on different machines find each other with DDS multicast, but only with the **same `ROS_DOMAIN_ID`** on every machine (here **47**) and a network that passes multicast. Ubuntu's firewall (`ufw`) must also let the other machine's UDP through (A5). Test in A7.

---

## 3. Files each machine needs

Repo: same repository everywhere, branch `myCobot-lab` on the lab machines.

### Robot Pi
| Needed for | Files / packages | Notes |
|---|---|---|
| Lab 8 notebook | `PickAndPlaceTemplate.ipynb` (work), `solutions/Lab8_PickAndPlace_solution.ipynb` (compare), the venv `~/venvs/mycobot` with JupyterLab | `pymycobot` and `RPi.GPIO` importable (the Pi has `python3-rpi-lgpio` for `RPi.GPIO`, gameplan §12); no ROS needed |
| Lab 8 `motion_node` | `pp_yolo_ws/src/mycobot_msgs`, `pp_yolo_ws/src/mycobot_motion_v1` (built); or the older `ros2_ws/src/mycobot_control/pi_motion3_node.py` (run as a script, imports `mycobot_msgs`) | ask the TAs which one is current; `motion_node.py` imports `termios`, so it needs a terminal |
| Lab 9 controller | `pp_moveit_ws/src/mycobot_controller` (built with colcon, A4) | needs `control_msgs` (installed, gameplan §12), `pymycobot` (module `pymycobot.mycobot280`), `RPi.GPIO` |
| Network | same `ROS_DOMAIN_ID` as the Lab PC, `ufw` rule for the Lab PC | A5, A6 |

**Not on the Pi:** MoveIt, RViz, torch/ultralytics, or VS Code extensions installed "in SSH" (gameplan §9 rule 7).

### Lab PC (and the home laptop for the simulation)
| Needed for | Files / packages | Notes |
|---|---|---|
| Lab 9 nodes | `pp_moveit_ws/src/{mycobot_interfaces, mycobot_description, mycobot_280_moveit2, mycobot_280pi, mycobot_vision, mycobot_brain}` built with colcon (A2) | the controller is **not** built here unless you test the real-hardware imports |
| ROS and MoveIt packages | The Lab PC got them in A1. The **home laptop has none** of the MoveIt packages (checked 2026-09-29: 0 `moveit` packages, no `control_msgs`, `moveit_msgs`, `controller_manager`, `joint_state_publisher_gui`, `xacro`) | same install block as A1, run with `sudo` yourself |
| Camera | the USB camera + `v4l-utils` **or** `solutions/lab9_old_synthetic/publish_test_image.py` (no camera needed) | |
| Lab 8 YOLO vision | `pp_yolo_ws/src/{vision, mycobot_msgs}`, `pp_yolo_ws/weights/best.pt` (42 MB, never commit), a venv `yolovenv` from `Vision_requirements/requirements.txt` (numpy 2.x, torch) | three hardcoded paths to edit in your copy: weights `/home/tejas/YOLO/runs/pose/train/weights/best.pt`, `CALIB_FILE /home/tejas/z_scale_calibration.json`, camera `/dev/video2` |
| Your tools | `solutions/lab9_real_hardware/` (Lab 9 guide and scripts); for reference `solutions/lab9_old_synthetic/` (old notebook, `publish_test_image.py`, `lab9_synthetic.py`, `hsv_calibrator.py`) | |

One more constraint **[code]**: `mycobot_description` exists in **both** `ros2_ws` and `pp_moveit_ws` (277 MB each, same package name). In one terminal source **only one** workspace overlay; for Labs 8 and 9 that is `pp_moveit_ws`/`pp_yolo_ws`, not `ros2_ws`.

---

## 4. One-time setup (A0–A7)

Steps A1–A6 are **[done]** on the lab machines per the 2026-10-01 notes; they stay here for reinstalls and for the home laptop. A0 and the probe only **read** (no motion): safe at any time.

### A0. SSH key and alias (gameplan §6c) [done]
Why: one password-less login means every command in this guide can start with `ssh cobot`, and the connection-reuse lines make a second shell on the Pi open instantly.
```bash
ssh-keygen -t ed25519                      # press Enter for defaults
ssh-copy-id cobot@129.217.130.85           # asks for the Pi password once
```
Add to `~/.ssh/config`:
```
Host cobot
  HostName 129.217.130.85
  User cobot
  IdentityFile ~/.ssh/id_ed25519
  ConnectTimeout 30
  ServerAliveInterval 30
  ServerAliveCountMax 4
  ControlMaster auto
  ControlPath ~/.ssh/cm-%r@%h:%p
  ControlPersist 10m
```
Test: `ssh cobot 'hostname; uptime'` → `cobot-pi1`.

### A0b. Read-only probe of the Pi (optional; copy the whole block)
Use it after a Pi change, or when something is unexpectedly missing.
```bash
ssh cobot 'bash -s' <<'EOF'
echo "== system";   hostname; lsb_release -ds; uname -m; free -h | head -2
echo "== ROS";      ls /opt/ros; D=$(ls /opt/ros | head -1); source /opt/ros/$D/setup.bash; echo "ROS_DISTRO=$ROS_DISTRO ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
ros2 pkg list 2>/dev/null | grep -E '^(control_msgs|cv_bridge|rclpy|sensor_msgs)$'
echo "moveit packages: $(ros2 pkg list 2>/dev/null | grep -c moveit)"
echo "== python";   ~/venvs/mycobot/bin/python -c "import pymycobot; print('pymycobot', pymycobot.__version__)"
~/venvs/mycobot/bin/python -c "from pymycobot.mycobot280 import MyCobot280; print('MyCobot280 import ok')"
~/venvs/mycobot/bin/python -c "import RPi.GPIO as G; print('RPi.GPIO', G.VERSION)"
echo "== hardware"; ls -l /dev/serial0; groups; ls /dev/video* 2>&1 | head; v4l2-ctl --list-devices 2>&1 | head -12
echo "== who holds the port"; fuser -v /dev/serial0 2>&1
echo "== repo";     cd ~/mycobot-project && git status -sb | head -3; ls pp_moveit_ws/src pp_yolo_ws/src 2>&1 | head -20
EOF
```
What the answers mean:

| Output | Meaning / next step |
|---|---|
| `ls /opt/ros` empty or missing | No ROS on the Pi: the controller cannot run there. Ask the TAs. The Lab 8 notebook still works. |
| `ROS_DISTRO=jazzy` | Matches the Lab PC. A different distro: build and run everything against the Pi's distro and expect interface differences. |
| `control_msgs` missing | `sudo apt install ros-<distro>-control-msgs` (you run sudo). |
| `moveit packages: 0` on the Pi | Expected and fine: MoveIt runs on the Lab PC. |
| `MyCobot280 import ok` fails | The Pi's pymycobot is too old for `controller.py`. `pip install -U pymycobot` **inside the venv**; the laptop venv has 4.0.7. |
| `RPi.GPIO` import fails | The pump cannot be driven from Python. On Ubuntu 24.04 the compatible package is `rpi-lgpio` (`python3-rpi-lgpio`, which the Pi now has) or `python3-rpi.gpio`. |
| `v4l2-ctl: command not found` | `sudo apt install v4l-utils`; or use `ls /dev/video*` (there are usually two nodes per camera; the lower index is the video one). |
| `fuser` lists a process | Something already holds the robot port: stop that first (Jupyter serial notebook, `mycobot_control`, a controller). |

### A1. Lab PC: install (sudo) [done]
Why: MoveIt, the ros2_control messages and the xacro tool are not in a base ROS install; the packages are the dependencies listed in the MoveIt config's `package.xml` **[code]**.
```bash
sudo apt update
sudo apt install ros-jazzy-moveit ros-jazzy-moveit-configs-utils ros-jazzy-moveit-msgs ros-jazzy-control-msgs \
     ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
     ros-jazzy-joint-trajectory-controller ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro v4l-utils
```
OK = no `E:` lines. (`ros-jazzy-warehouse-ros-mongo` does not exist for Jazzy: leave it out. `gazebo_ros_control` in `package.xml` has no Jazzy package of that name: skip it, nothing in the launch files uses it **[check]**.) If another name is not found, rosdep can resolve the rest:
```bash
cd ~/mycobot-project/pp_moveit_ws && rosdep install --from-paths src --ignore-src -r -y --skip-keys "gazebo_ros_control python3-pymycobot python3-rpi.gpio"
```

### A2. Lab PC: build [done]
Why the venv must be off: colcon bakes the Python that runs it into the entry-point scripts, and ROS Jazzy packages are built for the system Python. With a venv active, the nodes would start in the wrong interpreter **[check]**.
New terminal, venv **not** active:
```bash
deactivate 2>/dev/null; source /opt/ros/jazzy/setup.bash
cd ~/mycobot-project/pp_moveit_ws
colcon build --symlink-install --packages-up-to mycobot_brain mycobot_vision mycobot_280_moveit2 mycobot_280pi
source install/setup.bash && ros2 pkg list | grep mycobot
```
OK = 6 packages: `mycobot_280_moveit2 mycobot_280pi mycobot_brain mycobot_description mycobot_interfaces mycobot_vision`. `--symlink-install` means later edits to the Python files take effect without rebuilding.

### A3. Lab PC: camera index [done: /dev/video0 → index 0]
```bash
v4l2-ctl --list-devices
```
`Logitech Webcam C930e` → first `/dev/videoN` → use `num:=N` in B4.

### A4. Pi: build the controller [done]
Why only this package and one worker: the Pi has 1.8 GB RAM and no swap; a parallel build or Jupyter running at the same time can make it hang.
```bash
ssh cobot                 # keep this window open until the build ends; if SSH drops, just rerun it
source /opt/ros/jazzy/setup.bash
cd ~/mycobot-project/pp_moveit_ws
colcon build --symlink-install --parallel-workers 1 --packages-select mycobot_controller
```
OK = `Summary: 1 package finished`. After a `git pull` on the Pi, rebuild only if the package files (not just the Python code, thanks to `--symlink-install`) changed.

### A5. Firewalls (sudo, both machines) [done: UDP verified both ways]
Why: DDS discovery and data use UDP ports that depend on the domain id; Ubuntu's firewall drops them unless allowed. Allowing everything from the one peer IP is the simplest rule for a closed lab network.
```bash
# Lab PC
sudo ufw allow from 129.217.130.85 comment 'cobot-pi1 ROS 2'
# Pi
sudo ufw allow from 129.217.130.101 comment 'CoRobot2 ROS 2'
```
Check: `sudo ufw status` shows an ALLOW line for the other machine.
Undo: `sudo ufw delete allow from <ip>`. If an IP changes (`hostname -I`), redo the rule.

### A6. `~/rosenv9.sh` on both machines [done]
Why one file: every terminal needs the same three things (ROS, the workspace overlay, the same domain id), and a typo in one terminal gives "waiting for server" with no error. `unset` removes any localhost-only setting left in `.bashrc`, which would hide the other machine.
One line, safe to paste:
```bash
printf '%s\n' 'source /opt/ros/jazzy/setup.bash' 'source ~/mycobot-project/pp_moveit_ws/install/setup.bash' 'export ROS_DOMAIN_ID=47' 'unset ROS_LOCALHOST_ONLY ROS_AUTOMATIC_DISCOVERY_RANGE' > ~/rosenv9.sh && source ~/rosenv9.sh && ros2 daemon stop; echo "domain=$ROS_DOMAIN_ID"
```
OK = `domain=47`. **Every Lab 9 terminal starts with `source ~/rosenv9.sh`.** Use the venv (`source ~/venvs/mycobot/bin/activate`) only for notebooks and the helper scripts; it has system site packages, so `rclpy` works there too. (On the home laptop the workspace path is `~/TUD/cobots/pp_moveit_ws/install/setup.bash`.)

### A7. Network test (no motion) [TODO]
1. Lab PC: `source ~/rosenv9.sh && ros2 multicast receive`   (waits)
2. Pi:     `source ~/rosenv9.sh && ros2 multicast send`
   OK = Lab PC prints `Received from 129.217.130.85:... 'Hello World!'`
3. Pi:     `ros2 topic pub /chatter std_msgs/msg/String "{data: hi}" -r 1`
   Lab PC: `ros2 topic echo /chatter`
   OK = `data: hi` every second. Ctrl+C both.

If A7 fails: stop. Check A5 (`sudo ufw status` on both), `ping -c 3 <other IP>`, and `domain=47` on both. Still failing → the lab network blocks multicast: ask the TAs (or fall back to running everything on one machine).

### Terminals without tmux (default in this guide)
`tmux` would only keep a process alive after the SSH window closes. You do not need it in the lab: you sit at the machine, so use **one terminal per process and leave it open**. The process runs in the foreground, you see its log, Ctrl+C stops it.

| Process | Machine | Start (own terminal) | Stop |
|---|---|---|---|
| controller | Pi (`ssh cobot`) | `source ~/rosenv9.sh && ros2 run mycobot_controller controller` | Ctrl+C, then `exit` |
| Jupyter (Lab 8 notebook) | Pi | `jupyter lab --no-browser --ip=127.0.0.1 --port 8888` | Ctrl+C twice |
| port forward for Jupyter | home laptop | `ssh -N -L 8888:localhost:8888 cobot` | Ctrl+C |
| move_group, camera, vision, brain | Lab PC | one terminal each, `source ~/rosenv9.sh` first | Ctrl+C (brain: `exit` in the menu first) |
| long build on the Pi | Pi | the `colcon build` of A4 | wait; rerun if SSH dropped |

A second shell on the Pi = a new terminal + `ssh cobot` (connection reuse makes it instant). If something **must survive closing the window** (e.g. Jupyter overnight), start it in the background with a log and stop it by pid:
```bash
nohup jupyter lab --no-browser --ip=127.0.0.1 --port 8888 > ~/jup.log 2>&1 &
echo $! > ~/jup.pid          # stop later: kill $(cat ~/jup.pid)
tail -f ~/jup.log            # watch the log; Ctrl+C leaves it running
```
Do **not** do this with the controller: it drives the pump and robot, and you want to see its log and be able to Ctrl+C it at once. `tmux` (`tmux new -s ctl`, detach Ctrl+B then D, `tmux attach -t ctl`) also works but is optional. **[check]** the `nohup` lines are untested on the Pi; verify there once.

---

## 5. Lab 8 runbook

Lab 8 was done on the robot on 2026-10-01 (gameplan §12). This section stays as the reference, and for experiments.

### 5a. The notebook route (needs only the Pi) [MOVES, when you switch it on]
1. Stop anything that holds the serial port (`fuser -v /dev/serial0`): the Lab 7 notebook (`ser.close()`), `mycobot_control`.
2. Start Jupyter on the Pi as in README → *Robot Pi → Start*; open `PickAndPlaceTemplate.ipynb`.
3. Work through the tasks. **Every motion cell is gated by a flag** (`enable_motion`, `enable_pump_test`) that is `False` in the template. Flip one at a time, with the workspace clear.
4. Order that is easiest on the hardware: Task 1 (connect) → 6–8 (GPIO and pump with an object held at the nozzle) → 9 (`go_home`) → 11 (one move) → 13 (full sequence, one cube at (150, 0), cube on the table) → 25–26.
5. In `solutions/Lab8_PickAndPlace_solution.ipynb` set `USE_REAL_HARDWARE = True` **only on the Pi**; elsewhere leave it `False`: it then prints what would be sent, with simulated times, and nothing moves.

### 5b. The ROS route (optional)
Needs three terminals on two machines. Order matters because `motion_node` moves on start.
1. **Lab PC:** vision (needs the three edited paths of section 3):
   ```bash
   source ~/yolovenv/bin/activate && source /opt/ros/jazzy/setup.bash && source ~/mycobot-project/pp_yolo_ws/install/setup.bash
   ros2 run vision vision_node
   ```
   First start: no calibration file, so it enters **calibration mode** (a window "Z-Scale Calibration"): place **one cube** (any colour) in the workspace, click the window and press `c` to capture a sample; it needs **5 samples with the cube in different positions** (the depth scale of Task 19); `r` resets the samples, `q` quits without calibrating. The result is written to `CALIB_FILE`, so later starts skip this step. **[code]**
2. **Check, no motion:** `ros2 topic echo /detected_objects` shows `x, y, z, color` in metres for cubes in view.
3. **Pi:** [MOVES immediately: intermediate pose, then home] `ros2 run mycobot_motion_v1 motion_node`. It waits for detections, collects them for 2 s, picks and sorts. Stop key `s` only works if you set `DEBUG_STOP_ENABLE = True`.
4. End: Ctrl+C both. `motion_node` calls `GPIO.cleanup()` only after `spin()` returns normally; after Ctrl+C it may be skipped **[check]**, so the pins can stay as they were (the pump driver is switched off by `pump_off` after every cycle, so this is mostly harmless).

---

## 6. Lab 9 runbook

### 6a. Simulation first (Lab PC or laptop, **controller OFF**)
**Vision experiments with no camera and no robot** (safe, and where most extension work happens):
```bash
# terminal 1
source ~/rosenv9.sh
python3 ~/mycobot-project/solutions/lab9_old_synthetic/publish_test_image.py --scene shapes     # or a saved frame: publish_test_image.py frame.png
# terminal 2
source ~/rosenv9.sh
ros2 run mycobot_vision vision                                         # window "Color Detection" with the mouse HSV overlay
# terminal 3: ask the service (no motion)
source ~/rosenv9.sh
ros2 service call /cube_coordinates mycobot_interfaces/srv/GetCubeCoords "{color: red}"
```
The answer is `coords=[x, y, z, roll, pitch, yaw]` in metres/radians, or six times `1.0` for "not detected". `publish_test_image.py` sends a synthetic scene (from `lab9_synthetic.py`) at 10 Hz as `camera/image`; with a real camera use `ros2 run mycobot_280pi opencv_camera --ros-args -p num:=0` instead (index from `v4l2-ctl --list-devices`).

**Motion in RViz (simulation, no robot, no controller):** [no physical motion] (runbook step B0)
```bash
source ~/rosenv9.sh
ros2 launch mycobot_brain brain_simulation.launch.py     # opens 4 gnome-terminal tabs: Brain, MoveIt demo (RViz), OpenCV camera, Vision
```
Brain tab: `2` (sort) → a colour → bin `A`; watch RViz. It needs a desktop and a camera at index 0; without a camera, replace the OpenCV tab by `publish_test_image.py` (start the other three by hand). The RViz demo contains the table, the bins and cubes, and serves the same trajectory action as the real controller. **Do not start the real controller while the simulation runs; close all tabs before B1.**

### 6b. With the robot [MOVES] (runbook steps B1–B6)
One terminal per process (see "Terminals without tmux" in section 4): one SSH terminal for the Pi, separate terminals on the Lab PC. Every terminal starts with `source ~/rosenv9.sh`.

| # | Where | Command | Expect | Why in this order |
|---|---|---|---|---|
| B1 | **Pi** (own SSH terminal) | `source ~/rosenv9.sh && ros2 run mycobot_controller controller` | logs `Joint state publisher ready!`, `Pump ready!` (pin 20 set HIGH = pump off), `FJT action server ready!`, `Current coordinates: [...]`. No motion yet. | MoveIt and brain need its action server and `/joint_states`; the log tells you the robot port opened. |
| B2 | Lab PC | `source ~/rosenv9.sh && ros2 topic echo /joint_states --once` | six joint values from the Pi: the network works. Nothing → back to A7. | Proves the network before the long-running nodes start. |
| B3 | Lab PC | `ros2 launch mycobot_280_moveit2 move_group.launch.py` | `You can start planning now!` **[check]** (MoveIt's usual last line); no errors about a missing robot description | It reads the current joint state from B1. |
| B4 | Lab PC | `ros2 run mycobot_280pi opencv_camera --ros-args -p num:=0` **or** `python3 ~/mycobot-project/solutions/lab9_old_synthetic/publish_test_image.py` | `ros2 topic hz camera/image` ≈ 10 Hz (check in another terminal) | Vision needs frames before it can answer. |
| B5 | Lab PC | `ros2 run mycobot_vision vision` | window with rectangles on the cubes, and `Cube coordinate service ready!`. Then the service call of 6a for each colour you will use: x ≈ 0.08…0.23, y ≈ −0.075…0.075 for the old mapping. `[1.0, 1.0, ...]` = colour not detected. | Brain asks this service; wrong answers here become wrong grasps. **No motion yet.** |
| B6 | Lab PC, **brain last** | `ros2 run mycobot_brain brain` | it waits for four servers (`... ready!` lines), then **moves the arm to home by itself**, then shows the menu | It is the only node that moves the arm, so everything else must be proven first. |

Before B6, do the accuracy check in 6d (D2, D3) at least once per session where the camera or the cubes may have moved.

Workspace clear, one person at the power switch. Menu **[code]**: `1` stack (green on red, then blue on green; yellow is commented out), `2` sort. Sort asks `Select Cube to Pick: red (r), yellow (y), green (g), blue (b)`, picks it, then `Select Bin to Drop: A, B, C, D`; `exit` goes back to the top. First run: `2` (sort) → one cube near the centre → bin `A`. After **every** run the arm goes **straight up (all zeros) and back** ("rejoice", section 0 item 2): keep the space above it clear.

Port busy on the Pi? `fuser -v /dev/serial0` → close the notebook kernel / stop `mycobot_control` first.

### 6c. Stopping (runbook Part C)
1. Brain menu: `exit`, then Ctrl+C in the brain terminal (it switches the pump off and removes the cubes from the planning scene in its `finally`).
2. Ctrl+C vision, camera, MoveIt.
3. On the Pi: in the controller terminal press Ctrl+C (it calls `GPIO.cleanup()`), then `exit`.
4. Pi: commit and `git push` your changes if you changed files there, then `ssh cobot 'git credential-cache exit'` (forgets the token).

**Emergency stop = the robot's power switch.** Ctrl+C in brain does not stop a trajectory already sent.

### 6d. First pick and place with the camera (runbook Part D, Lab 9 Part 0)
No code or notebook edits are needed: this runs the **unchanged course code**. The step-by-step version of this part with recording tables is Part 0 in `solutions/lab9_real_hardware/GUIDE.md`; `solutions/lab9_old_synthetic/Lab9_…ipynb` is only a reference (synthetic data); nothing at runtime uses it.

**The one real risk:** `vision.py` `send_cube_coords()` (lines 44–49) maps pixels to robot coordinates with fixed numbers measured for the **old** camera pose (the PDF says the camera moved) **[code]**:
```python
rx = 0.08   + (cy - 55)  / 280.0 * 0.15     # pixel row    → robot x (m)
ry = -0.075 + (cx - 185) / 280.0 * 0.15     # pixel column → robot y (m)
z  = 0.01                                    # fixed height
```
The comment in the code gives the fit: pixel square (185,335) BL, (465,335) BR, (465,55) TR, (185,55) TL ↔ real square (0.23,−0.075), (0.23,0.075), (0.08,0.075), (0.08,−0.075). So check the accuracy **before** `brain` (the first thing that moves the arm).

**D1. Start everything except brain.** B1–B5 (controller on the Pi → `/joint_states` check → MoveIt → camera `num:=0` → vision). No motion.

**D2. Image size (vision assumes 640×480).**
```bash
source ~/rosenv9.sh
ros2 topic echo /camera/image --once --field width;  ros2 topic echo /camera/image --once --field height
```
OK = `640` and `480`. Anything else → the pixel constants cannot be right: stop.

**D3. Accuracy check (no motion, ~10 min).** Robot base frame: origin = centre of the base, **x forward** (away from the robot, into the camera view), **y left**. Put **one red cube** at each position (ruler from the base centre) and ask vision:
```bash
ros2 service call /cube_coordinates mycobot_interfaces/srv/GetCubeCoords "{color: red}"
```
| Placed at (x, y) m | Returned (x, y) | Error |
|---|---|---|
| (0.15, 0.00) | | |
| (0.10, −0.05) | | |
| (0.20, 0.05) | | |

- Vision window: the red cube must have a **rectangle**. `[1.0, 1.0, ...]` = not detected → fix lighting/thresholds first (section 8.2; `solutions/lab9_real_hardware/hsv_calibrate.py`, GUIDE §8).
- Errors **≤ ~1 cm** → D4. Why 1 cm: the hover-then-straight-down approach tolerates a few millimetres (section 7); a few centimetres knocks the cube.
- Larger → stop. Either (a) **Task A** (ArUco calibration): the real fix, and one of your two tasks (`solutions/lab9_real_hardware/GUIDE.md` §6); or (b) a **temporary workaround** for a first run: re-fit the 4 numbers in lines 44–47 from the measured points. Mark it as a workaround in the report (it breaks again if the camera moves).

**D4. First run (B6) [MOVES].**
- One person at the power switch; workspace clear; **space above the arm clear** (rejoice).
- Start `brain` → it moves to home by itself → menu.
- Menu `2` (sort) → `r` → wait until the cube is gripped → bin `A`. Then `exit`.
- Only when that worked: `1` (stack), cubes placed as the PDF describes.

**D5. Record for the Part 0 report (PDF deliverable).** Per run: initial cube positions, menu choices, bin, outcome (success / missed / dropped). Plus:
- the **data flow** of one pick: camera → `camera/image` → vision (HSV, pixel → robot) → `/cube_coordinates` → brain → MoveIt (`/move_action`, `/compute_cartesian_path`) → controller → arm; `/pump_controller` → pump;
- **at least 2 limitations** you observed (likely: fixed pixel mapping, fixed z = 0.01, one object per colour, lighting sensitivity, open-loop trajectory replay; section 11).
Recording tables and a report skeleton: `solutions/lab9_real_hardware/GUIDE.md` §4 (Part 0).

**Known surprises on the first runs:** blue needs S and V ≥ 200 → often missed in normal light, so start with red. Weak suction: the Lab 9 controller switches only pump pin 20, not 21 (section 0 item 5). Ctrl+C in brain does not stop a move already sent; the power switch does.

---

## 7. Why it is built this way

- **Controller on the Pi, brain and MoveIt elsewhere.** Hardware access (serial, GPIO) is local to the Pi. MoveIt plans in seconds on a PC and would take most of the Pi's 1.8 GB. The only things crossing the network are small messages (`/joint_states` at 20 Hz, trajectories, pump strings). The camera image is the one big stream: keep the camera node and Vision on the same machine.
- **Vision answers a service instead of publishing continuously.** The Brain asks "where is the red cube?" at the moment it needs it: simple, and the answer is always the latest frame. It is a **single measurement**: the Brain does not update the grasp pose while approaching (the PDF calls this single-measurement PBVS). Hence the accuracy of the pixel-to-robot map is decisive.
- **HSV segmentation + rectangle test (Lab 9)** instead of a neural network (Lab 8): fast on a laptop CPU, no training data, easy to inspect (mouse overlay). The price: sensitive to lighting, one object per colour, and only squares. Lab 8's YOLO + keypoints + `solvePnP` gives a 3D pose and handles stacks, but needs torch and a depth calibration.
- **Two kinds of motion.** `move_action` (pose goal with collision checking) for the long moves; `compute_cartesian_path` (straight line, `max_step` 1 cm) for the short vertical approach and retreat, because a straight line avoids hitting the cube sideways and a pose goal does not guarantee the path.
- **Hover first.** Pick: hover 10 cm above → straight down 5 cm → pump on → straight up 10 cm. A vertical approach tolerates a few millimetres of detection error; a sideways approach would knock the cube.
- **Collision objects follow the cube.** A cube is first attached to the table (so cubes do not "collide" with each other in the scene), then re-attached to `pump_head` when picked, so the planner moves the arm *with* the cube, then back to the table when released.
- **The pump is a topic.** One `std_msgs/String` (`"on"`/`"off"`): a `Bool` was tried and abandoned because almost any input was read as true (comment in `controller.py`).
- **The controller replays waypoints open loop.** For each trajectory point it sends the joint angles and sleeps for the time to the next waypoint; it never checks that the arm arrived. Simple, but timing errors accumulate and nothing detects a stalled joint.
- **The Lab 8 sequence uses fixed sleeps too** (`MOTION_SLEEP = 1.5 s`): a slower move would be cut short by the next command. That is why a speed increase must go together with a check of the sleeps.
- **Domain 47, `ufw` rule, one `rosenv9.sh`.** The domain id keeps your group's robot apart from the other groups' on the same network; the firewall rule and the single environment file remove the two silent failure modes (blocked UDP, mismatched domain) that both look like "brain waits forever".

---

## 8. Values you can change

Rule for every change: **one value at a time**, write down old → new and what you observed, try it in the simulation or without the cube first. "Try" is guidance from the geometry, not a verified limit, and the "Why" columns are my reading of the code and the PDF, not statements from the course authors: the TAs can confirm them.

### 8.1 Lab 8 (`motion_node.py` and notebook)
| Value | Now | Why this value | Try | Effect / watch for |
|---|---|---|---|---|
| `MOVE_SPEED` | 50 | middle of the 1–100 range: fast enough, not violent | 20 → 70 | slower = more reliable approach; faster = the 1.5 s sleep may no longer cover the move |
| `MOTION_SLEEP` | 1.5 s | so that a plain move at speed 50 has probably finished before the next command (an assumption, not measured here) | 1.0 → 3.0 | too short: the next command interrupts the move; too long: the cycle is slow (19 s per cube today) |
| `NOZZLE_LENGTH_MM` | 68 | length of the suction cup from the flange (Lab 5 TCP, applied by hand) | ± 5 mm | wrong value: nozzle stops in the air or presses on the cube |
| `HOVER_MARGIN` | 70 mm | clearance above the pick height | 40 → 100 | smaller: hits the neighbours; bigger: longer moves |
| `DROP_Z_MM` | 60 mm | release height above the bin | 40 → 100 | too high: cube falls and bounces; too low: collides with the bin |
| `WORKSPACE_*` | x 75…225, y −75…75 mm | most likely the part of the table the camera sees reliably and the arm reaches | shrink | a smaller box is safer; enlarging beyond the camera view gives wrong positions |
| `HOME_COORDS`, `INTER_COORDS`, `HOME_SPEED`, `HOME_DELAY` | see notebook Task 3; 40; 4 s | home is the camera-viewing pose; the waypoint turns the wrist gradually | do not change without measuring | wrong poses can hit the table or the camera mount |
| `DETECTION_WAIT_TIME` | 2.0 s | collect several frames before choosing (flicker, stacks) | 0.5 → 4 | shorter: may miss a higher-priority cube; longer: slow |
| `BIN_COORDS`, `COLOR_PRIORITY` | notebook Tasks 5, 14 | the bins' measured positions; red first | swap priorities | no risk; check the bins are where the table says |
| vacuum timing in `pump_off` | 0.3 s then 2.0 s | let the motor stop, then vent and let the cube drop | 1.0 → 3.0 s | too short: the arm leaves with the cube stuck to the cup |
| vision `CONF_THRESH` | 0.75 | reject weak YOLO detections | 0.5 → 0.9 | lower: false positives; higher: missed cubes |
| `Z_ONE_CUBE`, `CUBE_HEIGHT` | 0.36 m, 0.04 m | depth of a one-cube top face from the camera; cube size | re-measure | wrong → every stack level and height is off (Tasks 19–20) |
| `t_base_cam`, `R_base_cam`, camera matrix | `[0.170, 0, 0.39]` m; swap-and-flip; `fx 756.8, fy 751.6, cx 327.9, cy 224.3` | calibrated for the previous camera pose | re-calibrate (Lab 9 task A method) | a shifted camera moves every detection by the same offset |

### 8.2 Lab 9 vision (`vision.py`, `opencv_camera.py`)
| Value | Now | Why | Try | Effect / watch for |
|---|---|---|---|---|
| HSV ranges | red `0–15`/`170–200`, yellow `16–55`, green `56–80`, blue `81–110`; `S ≥ 100`, `V ≥ 100` (**blue: S, V ≥ 200**) | hue picks the colour; S and V reject grey and dark; red wraps around 0 | use the mouse overlay and `lab9_real_hardware/hsv_calibrate.py`; `V ≥ 50` | hue is 0–179 in OpenCV (`200` is invalid); a too strict V loses cubes in shadow; too loose accepts the table |
| side length | 50…150 px | a 4 cm cube at this camera height | measure | smaller min: noise passes; larger max: merged cubes pass |
| aspect ratio | ≥ 0.75 | a square top is ≈ 1; allows perspective | 0.7 → 0.9 | too strict: tilted cubes lost; too loose: rectangles pass |
| pixel → robot | `x = 0.08 + (cy−55)/280·0.15`, `y = −0.075 + (cx−185)/280·0.15`, `z = 0.01` | linear map fitted to four corner points of the **previous** camera pose | replace by the Task A calibration | systematic offset if the camera moved: measure with a ruler (6d, D3) |
| `camera/image` rate, `num` | 10 Hz (timer 0.1 s), device 0 | enough for static cubes | `-p num:=2` | wrong index: `The camera was successfully turned on!` never appears |
| mask cleanup (Task B) | none in the original; 5×5 ellipse in `lab9_real_hardware/vision_B.py` (7×7 in the old notebook) | removes isolated pixels and bridges 3–4 px gaps | 5 → 11 | bigger: rounds the cube, can delete small cubes |

### 8.3 Lab 9 brain and MoveIt (`brain.py`, `mycobot_280_moveit2/config`)
| Value | Now | Why | Try | Effect / watch for |
|---|---|---|---|---|
| hover above cube | `z + 0.10` | clearance for a vertical approach | 0.08 → 0.15 | lower: risk of hitting the cube; higher: longer Cartesian move |
| descent | `−0.05` (pump head ends at `z + 0.05`) | with `z = 0.01` the pump head stops 2 cm above the cube's top face | change **together with** the returned z | see Task A step 5 |
| `place` hover | `z + 0.04·n + 0.06`, lower `−0.05` | `n` = cubes already stacked; 4 cm per cube | | a cylinder or other height breaks the 0.04 |
| home pose | `[0.16, −0.06, 0.32, 0, 1.57, 0]` | out of the camera's view of the table | do not change blindly | |
| pump orientation | pitch `3.14159` | pump pointing down | | |
| bin poses A–D | e.g. A `[0.13, 0.17, 0.15, 0, 3.14, 0]`; B has `rz = 1.57` (tube out of the way); C has `ry = 2.4` (reachability) | measured positions 15 cm above the bins | move 1–2 cm | check reachability in RViz first |
| goal tolerance | position 1 mm sphere; orientation 0.01 rad per axis | exact grasp pose | 3–5 mm, 0.05 rad | the tight values make planning fail near the edges; looser = more success, less exact |
| planning | 10 attempts, 5 s, velocity/acceleration scaling 1.0 | speed | 0.3–0.5 scaling | 1.0 × the joint limit of 2.0 rad/s is fast (default scaling in `joint_limits.yaml` is 0.5) |
| Cartesian `max_step` | 0.01 m | interpolation resolution of straight lines | 0.005 | finer = more waypoints, smoother |
| IK solver | KDL, timeout 5 ms, resolution 0.005 | `kinematics.yaml` | timeout 0.05 | the 5 ms timeout is a likely cause of planning failures **[check]** |
| `cubes_stacked` | 2 | the red cube is the base | | hard-coded stack logic |

### 8.4 Lab 9 controller (`controller.py`)
| Value | Now | Why | Effect / watch for |
|---|---|---|---|
| `/joint_states` timer | 0.05 s (20 Hz) | MoveIt needs the current state at planning start | slower: stale start state |
| speed per point | `int(100·max|velocity|)` | scale MoveIt's speed to the 0–100 API | see section 0 item 7 |
| waypoint sleep | `time_from_start` difference, skipped if ≤ 0.01 s | follow MoveIt's timing | open loop |
| pump | GPIO 20 LOW = on, HIGH = off | active-low driver | section 0 item 5 (pin 21) |

### 8.5 Network and environment
| Value | Now | Why | Effect / watch for |
|---|---|---|---|
| `ROS_DOMAIN_ID` | 47 (both machines) | separates your group's topics from other robots on the network | must be identical everywhere; a different value shows up as `brain` waiting forever |
| `ufw` rule | allow all from the peer IP | simplest rule that lets DDS UDP through | a stricter rule (specific ports) needs the port formula of your DDS; if an IP changes, redo A5 |
| camera `num` | 0 | `/dev/video0` is the C930e on the Lab PC | change if you plug in another camera |

---

## 9. Experiments, safest first

| # | Experiment | Needs | Measure |
|---|---|---|---|
| 9.1 | Tune the HSV thresholds with the mouse overlay on a saved frame; compare `lab9_real_hardware/hsv_calibrate.py` output with the fixed ranges | no robot | masks before/after, false and missed detections |
| 9.2 | Change the morphology kernel 3 → 7 → 11 on frames with a cable or a reflection (real frames from `lab9_real_hardware/record_frames.py`) | no robot | whether the cube stays one blob; centre shift in px |
| 9.3 | Read the service answers for 5 known positions (ruler) before and after your calibration (6d, D3 extended) | camera, `vision` | position error in mm (mean, max) |
| 9.4 | Same cube, two lightings | camera | which colours disappear; threshold margins |
| 9.5 | In RViz (`demo.launch.py`): vary hover height, tolerances, scaling factors | simulation | planning success rate, time per move |
| 9.6 | Simulation: observe what the controller would receive: log `speed` values | simulation | whether speeds exceed 100 (section 0 item 7) |
| 9.7 | Real robot, one cube near the centre, bin A: record one full cycle with a stopwatch; then change **one** of speed/hover/tolerance | robot [MOVES] | cycle time, success, grasp offset in mm |
| 9.8 | Pump test: compare Lab 8's `pump_on` (both pins LOW) with the Lab 9 topic (pin 20 only) holding the same cube | pump | hold time, release |
| 9.9 | Task E: ask for an absent colour, two same-colour objects | vision + service | `found` flag, ids, which match is chosen |

Keep a table: value, old, new, result, photo or log. The interview asks why a value is what it is.

---

## 10. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `source: command not found` in a script | paste artefact `^[[200~` in the file | recreate it with the A6 one-liner |
| `brain` stays at "Waiting for ... service/server" | a node is not running, or other domain id | the things it waits for: Vision (`/cube_coordinates`), MoveIt (`move_action`, `/compute_cartesian_path`), Controller (`/arm_group_controller/follow_joint_trajectory`). `ros2 node list`, `ros2 service list`, `ros2 action list`, `echo $ROS_DOMAIN_ID` on both machines. |
| `/joint_states` not seen on the Lab PC, or `ros2 topic list` shows nothing from the other machine | domain id differs, multicast blocked, firewall, controller not running | A7; `sudo ufw status` on both; look at the controller's terminal |
| Vision window does not open / `cannot connect to display` | no display (SSH, tmux) | run Vision on the Lab PC (needs its display), not over SSH |
| `AttributeError: module 'numpy' has no attribute 'int0'` | NumPy 2 in the Python that runs `vision` | use NumPy < 2 (ROS Jazzy ships 1.26) or replace `np.int0(box)` by `box.astype(int)` |
| `.../camera/image` stays empty | wrong device index, camera in use, no camera | A3 (`v4l2-ctl --list-devices`), `-p num:=<n>`, or `lab9_old_synthetic/publish_test_image.py` (no camera) |
| Coordinates `[1,1,1,1,1,1]` | colour not detected (or stale-free after Task B) | look at the window: is the cube outlined? thresholds, lighting (blue needs S, V ≥ 200) |
| Planning fails ("Trajectory execution failed with error code ...") | unreachable pose, 1 mm tolerance, 5 ms IK timeout, collision objects in the way | RViz: is the goal reachable? loosen the tolerance; clear stale collision objects (restart `brain`) |
| `AttributeError ... error_code.val` after a failed Cartesian path | bug in `send_cartesian_path` (the result is an int) | known issue in section 11; not your mistake |
| Arm ends above or in the cube | z convention / camera offset | Task A step 5; measure with a ruler (6d, D3) |
| `could not open port /dev/serial0` | another process holds it | `fuser -v /dev/serial0`; stop it |
| Pi slow, hangs, SSH times out | out of memory | `free -h` on the Pi; nothing but the controller should run there; stop Jupyter or MoveIt; gameplan §9 rule 7 |

---

## 11. Problems found in the code (for the report, "limitations")

`vision.py`: persistent `detected_cubes` (stale detections), constants for an old camera pose, `z = 0.01`, `np.int0`, blue S/V ≥ 200 vs 100 in the README, red hue up to 200, `cv2.namedWindow` in the constructor, one object per colour.
`brain.py`: recursive menus, `rejoice()` after each run, `error_code.val` on an `int`, `"world"` frame in the Cartesian request, tight goal tolerances, box centre at 0.01, id = colour, hard-coded `cubes_stacked = 2`.
`controller.py`: open-loop replay, speed mapping (up to 200 vs 0–100), only pin 20 for the pump, no cancel handler.
`motion_node.py` (Lab 8): moves on start, stop key disabled by default, hardcoded `/home/tejas/...` paths in the vision node.
Details and the improved versions: `lab9_real_hardware/GUIDE.md` (Part 0, Tasks A, B, C on the real system), `lab9_old_synthetic/Lab9_PickAndPlace_MoveIt_solution.ipynb` (Tasks A–E on synthetic data, reference) and `Lab8_PickAndPlace_solution.ipynb` (Task 24).
