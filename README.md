# myCobot ROS 2 workspace

Source workspace containing robot descriptions, MoveIt configuration, control,
custom messages, motion nodes, vision, notebooks, and YOLO weights.

For the corrected ArUco/Brain pick-and-place application, follow
[the ordered robot-PC setup and calibration guide](README_ROBOT_PC.md).
It covers updating, building, calibration, validation, and supervised execution.

## Humble and Jazzy

The existing workspace was set up with **ROS 2 Humble**. Native Debian packages
use Ubuntu 22.04 for Humble and Ubuntu 24.04 for Jazzy. See the official
[Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
and [Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Clone the same source on either machine and rebuild for its installed ROS version.
Never copy `build/`, `install/`, or `log/` between machines or ROS distributions.
They are excluded from Git. Jazzy build and runtime compatibility have **not yet
been verified**; MoveIt configuration, Python dependencies, and hardware access
need validation on the destination machine. Changing the sourced ROS version
alone does not prove compatibility.

For an unchanged Humble environment on a Jazzy host, a separate Ubuntu 22.04 /
Humble container or VM is another option; camera, serial, GPIO, and GUI access
must be configured for that environment.

## Clone and build

Install ROS 2 on the target system first. Open a fresh terminal that does not
source another workspace or ROS distribution, then run:

```bash
git clone <YOUR_REPOSITORY_URL> mycobot_ws
cd mycobot_ws
# Use humble on the Humble machine, jazzy on the Jazzy machine.
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-venv
# Run once per machine, only if rosdep has not already been initialized:
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -r -y \
  --skip-keys "python3-pymycobot python-tk"
colcon build --symlink-install
source install/setup.bash
```

The skipped manifest keys are handled separately: `python3-pymycobot` refers to
the vendor Python SDK, and the legacy `python-tk` entry needs Python 3 Tk on these
systems (`sudo apt install python3-tk`). Do not ignore other rosdep failures;
resolve them before building. The Raspberry Pi controller also needs `RPi.GPIO`
and physical GPIO/serial hardware; it is not a desktop hardware simulator.

For nodes that import the vendor SDK, use a virtual environment created with the
system Python so ROS modules remain accessible:

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install pymycobot
# Build Python entry points with this interpreter if using this environment:
python -m colcon build --symlink-install
source install/setup.bash
```

Use a clean clone/build when switching distributions. Do not source Humble and
Jazzy in the same terminal. Communicating between Humble and Jazzy nodes over the
network is a separate compatibility task; prefer the same ROS distribution on
communicating machines.

## Vision and machine-specific configuration

`requirements/vision/requirements.txt` is an existing pinned environment snapshot,
not a verified cross-distribution dependency lock. In particular its NumPy 2.4
and SciPy 1.17 pins cannot be used with Humble's default Python 3.10. Do not install
it blindly into the ROS system Python. Resolve the vision environment for the
target Python version and check NumPy/OpenCV compatibility with `cv_bridge` if
using `mycobot_vision`. The `vision` YOLO node and `mycobot_vision` are separate
packages.

The YOLO model is included at `resources/weights/best.pt`. Configure the `vision`
node from the cloned workspace root:

```bash
export MYCOBOT_YOLO_MODEL="$PWD/resources/weights/best.pt"
export MYCOBOT_CALIB_FILE="$HOME/z_scale_calibration.json"
export MYCOBOT_CAMERA=/dev/video2
ros2 run vision vision_node
```

The environment variables override legacy defaults. Camera intrinsics and the
camera-to-robot transform in `src/vision/vision/vision_node.py` are specific to the
original setup and need calibration for a changed camera or mounting position.

Other existing machine-specific settings still require configuration:

- `src/mycobot_control/launch/mycobot_control.launch.py`: serial port and external
  `mycobot_280_gazebo.urdf` path. That exact URDF is not included in this workspace;
  supply the correct model and matching link names before using Cartesian control.
- `src/mycobot_control/scripts/fk_tool_calib.py` and `ik_debug.py`: local URDF paths.
- Raspberry Pi nodes: serial device, baud rate, GPIO pins, and device permissions.

See individual package READMEs for launch commands. Validate planning and
configuration before running hardware nodes.

## Repository contents

Git includes source, package manifests, launch/config files, notebooks, meshes,
CAD assets, and the YOLO model. Generated builds, caches, virtual environments,
local editor settings, and common credential files are excluded. Dependencies
must be installed on each destination system; they are not embedded in the repo.
