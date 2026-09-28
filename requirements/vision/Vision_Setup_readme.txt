See ../../README.md for Humble/Jazzy setup and vision configuration.

The requirements.txt here is an original pinned environment snapshot, not a
verified environment for both ROS distributions. Its NumPy and SciPy pins do
not support Humble's default Python 3.10. Resolve dependencies for your target
Python version in a virtual environment created with --system-site-packages.
Use the system Python belonging to the installed ROS distribution.

After installing compatible vision dependencies and rebuilding the workspace:
  source /opt/ros/<humble-or-jazzy>/setup.bash
  source .venv/bin/activate
  source install/setup.bash
  export MYCOBOT_YOLO_MODEL="$PWD/resources/weights/best.pt"
  export MYCOBOT_CALIB_FILE="$HOME/z_scale_calibration.json"
  export MYCOBOT_CAMERA=/dev/video2
  ros2 run vision vision_node

Run these commands from the workspace root. Calibrate for the actual hardware.
