"""Lab 9 on the real robot, Lab PC side: MoveIt (lab9_moveit.launch.py), camera and vision_lab9, one gnome-terminal tab each.

Not started here: controller_lab9 (runs on the Pi, start it first) and brain_lab9 (start it last, after the checks in
solutions/lab9_real_hardware/README.md, because it moves the arm home as soon as it starts).
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def tab(title, cmd):
    return ExecuteProcess(cmd=["gnome-terminal", "--tab", f"--title={title}", "--"] + cmd, output="screen")


def generate_launch_description():
    return LaunchDescription([
        tab("MoveIt", ["ros2", "launch", "lab9_pick_place", "lab9_moveit.launch.py"]),
        tab("Camera", ["ros2", "run", "mycobot_280pi", "opencv_camera"]),
        tab("Vision", ["ros2", "run", "lab9_pick_place", "vision_lab9"]),
    ])
