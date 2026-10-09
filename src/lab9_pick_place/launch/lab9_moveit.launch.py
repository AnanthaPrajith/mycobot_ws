"""MoveIt move_group for Lab 9 on the real robot: the course mycobot_280_moveit2/move_group.launch.py plus longer
execution time limits. The course file is not changed.

Why: the real controller sends waypoints over serial and is slower than the plan. With the defaults (x1.2 + 0.5 s)
every execution ended TIMED_OUT while the arm kept moving (the controller cannot be cancelled), 2026-10-05.
"""
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2").to_moveit_configs()
    moveit_config.trajectory_execution.update({
        "trajectory_execution.allowed_start_tolerance": 0.3,             # as in the course move_group.launch.py
        "trajectory_execution.allowed_execution_duration_scaling": 4.0,  # Lab 9
        "trajectory_execution.allowed_goal_duration_margin": 5.0,        # Lab 9
    })
    return generate_move_group_launch(moveit_config)
