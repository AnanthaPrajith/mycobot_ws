from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2").to_moveit_configs()
    
    moveit_config.trajectory_execution.update({                         # yannick: added this to fix a planning error
        "trajectory_execution.allowed_start_tolerance": 0.3             # where two joints deviated
    })                                                                  # more than 0.01 from start state

    return generate_move_group_launch(moveit_config)
