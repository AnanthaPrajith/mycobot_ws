from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("firefighter", package_name="mycobot_280_moveit2").to_moveit_configs()

    # The Jazzy generate_moveit_rviz_launch() helper does not pass the URDF or
    # SRDF to RViz.  When RViz and move_group are launched separately, the
    # MotionPlanning display consequently reports "Could not find robot model".
    # Supply the complete model explicitly so this launch file is standalone.
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=str(moveit_config.package_path / "config" / "moveit.rviz"),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([rviz_config, rviz])
