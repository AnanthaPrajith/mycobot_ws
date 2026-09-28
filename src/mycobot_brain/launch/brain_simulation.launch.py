from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    def run_in_tab(title, command_list):
        return ExecuteProcess(
            cmd=['gnome-terminal', '--tab', f'--title={title}', '--'] + command_list,
            output='screen'
        )
  
    # Tab: Brain
    tab_brain = TimerAction(
        period=0.0,
        actions=[run_in_tab('Brain', ['ros2', 'run', 'mycobot_brain', 'brain'])]
    )

    # Tab: MoveIt
    tab_moveit = TimerAction(
        period=0.0,
        actions=[run_in_tab('MoveIt', ['ros2', 'launch', 'mycobot_280_moveit2', 'demo.launch.py'])]
    )

    # Tab: OpenCV
    tab_opencv = TimerAction(
        period=0.0,
        actions=[run_in_tab('OpenCV', ['ros2', 'run', 'mycobot_280pi', 'opencv_camera'])]
    )

    # Tab: Vision
    tab_vision = TimerAction(
        period=0.0,
        actions=[run_in_tab('Vision', ['ros2', 'run', 'mycobot_vision', 'vision'])]
    )

    return LaunchDescription([
        tab_brain,
        tab_moveit,
        tab_opencv,
        tab_vision
    ])