from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mycobot_control',
            executable='mycobot_control',
            name='mycobot_control',
            output='screen',
            parameters=[{
                'port': '/dev/serial0',
                'baud': 1000000,
                'num_joints': 6,

                'cmd_tick_hz': 20.0,
                'state_rate_hz': 5.0,

                'deadband': 0.01,
                'max_joint_speed_deg_s': 150.0,
                'ee_min_time_s': 0.5,
                'ee_replan_max': 0,

                'stop_burst': 2,
                'stop_opcode': 0x29,
                'resume_opcode': 0x28,
                'send_stop_on_startup': False,
                'zero_behavior': 'stop',
                'hold_speed': 1,

                'speed_mode': 'max',
                'fixed_speed': 50,
                'max_step_deg': 2.0,
                'init_read_rate_hz': 1.0,
                'read_timeout_s': 0.75,
                'allow_uninitialized': False,
                'accel_limit_rad_s2': 1.0,
                'read_while_streaming': False,
                'publish_estimated_states': True,

                'pos_kp': 2.0,
                'position_tolerance_rad': 0.01,
                'urdf_path': '/home/mycobot/ros2_ws/src/mycobot_280_gazebo.urdf',
                'ik_base_link': 'g_base',
                'ik_ee_link': 'joint6_flange',
                'ik_base_xyz': [0.0003774967, 0.0003297874, -0.0015915640],
                'ik_base_rpy': [-4.76e-05, 2.29e-05, 1.87e-04],
                'ik_tool_xyz': [-0.0008809655, -0.0017207283, 0.0001719996],
                'ik_tool_rpy': [4.91e-05, 2.52e-04, 5.37e-05],
                'ee_pose_dedup_pos_mm': 0.5,
                'ee_pose_dedup_rot_deg': 0.5,
                'ee_pose_dedup_window_s': 30.0,
                'ee_pose_max_time_s': 10.0,
                'ee_pose_lock_active': True,

                'log_tx': True,
                'tx_log_period_s': 1.0,
            }]
        )
    ])
