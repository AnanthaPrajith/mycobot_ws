#!/usr/bin/env python3
"""Request a home trajectory from MoveIt without executing it."""
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint


def main():
    cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'src/lab9_pick_place/config/lab9.yaml').read_text())
    names=['joint2_to_joint1','joint3_to_joint2','joint4_to_joint3','joint5_to_joint4','joint6_to_joint5','joint6output_to_joint6']
    angles=cfg['robot']['home_joints_deg']
    rclpy.init();node=Node('plan_home_only')
    try:
        client=node.create_client(GetMotionPlan,'/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt planning service unavailable.')
        req=GetMotionPlan.Request();mr=req.motion_plan_request
        mr.group_name='arm_group';mr.num_planning_attempts=10;mr.allowed_planning_time=5.0
        mr.max_velocity_scaling_factor=.2;mr.max_acceleration_scaling_factor=.2
        mr.start_state.is_diff=True
        constraints=Constraints()
        for name,angle in zip(names,angles):
            constraints.joint_constraints.append(JointConstraint(joint_name=name,position=math.radians(angle),tolerance_above=.01,tolerance_below=.01,weight=1.0))
        mr.goal_constraints=[constraints]
        future=client.call_async(req)
        rclpy.spin_until_future_complete(node,future,timeout_sec=30)
        if not future.done() or future.result() is None:raise RuntimeError('Home planning timed out.')
        response=future.result().motion_plan_response
        print('HOME_JOINTS_DEG:',angles)
        print('PLAN_ERROR_CODE:',response.error_code.val)
        print('TRAJECTORY_POINTS:',len(response.trajectory.joint_trajectory.points))
        print('No trajectory was executed.')
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
