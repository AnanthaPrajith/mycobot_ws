#!/usr/bin/env python3
"""Check current joints against MoveIt's collision scene; sends no motion goals."""
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetStateValidity


def main():
    rclpy.init()
    node=Node('check_robot_state')
    received=[]
    sub=node.create_subscription(JointState,'/joint_states',lambda msg: received.append(msg),1)
    client=node.create_client(GetStateValidity,'/check_state_validity')
    try:
        if not client.wait_for_service(timeout_sec=10):raise RuntimeError('MoveIt state validity service unavailable.')
        deadline=time.monotonic()+10
        while not received and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.2)
        if not received:raise RuntimeError('No joint states received.')
        state=received[-1]
        if len(state.name)!=6 or len(state.position)!=6:raise RuntimeError('Expected six robot joint positions.')
        req=GetStateValidity.Request()
        req.group_name='arm_group'
        req.robot_state.joint_state=state
        req.robot_state.is_diff=False
        future=client.call_async(req)
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or future.result() is None:raise RuntimeError('State validity check timed out.')
        response=future.result()
        print('CURRENT_STATE_VALID:',response.valid)
        for contact in response.contacts:
            print('COLLISION:',contact.contact_body_1,'<->',contact.contact_body_2)
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
