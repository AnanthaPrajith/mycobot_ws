import sys
import time 
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from scipy.spatial.transform import Rotation

class Brain_Old(Node):
    def __init__(self):
        super().__init__("mycobot_brain")
        self.trajectory_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.get_logger().info('Waiting for MoveIt Action Server...')
        self.trajectory_client.wait_for_service()
        self.get_logger().info('MoveIt Action Server ready!')
    
    # returns the trajectory to get to goal pose collision-free
    def get_trajectory(self, goal_coords):
        # moveit configugartion
        trajectory_request = GetMotionPlan.Request()
        trajectory_request.motion_plan_request.group_name = "arm_group"
        trajectory_request.motion_plan_request.num_planning_attempts = 10
        trajectory_request.motion_plan_request.allowed_planning_time = 5.0
        trajectory_request.motion_plan_request.max_velocity_scaling_factor = 1.0
        trajectory_request.motion_plan_request.max_acceleration_scaling_factor = 1.0
        
        # goal pose definition
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "g_base"
        # goal pose coordinates
        goal_pose.pose.position.x = float(goal_coords[0])         
        goal_pose.pose.position.y = float(goal_coords[1])
        goal_pose.pose.position.z = float(goal_coords[2])
        # goal pose orientation converted to quaternion
        rot = goal_coords[3:6]
        quat = Rotation.from_euler('xyz', rot).as_quat()
        goal_pose.pose.orientation.x = float(quat[0])
        goal_pose.pose.orientation.y = float(quat[1])
        goal_pose.pose.orientation.z = float(quat[2])
        goal_pose.pose.orientation.w = float(quat[3])

        print(f"Goal pose:{goal_pose.pose}")

        # postion constraints definition
        # sphere of 1 mm radius
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.001]        
        # tolerance sphere for ee position definition
        tolerance_sphere = BoundingVolume()
        tolerance_sphere.primitive_poses.append(goal_pose.pose)
        tolerance_sphere.primitives.append(sphere)
        # position constraints definition
        position_contraints = PositionConstraint()
        position_contraints.header.frame_id = "g_base"
        position_contraints.link_name = "pump_head"
        position_contraints.constraint_region = tolerance_sphere
        position_contraints.weight = 1.0

        # orientation constraints definition
        orientation_constraints = OrientationConstraint()
        orientation_constraints.header.frame_id = "g_base"
        orientation_constraints.link_name = "pump_head"
        orientation_constraints.orientation = goal_pose.pose.orientation
        orientation_constraints.absolute_x_axis_tolerance = 0.1
        orientation_constraints.absolute_y_axis_tolerance = 0.1
        orientation_constraints.absolute_z_axis_tolerance = 0.1
        orientation_constraints.weight = 1.0

        # constraints definition
        constraints = Constraints()
        constraints.position_constraints.append(position_contraints)
        constraints.orientation_constraints.append(orientation_constraints)

        # add contraints to trajectory request
        trajectory_request.motion_plan_request.goal_constraints.append(constraints)

        # call trajectory request
        return self.trajectory_client.call_async(trajectory_request)
        

def main():
    rclpy.init()
    node = Brain_Old()
    try:
        test = [0.1, 0, 0.3, 0, 0, 3.14159]
        future = node.get_trajectory(test)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()
        node.get_logger().info(f"response: {result}")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()