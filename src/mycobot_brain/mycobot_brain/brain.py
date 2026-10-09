import sys
import time
import copy
import math
import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import GetCubeCoords
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, CollisionObject, AttachedCollisionObject, PlanningScene, AllowedCollisionEntry, AllowedCollisionMatrix
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String, Bool
from scipy.spatial.transform import Rotation

class Brain(Node):
    def __init__(self):
        super().__init__("mycobot_brain")
        
        # Coordinates are cube TOP / pump_head poses in g_base (m, rad).
        self.declare_parameter("cube_size_m", 0.04)
        self.declare_parameter("sort_colors", ["red", "green", "blue"])
        # Supply measured pump_head release poses; SDK flange poses need conversion.
        self.declare_parameter("red_bin_pose", [0.0] * 6)
        self.declare_parameter("other_bin_pose", [0.0] * 6)
        self.declare_parameter("bin_poses_verified", False)
        self.cube_size = float(self.get_parameter("cube_size_m").value)
        if not math.isfinite(self.cube_size) or self.cube_size <= 0:
            raise ValueError("cube_size_m must be finite and positive")
        self.sort_colors = list(self.get_parameter("sort_colors").value)
        if (len(self.sort_colors) != 3 or len(set(self.sort_colors)) != 3
                or "red" not in self.sort_colors
                or any(c not in {"red", "yellow", "green", "blue"} for c in self.sort_colors)):
            raise ValueError("sort_colors must contain red and two distinct supported colors")
        self.held_color = None
        self.stack_base_coords = None

        # initialize pump controller topic publisher
        self.pump_publisher = self.create_publisher(String, "/pump_controller", 10)

        # initialize cube coordinate service client
        self.cube_coords_client = self.create_client(GetCubeCoords, "/cube_coordinates")
        self.get_logger().info("Waiting for cube coordinates service...")
        self.cube_coords_client.wait_for_service()
        self.get_logger().info("Cube coordinates service ready!")

        # initialize cube collision object publisher
        self.cube_publisher = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.attached_cube_publisher = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        
        # initialize moveit trajectory action client
        self.trajectory_client = ActionClient(self, MoveGroup, "move_action")
        self.get_logger().info("Waiting for MoveIt action server...")
        self.trajectory_client.wait_for_server()
        self.get_logger().info("MoveIt action server ready!")

        # initialize moveit cartesian path service client and followjointtrajectory action client
        self.cartesian_client = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.get_logger().info("Waiting for MoveIt cartesian path service...")
        self.cartesian_client.wait_for_service()
        self.get_logger().info("MoveIt cartesian path service ready!")
        self.cartesian_acton_client = ActionClient(self, FollowJointTrajectory, "/arm_group_controller/follow_joint_trajectory")
        self.get_logger().info("Waiting for controller action server...")
        self.cartesian_acton_client.wait_for_server()
        self.get_logger().info("Controller action server ready!")

        # this is just for fun
        self.rejoice_publisher = self.create_publisher(Bool, "/rejoice", 10)


    @staticmethod
    def valid_coords(coords):
        try:
            return (coords is not None and len(coords) == 6
                    and all(math.isfinite(float(v)) for v in coords))
        except (TypeError, ValueError):
            return False

    # Keep futures bounded and never interpret missing replies as success.
    def wait_for_result(self, future):
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done():
            self.get_logger().error("ROS request timed out; stop and check execution before retrying.")
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"ROS request failed: {exc}")
            return None

    # sends request for cube coordinates of specific color
    # returns coords from response
    def get_cube_coords(self, color):
        request = GetCubeCoords.Request()
        request.color = color
        future = self.cube_coords_client.call_async(request)
        response = self.wait_for_result(future)
        if response is None or not self.valid_coords(response.coords):
            self.get_logger().error("No valid cube coordinates received")
            return False

        # if cube is not detected
        if list(response.coords) == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]:
            self.get_logger().warn(f"{color} cube not detected!")
            return False
        
        self.get_logger().info(f"{color} cube coordinates: {response.coords}")
        return list(response.coords)
    

    # spawns a cube in the moveit planning scene
    def spawn_cube(self, color, coords=None):
        center_coords = self.get_cube_coords(color) if coords is None else list(coords)
        
        # if cube was not detected
        if not center_coords:
            return
        
        cube = CollisionObject()
        cube.id = color                                                 # id to later attach cube to ee
        cube.header.frame_id = "g_base"
        cube.operation = CollisionObject.ADD

        # cube pose definition
        # cube pose coordinates
        cube.pose.position.x = float(center_coords[0])         
        cube.pose.position.y = float(center_coords[1])
        cube.pose.position.z = float(center_coords[2]) - self.cube_size / 2
        # cube pose orientation converted to quaternion
        rot = center_coords[3:6]
        quat = Rotation.from_euler('xyz', rot).as_quat()
        cube.pose.orientation.x = float(quat[0])
        cube.pose.orientation.y = float(quat[1])
        cube.pose.orientation.z = float(quat[2])
        cube.pose.orientation.w = float(quat[3])

        # cube shape definition
        # (4 cm)³ cube
        shape = SolidPrimitive()
        shape.type = SolidPrimitive.BOX
        shape.dimensions = [self.cube_size] * 3

        # shape pose definition
        # the shapes pose is is defined relative to cube.pose and is the same as the cubes pose
        shape_pose = Pose()
        shape_pose.position.x = 0.0
        shape_pose.position.y = 0.0
        shape_pose.position.z = 0.0
        shape_pose.orientation.x = 0.0
        shape_pose.orientation.y = 0.0
        shape_pose.orientation.z = 0.0
        shape_pose.orientation.w = 1.0

        # cube definition
        cube.primitives.append(shape)
        cube.primitive_poses.append(shape_pose)

        # add cube via collision object topic
        self.cube_publisher.publish(cube)
        self.get_logger().info(f"Cube '{color}' added to planning scene!")

        # attach cube to env table to prevent collision detection between cubes
        self.attach_cube(color, "env_table")


    # removes a cube in the moveit planning scene
    # i added this fct so that there is no trajectory planning error on the cube pick up
    def destroy_cube(self, color):
        if color == self.held_color:
            self.get_logger().error("Cannot remove the held cube from the planning scene")
            return
        self.detach_cube(color, "env_table")
        cube = CollisionObject()
        cube.id = color
        cube.operation = CollisionObject.REMOVE

        # remove cube via collision object topic
        self.cube_publisher.publish(cube)
        self.get_logger().info(f"Cube '{color}' removed from planning scene!")


    # attaches cube to the ee pump head or env table in the moveit simulation
    def attach_cube(self, color, link="pump_head"):
        # which cube
        cube = AttachedCollisionObject()
        cube.object.id = color
        cube.object.operation = CollisionObject.ADD
        cube.link_name = link
        # had to add pump_box because the pump boxes mesh is also the worlds ground plane
        cube.touch_links = ["pump_head", "pump_box", "env_table", "red", "yellow", "green", "blue"]

        # add cube via attached collision object topic
        self.attached_cube_publisher.publish(cube)
        self.get_logger().info(f"Cube '{color}' attached to {link}!")     


    # detaches cube from the ee pump head in the moveit simulation
    def detach_cube(self, color, link="pump_head"):
        # which cube
        cube = AttachedCollisionObject()
        cube.object.id = color
        cube.object.operation = CollisionObject.REMOVE
        cube.link_name = link
        cube.touch_links = ["pump_head", "pump_box", "env_table", "red", "yellow", "green", "blue"]

        # add cube via attached collision object topic
        self.attached_cube_publisher.publish(cube)
        self.get_logger().info(f"Cube '{color}' detached from {link}!")     


    # send desired pump state to controller node
    def send_pump_state(self, goal_state):
        goal_msg = String()
        goal_msg.data = goal_state
        self.pump_publisher.publish(goal_msg)
        time.sleep(0.5)
        self.get_logger().info(f"Pump is {goal_state}...")
        

    # sends goal pose and request to execute trajectory collision-free to moveit action server
    # returns the result error code as int 
    def send_goal_pose(self, goal_coords):    
        if not self.valid_coords(goal_coords):
            return None
        # goal pose definition
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "g_base"
        goal_pose.header.stamp = self.get_clock().now().to_msg()
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
        self.get_logger().info(f"Goal pose received: {goal_pose.pose}")

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
        orientation_constraints.absolute_x_axis_tolerance = 0.01
        orientation_constraints.absolute_y_axis_tolerance = 0.01
        orientation_constraints.absolute_z_axis_tolerance = 0.01
        orientation_constraints.weight = 1.0

        # constraints definition
        constraints = Constraints()
        constraints.position_constraints.append(position_contraints)
        constraints.orientation_constraints.append(orientation_constraints)

        # trajectory request definition
        trajectory_request = MotionPlanRequest()
        # moveit configugartion
        trajectory_request.group_name = "arm_group"
        trajectory_request.num_planning_attempts = 10
        trajectory_request.allowed_planning_time = 5.0
        trajectory_request.max_velocity_scaling_factor = 0.1
        trajectory_request.max_acceleration_scaling_factor = 0.1
        # add contraints to trajectory request
        trajectory_request.goal_constraints.append(constraints)

        # action goal definition
        goal = MoveGroup.Goal()
        goal.request = trajectory_request
        
        # send goal pose to moveit action server
        self.get_logger().info("Sending goal pose to MoveIt action server...")
        goal_future = self.trajectory_client.send_goal_async(goal)
        # handle goal result
        goal_handle = self.wait_for_result(goal_future)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().info("Goal pose denied!")
            return
        self.get_logger().info("Goal pose accepted!")

        # send trajectory execution request to moveit action server
        result_future = goal_handle.get_result_async()
        # handle result
        result_handle = self.wait_for_result(result_future)
        if result_handle is None:
            return None
        if not result_handle.result.error_code.val == 1:
            self.get_logger().error(f"Trajectory execution failed with error code: {result_handle.result.error_code.val}")
            return result_handle.result.error_code.val
        self.get_logger().info("Trajectory successfully executed!")
        return result_handle.result.error_code.val
    

    def send_cartesian_path(self, goal_coords):
        if not self.valid_coords(goal_coords):
            return None
        request = GetCartesianPath.Request()
        request.header.frame_id = "g_base"
        request.header.stamp = self.get_clock().now().to_msg()
        request.avoid_collisions = True

        # start pose
        request.start_state.is_diff = True 
        request.group_name = "arm_group"
        request.link_name = "pump_head"
        
        # goal pose definition
        goal_pose = Pose()
        # goal pose coordinates
        goal_pose.position.x = float(goal_coords[0])         
        goal_pose.position.y = float(goal_coords[1])
        goal_pose.position.z = float(goal_coords[2])
        # goal pose orientation converted to quaternion
        rot = goal_coords[3:6]
        quat = Rotation.from_euler('xyz', rot).as_quat()
        goal_pose.orientation.x = float(quat[0])
        goal_pose.orientation.y = float(quat[1])
        goal_pose.orientation.z = float(quat[2])
        goal_pose.orientation.w = float(quat[3])
        self.get_logger().info(f"Goal pose received: {goal_pose}")

        request.waypoints = [goal_pose]
        request.max_step = 0.01
        #request.jump_threshold = 0.0
    
        # call cartesian path service to get trajectory
        future = self.cartesian_client.call_async(request)
        response = self.wait_for_result(future)
        if (response is None or response.error_code.val != 1
                or not math.isfinite(response.fraction) or response.fraction < 0.999999
                or not response.solution.joint_trajectory.points):
            self.get_logger().error("Cartesian path incomplete or invalid; not executing")
            return None
        cartesian_path = response.solution.joint_trajectory
        # A zero stamp lets the controller start the accepted trajectory immediately.
        cartesian_path.header.stamp.sec = 0
        cartesian_path.header.stamp.nanosec = 0
        
        # send execution request to controller action server
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = cartesian_path
        goal_future = self.cartesian_acton_client.send_goal_async(goal)
        goal_handle = self.wait_for_result(goal_future)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Cartesian execution goal rejected")
            return None
        # wait till completion
        result_future = goal_handle.get_result_async()
        result_handle = self.wait_for_result(result_future)
        if result_handle is None:
            return None
        if not result_handle.result.error_code == 0:
            self.get_logger().error(f"Cartesian path execution failed with error code: {result_handle.result.error_code}")
            return result_handle.result.error_code
        self.get_logger().info("Cartesian path successfully executed!")
        return result_handle.result.error_code


    # go to home pose
    def go_home(self):
        home = [0.16, -0.06, 0.32, 0, 1.57, 0]
        success = self.send_goal_pose(home) == 1
        if success:
            self.get_logger().info("Returned to home position!")
        return success


    def pick(self, color):
        if self.held_color is not None:
            self.get_logger().error("Resolve the held object before another pickup")
            return False
        cube_coords = self.get_cube_coords(color)
        if not cube_coords:
            return False
        coords = cube_coords[:]
        coords[3:6] = [0.0, math.pi, 0.0]  # pump_head faces down

        # hover, then descend the full clearance to the detected top surface
        coords[2] += 0.1
        if self.send_goal_pose(coords) != 1:
            return False
        coords[2] -= 0.1
        if self.send_cartesian_path(coords) != 0:
            return False

        self.send_pump_state("on")
        self.held_color = color
        self.detach_cube(color, "env_table")
        self.attach_cube(color, "pump_head")
        coords[2] += 0.1
        if self.send_cartesian_path(coords) != 0:
            return False  # preserve suction; the menu must stop for recovery
        self.get_logger().info(f"{color} cube pickup commands completed")
        return self.go_home()


    def place(self, color_top, color_bottom, number):
        # number = total cubes after placement. Cache the original red top,
        # because vision's fixed plane cannot re-localize a raised stack.
        if self.held_color != color_top or number < 2:
            return False
        base = self.stack_base_coords
        if base is None:
            base = self.get_cube_coords(color_bottom)
        if not base:
            return False
        coords = list(base)
        coords[2] += self.cube_size * (number - 1)
        coords[3:6] = [0.0, math.pi, 0.0]
        placed_top = coords[:]

        coords[2] += 0.06
        if self.send_goal_pose(coords) != 1:
            return False
        coords[2] -= 0.06
        if self.send_cartesian_path(coords) != 0:
            return False

        self.send_pump_state("off")
        self.held_color = None
        self.detach_cube(color_top, "pump_head")
        # Update released geometry to its new position before retreating.
        self.spawn_cube(color_top, placed_top)
        coords[2] += 0.06
        if self.send_cartesian_path(coords) != 0:
            return False
        self.detach_cube(color_top, "env_table")
        self.get_logger().info(f"{color_top} cube placed on the stack")
        return self.go_home()


    def drop(self, color, coords):
        if self.held_color != color or self.send_goal_pose(coords) != 1:
            return False
        self.send_pump_state("off")
        self.held_color = None
        self.detach_cube(color, "pump_head")
        self.spawn_cube(color, coords)
        self.get_logger().info(f"Release commands completed for {color} bin")
        return self.go_home()


    def rejoice(self):
        rejoice = Bool()
        rejoice.data = True
        self.rejoice_publisher.publish(rejoice)
        self.get_logger().info("Rejoice sequence complete! Thank you!")


    def choose_pick(self):
        # Validate destinations before picking anything. Keep template poses out
        # of hardware runs until measured pump_head poses have been supplied.
        if not self.get_parameter("bin_poses_verified").value:
            self.get_logger().error("Set verified red_bin_pose and other_bin_pose first")
            return False
        for name in ("red_bin_pose", "other_bin_pose"):
            pose = self.get_parameter(name).value
            if not self.valid_coords(pose) or list(pose) == [0.0] * 6:
                self.get_logger().error(f"Invalid {name}")
                return False
        aliases = {"r": "red", "y": "yellow", "g": "green", "b": "blue"}
        while True:
            color = input(f"Select cube {self.sort_colors}, all, or exit: ").strip().lower()
            if color == "exit":
                return True
            color = aliases.get(color, color)
            if color != "all" and color not in self.sort_colors:
                self.get_logger().warn("Invalid color for the configured two-bin task")
                continue
            for candidate in ("red", "yellow", "green", "blue"):
                self.destroy_cube(candidate)
                self.spawn_cube(candidate)
            for selected in self.sort_colors if color == "all" else [color]:
                if not self.get_cube_coords(selected):
                    continue
                if not self.pick(selected) or not self.choose_drop(selected):
                    return False
            if color == "all":
                return True  # one pass; template assumes at most one cube per color


    def choose_drop(self, color):
        # Red has its own bin; the configured other colors share the second bin.
        if color not in self.sort_colors or not self.get_parameter("bin_poses_verified").value:
            return False
        name = "red_bin_pose" if color == "red" else "other_bin_pose"
        coords = list(self.get_parameter(name).value)
        if not self.valid_coords(coords) or coords == [0.0] * 6:
            return False
        return self.drop(color, coords)


    def stack_cubes(self):
        self.stack_base_coords = self.get_cube_coords("red")
        if not self.stack_base_coords:
            return False  # establish the support before picking anything
        for color in ("red", "green", "blue"):
            self.destroy_cube(color)
            self.spawn_cube(color)
        cubes_stacked = 1
        for color in ("green", "blue"):
            if not self.get_cube_coords(color):
                continue  # blue can go directly on red if green is absent
            if not self.pick(color):
                return False
            if not self.place(color, "red", cubes_stacked + 1):
                return False
            cubes_stacked += 1
        return cubes_stacked > 1


    def control_menu(self):
        self.send_pump_state("off")
        if not self.go_home():
            return
        while True:
            print("\n--- MyCobot Control Menu ---")
            print("1 - Stack Cubes: red -> green -> blue")
            print("2 - Sort Cubes into Two Bins")
            print("exit - Quit")
            mode = input("Select mode:\n").strip().lower()
            if mode == "exit":
                return
            if mode in ("1", "stack"):
                success = self.stack_cubes()
            elif mode in ("2", "sort"):
                success = self.choose_pick()
            else:
                self.get_logger().warn("Invalid choice!")
                continue
            if not success:
                self.get_logger().error("Sequence stopped; inspect the robot before restarting")
                return
            # Keep rejoice() available, but do not automatically move after a task.



def main():
    rclpy.init()
    node = Brain()

    try:
        node.control_menu()

    except KeyboardInterrupt:
        pass
    
    finally:
        if node.held_color is None:
            node.send_pump_state("off")
            for color in ("red", "yellow", "green", "blue"):
                node.destroy_cube(color)
        else:
            node.get_logger().error(
                "Object may be held: suction left on; support it and recover manually")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
