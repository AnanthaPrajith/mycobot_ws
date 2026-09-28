import sys
import time
import copy
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


    # sends request for cube coordinates of specific color
    # returns coords from response
    def get_cube_coords(self, color):
        request = GetCubeCoords.Request()
        request.color = color
        future = self.cube_coords_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        # if cube is not detected
        if list(response.coords) == [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]:
            self.get_logger().warn(f"{color} cube not detected!")
            return False
        
        self.get_logger().info(f"{color} cube coordinates: {response.coords}")
        return response.coords
    

    # spawns a cube in the moveit planning scene
    def spawn_cube(self, color):
        center_coords = self.get_cube_coords(color)
        
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
        cube.pose.position.z = float(center_coords[2])
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
        shape.dimensions = [0.04, 0.04, 0.04]

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
        trajectory_request.max_velocity_scaling_factor = 1.0
        trajectory_request.max_acceleration_scaling_factor = 1.0
        # add contraints to trajectory request
        trajectory_request.goal_constraints.append(constraints)

        # action goal definition
        goal = MoveGroup.Goal()
        goal.request = trajectory_request
        
        # send goal pose to moveit action server
        self.get_logger().info("Sending goal pose to MoveIt action server...")
        goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        # handle goal result
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal pose denied!")
            return
        self.get_logger().info("Goal pose accepted!")

        # send trajectory execution request to moveit action server
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        # handle result
        result_handle = result_future.result()
        if not result_handle.result.error_code.val == 1:
            self.get_logger().error(f"Trajectory execution failed with error code: {result_handle.result.error_code.val}")
            return result_handle.result.error_code.val
        self.get_logger().info("Trajectory successfully executed!")
        return result_handle.result.error_code.val
    

    def send_cartesian_path(self, goal_coords):
        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
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
        rclpy.spin_until_future_complete(self, future)
        cartesian_path = future.result().solution.joint_trajectory
        cartesian_path.header.stamp = self.get_clock().now().to_msg()
        
        # send execution request to controller action server
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = cartesian_path
        goal_future = self.cartesian_acton_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        
        # wait till completion
        goal_handle = goal_future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_handle = result_future.result()
        if not result_handle.result.error_code == 0:
            self.get_logger().error(f"Cartesian path execution failed with error code: {result_handle.result.error_code.val}")
            return result_handle.result.error_code
        self.get_logger().info("Cartesian path successfully executed!")
        return result_handle.result.error_code


    # go to home pose
    def go_home(self):
        home = [0.16, -0.06, 0.32, 0, 1.57, 0]
        self.send_goal_pose(home)
        self.get_logger().info("Returned to home position!")

    
    def pick(self, color):
        cube_coords = self.get_cube_coords(color)

        # if cube to pick is not detected
        if not cube_coords:
            return False

        coords = cube_coords[:]                     # so that cube_coords isnt affected

        # hover over cube
        coords[2] += 0.1                            # height + 10 cm
        coords[4]= 3.14159                          # pump facing down
        self.send_goal_pose(coords)

        # lower down on cube
        coords [2] -= 0.05
        self.send_cartesian_path(coords)

        # pick up cube
        self.send_pump_state("on")
        self.detach_cube(color, "env_table")
        self.attach_cube(color, "pump_head")

        # retract up
        coords[2] += 0.1
        self.send_cartesian_path(coords)
        
        self.get_logger().info(f"{color} cube picked up!")
        self.go_home()

        return True

    
    def place(self, color_top, color_bottom, number):
        extra_height = 0.04 * number                        # extra height depending on the number of cubes already stacked
        coords = self.get_cube_coords(color_bottom)

        # if cube to place on is not detected
        if not coords:
            return False

        # hover over cube
        coords[2] += extra_height + 0.06
        coords[4] = 3.14159                         # pump facing down
        self.send_goal_pose(coords)

        # lower down
        coords [2] -= 0.05                          # move down 5 cm
        self.send_cartesian_path(coords)

        # place cube
        self.send_pump_state("off")
        self.detach_cube(color_top, "pump_head")
        
        # retract up
        coords[2] += + 0.05
        self.attach_cube(color_top, "env_table")        # this is to disable collisions with pump head
        self.send_cartesian_path(coords)
        self.detach_cube(color_top, "env_table")        # this is to enable collision with pump head again

        self.get_logger().info(f"{color_top} cube placed on top of {color_bottom} cube!")
        self.go_home()

        return True
    

    def drop(self, color, coords):
        # hover over place spot
        self.send_goal_pose(coords)

        # place cube
        self.send_pump_state("off")
        self.detach_cube(color, "pump_head")
        self.attach_cube(color, "env_table")
        self.get_logger().info(f"{color} cube in the bin!")

        self.go_home()


    def rejoice(self):
        rejoice = Bool()
        rejoice.data = True
        self.rejoice_publisher.publish(rejoice)
        self.get_logger().info("Rejoice sequence complete! Thank you!")


    def choose_pick(self):
        # destroy old cubes
        self.destroy_cube("red")
        self.destroy_cube("yellow")
        self.destroy_cube("green")
        self.destroy_cube("blue")

        # spawn cubes
        self.spawn_cube("red")
        self.spawn_cube("yellow")
        self.spawn_cube("green")
        self.spawn_cube("blue")
        
        color = input("Select Cube to Pick: red (r), yellow (y), green (g), blue (b)\n")
        color = color.lower()

        if not color in ["r", "red", "y", "yellow", "g", "green", "b", "blue", "exit"]:
            self.get_logger().warn("Invalid choice!")
            self.choose_pick()
        elif color in ["r", "red"]:
            if self.pick("red"):
                self.choose_drop("red")
        elif color in ["y", "yellow"]:
            if self.pick("yellow"):
                self.choose_drop("yellow")
        elif color in ["g", "green"]:
            if self.pick("green"):
                self.choose_drop("green")
        elif color in ["b", "blue"]:
            if self.pick("blue"):
                self.choose_drop("blue")
        elif color == "exit":
            self.send_pump_state("off")
            self.control_menu()

        self.choose_pick()


    def choose_drop(self, color):
        binA = [0.13, 0.17, 0.15, 0.0, 3.14, 0.0]  
        binB = [0.0, 0.17, 0.15, 0.0, 3.14, 1.57]           # rz = 1.57 so that the pump tube is out of the way
        binC = [0.23, -0.16, 0.15, 0.0, 2.4, 0.0]           # ry = 2.4 so that the goal pose is reachable  
        binD = [0.13, -0.16, 0.15, 0.0, 3.14, 0.0]
        
        bin = input("Select Bin to Drop: A, B, C, D\n")
        bin = bin.lower()

        if not bin in ["a", "b", "c", "d", "exit"]:
            self.get_logger().warn("Invalid choice!")
            self.choose_drop(color)
        elif bin == "a":
            self.drop(color, binA)
        elif bin == "b":
            self.drop(color, binB)
        elif bin == "c":
            self.drop(color, binC)
        elif bin == "d":
            self.drop(color, binD)
        elif bin == "exit":
            self.send_pump_state("off")
            self.control_menu()


    def stack_cubes(self):
        # destroy old cubes
        self.destroy_cube("red")
        self.destroy_cube("yellow")
        self.destroy_cube("green")
        self.destroy_cube("blue")

        # spawn cubes
        self.spawn_cube("red")
        self.spawn_cube("yellow")
        self.spawn_cube("green")
        self.spawn_cube("blue")
        
        cubes_stacked = 2
        
        if self.pick("green"):
            self.go_home()
            if self.place("green", "red", cubes_stacked):
                cubes_stacked += 1

        if self.pick("blue"):
            self.go_home()
            if self.place("blue", "green", cubes_stacked):
                cubes_stacked += 1

        #if self.pick("yellow"):
        #    self.go_home()
        #    self.place("yellow", "blue", cubes_stacked)


    def control_menu(self):
        # initial setup
        self.send_pump_state("off")
        self.go_home()
        
        print("\n--- MyCobot Control Menu ---")
        print("1 - Stack Cubes: red -> green -> blue")
        print("2 - Sort Cubes into Bins")
        print("-------------------------------")
        mode = input("Select mode:\n")
        mode = mode.lower()

        if mode in ["1", "stack"]:
            self.get_logger().info("Starting cube stack sequence...")
            self.stack_cubes()
            self.get_logger().info("Cube stack sequence executed!")
            self.rejoice()
        elif mode in ["2", "sort"]:
            self.get_logger().info("Starting cube sort program...")
            self.choose_pick()
            self.rejoice()
        else:
            self.get_logger().warn("Invalid choice!")
        
        self.control_menu()



def main():
    rclpy.init()
    node = Brain()

    try:
        node.control_menu()

    except KeyboardInterrupt:
        pass
    
    finally:
        node.send_pump_state("off")
        node.destroy_cube("red")
        node.destroy_cube("yellow")
        node.destroy_cube("green")
        node.destroy_cube("blue")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()