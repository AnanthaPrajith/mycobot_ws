"""Conservative Cartesian keyboard jogging through MoveIt and ROS actions.

This node never opens the robot serial port.  It plans one small base-frame
Cartesian displacement at a time, publishes the result for RViz inspection,
and requires both an explicit enable command and per-move confirmation before
asking MoveIt to execute the planned trajectory.
"""

import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener


class MeasurementJog(Node):
    """Plan and optionally execute small pump-head moves in g_base."""

    def __init__(self):
        super().__init__('measurement_jog')
        self.declare_parameter('step_mm', 5.0)
        self.declare_parameter('velocity_scale', 0.05)
        self.declare_parameter('acceleration_scale', 0.05)
        self.declare_parameter('x_limits_m', [0.075, 0.225])
        self.declare_parameter('y_limits_m', [-0.075, 0.075])
        self.declare_parameter('z_limits_m', [0.080, 0.420])

        self.step_mm = float(self.get_parameter('step_mm').value)
        self.velocity_scale = float(self.get_parameter('velocity_scale').value)
        self.acceleration_scale = float(
            self.get_parameter('acceleration_scale').value)
        self.limits = {
            'x': tuple(float(v) for v in self.get_parameter('x_limits_m').value),
            'y': tuple(float(v) for v in self.get_parameter('y_limits_m').value),
            'z': tuple(float(v) for v in self.get_parameter('z_limits_m').value),
        }
        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = ActionClient(self, MoveGroup, '/move_action')
        self.cartesian_path = self.create_client(
            GetCartesianPath, '/compute_cartesian_path')
        self.execute_trajectory = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')
        self.display_pub = self.create_publisher(
            DisplayTrajectory, '/display_planned_path', 1)
        self.execution_enabled = False

    def _validate_parameters(self):
        if not 1.0 <= self.step_mm <= 10.0:
            raise ValueError('step_mm must be between 1 and 10 mm')
        for name, value in (
            ('velocity_scale', self.velocity_scale),
            ('acceleration_scale', self.acceleration_scale),
        ):
            if not 0.0 < value <= 0.10:
                raise ValueError(f'{name} must be greater than 0 and at most 0.10')
        for axis, bounds in self.limits.items():
            if len(bounds) != 2 or bounds[0] >= bounds[1]:
                raise ValueError(f'invalid {axis}_limits_m')

    def wait_for_interfaces(self):
        self.get_logger().info('Waiting for MoveIt planning action...')
        if not self.move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/move_action is unavailable; start move_group')
        self.get_logger().info('Waiting for MoveIt Cartesian-path service...')
        if not self.cartesian_path.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                '/compute_cartesian_path is unavailable; start move_group')
        self.get_logger().info('Waiting for MoveIt execution action...')
        if not self.execute_trajectory.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('/execute_trajectory is unavailable; start move_group')
        self.current_pose(timeout_sec=5.0)

    def current_pose(self, timeout_sec=2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                transform = self.tf_buffer.lookup_transform(
                    'g_base', 'pump_head', Time(),
                    timeout=Duration(seconds=0.1))
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                pose.orientation = transform.transform.rotation
                return pose
            except TransformException:
                pass
        raise RuntimeError('No g_base -> pump_head TF; start robot_state_publisher')

    @staticmethod
    def describe_pose(pose):
        return (
            f'X={pose.position.x * 1000:.1f} mm, '
            f'Y={pose.position.y * 1000:.1f} mm, '
            f'Z={pose.position.z * 1000:.1f} mm, '
            f'q=[{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, '
            f'{pose.orientation.z:.3f}, {pose.orientation.w:.3f}]'
        )

    def target_for(self, command):
        target = self.current_pose()
        delta = self.step_mm / 1000.0
        moves = {
            'x+': ('x', delta), 'x-': ('x', -delta),
            'y+': ('y', delta), 'y-': ('y', -delta),
            'z+': ('z', delta), 'z-': ('z', -delta),
        }
        if command == 'down':
            # MoveIt pump-head convention used by brain.py: RPY [0, pi, 0].
            target.orientation.x = 0.0
            target.orientation.y = 1.0
            target.orientation.z = 0.0
            target.orientation.w = 0.0
            return target
        axis, amount = moves[command]
        setattr(target.position, axis, getattr(target.position, axis) + amount)
        return target

    def within_limits(self, pose):
        for axis in ('x', 'y', 'z'):
            value = getattr(pose.position, axis)
            low, high = self.limits[axis]
            if not low <= value <= high:
                self.get_logger().error(
                    f'Rejected: {axis.upper()}={value:.3f} m is outside '
                    f'[{low:.3f}, {high:.3f}] m')
                return False
        return True

    def make_request(self, target):
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.002]
        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(target)

        position = PositionConstraint()
        position.header.frame_id = 'g_base'
        position.link_name = 'pump_head'
        position.constraint_region = volume
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = 'g_base'
        orientation.link_name = 'pump_head'
        orientation.orientation = target.orientation
        orientation.absolute_x_axis_tolerance = 0.02
        orientation.absolute_y_axis_tolerance = 0.02
        orientation.absolute_z_axis_tolerance = 0.02
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)

        request = MotionPlanRequest()
        request.group_name = 'arm_group'
        request.start_state.is_diff = True
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale
        request.goal_constraints.append(constraints)
        return request

    @staticmethod
    def validate_small_trajectory(trajectory):
        """Reject IK branch changes that are inappropriate for a small jog."""
        points = trajectory.joint_trajectory.points
        if len(points) < 2:
            return False, 'trajectory has fewer than two points'
        first = points[0].positions
        if not first:
            return False, 'trajectory has no joint positions'
        max_excursion = 0.0
        max_increment = 0.0
        previous = first
        for point in points[1:]:
            if len(point.positions) != len(first):
                return False, 'trajectory joint dimensions changed'
            max_excursion = max(
                max_excursion,
                max(abs(value - start) for value, start in zip(point.positions, first)),
            )
            max_increment = max(
                max_increment,
                max(abs(value - old) for value, old in zip(point.positions, previous)),
            )
            previous = point.positions
        # A 1-10 mm Cartesian jog should not require an IK branch change or a
        # large discontinuous joint step. Reject it instead of guessing.
        if max_excursion > 0.20:
            return False, f'joint excursion {max_excursion:.3f} rad exceeds 0.20 rad'
        if max_increment > 0.05:
            return False, f'joint increment {max_increment:.3f} rad exceeds 0.05 rad'
        return True, ''

    def publish_plan(self, start_state, trajectory):
        display = DisplayTrajectory()
        display.model_id = 'firefighter'
        display.trajectory_start = start_state
        display.trajectory.append(trajectory)
        self.display_pub.publish(display)

    def plan_cartesian(self, target):
        request = GetCartesianPath.Request()
        request.header.frame_id = 'g_base'
        request.header.stamp = self.get_clock().now().to_msg()
        request.start_state.is_diff = True
        request.group_name = 'arm_group'
        request.link_name = 'pump_head'
        request.waypoints = [target]
        request.max_step = 0.001
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.velocity_scale
        request.max_acceleration_scaling_factor = self.acceleration_scale

        future = self.cartesian_path.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            self.get_logger().error('Cartesian-path service returned no response')
            return None
        if response.error_code.val != 1 or response.fraction < 0.999:
            self.get_logger().error(
                f'Cartesian jog rejected: fraction={response.fraction:.3f}, '
                f'MoveIt error={response.error_code.val}')
            return None
        valid, reason = self.validate_small_trajectory(response.solution)
        if not valid:
            self.get_logger().error(f'Unsafe jog plan rejected: {reason}')
            return None
        self.publish_plan(response.start_state, response.solution)
        return response.solution

    def plan_orientation_preview(self, target):
        goal = MoveGroup.Goal()
        goal.request = self.make_request(target)
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        send_future = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveIt rejected the planning request')
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(
                f'Planning failed with MoveIt error {result.error_code.val}')
            return None
        if not result.planned_trajectory.joint_trajectory.points:
            self.get_logger().error('MoveIt returned an empty trajectory')
            return None

        self.publish_plan(result.trajectory_start, result.planned_trajectory)
        return result.planned_trajectory

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_trajectory.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveIt rejected trajectory execution')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(
                f'Execution failed with MoveIt error {result.error_code.val}')
            return False
        return True

    def handle_move(self, command):
        target = self.target_for(command)
        if not self.within_limits(target):
            return
        print(f'Target: {self.describe_pose(target)}')
        trajectory = (self.plan_orientation_preview(target) if command == 'down'
                      else self.plan_cartesian(target))
        if trajectory is None:
            return
        points = trajectory.joint_trajectory.points
        duration = points[-1].time_from_start
        seconds = duration.sec + duration.nanosec * 1e-9
        print(f'Plan published to RViz: {len(points)} points, {seconds:.2f} s.')
        if command == 'down':
            print('The down-orientation command is preview-only because it may '
                  'require a large joint-space move.')
            return
        if not self.execution_enabled:
            print('Execution locked. Inspect the plan; use "enable" when safe.')
            return
        if input('Type yes to execute this one move: ').strip().lower() != 'yes':
            print('Not executed.')
            return
        if self.execute(trajectory):
            actual = self.current_pose(timeout_sec=3.0)
            print(f'Actual: {self.describe_pose(actual)}')

    def run(self):
        self.wait_for_interfaces()
        print(__doc__)
        print('Commands: status, x+, x-, y+, y-, z+, z-, down,')
        print('          step N, enable, disable, quit')
        print('Execution begins LOCKED. Keep access to physical robot power.')
        while rclpy.ok():
            command = input('jog> ').strip().lower()
            if command in {'quit', 'q'}:
                return
            if command == 'status':
                print(self.describe_pose(self.current_pose()))
            elif command == 'disable':
                self.execution_enabled = False
                print('Execution locked.')
            elif command == 'enable':
                confirmation = input('Type ENABLE after checking the scene: ').strip()
                self.execution_enabled = confirmation == 'ENABLE'
                print('Execution enabled.' if self.execution_enabled else 'Still locked.')
            elif command.startswith('step '):
                try:
                    step = float(command.split()[1])
                    if not 1.0 <= step <= 10.0:
                        raise ValueError
                    self.step_mm = step
                    print(f'Step set to {step:.1f} mm.')
                except (ValueError, IndexError):
                    print('Step must be from 1 to 10 mm.')
            elif command in {'x+', 'x-', 'y+', 'y-', 'z+', 'z-', 'down'}:
                self.handle_move(command)
            elif command:
                print('Unknown command.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MeasurementJog()
        node.run()
    except (KeyboardInterrupt, EOFError):
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'measurement_jog: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
