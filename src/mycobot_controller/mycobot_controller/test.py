# smooth_trajectory_executor.py
# Ersatz für sync_plan — führt ganze Trajektorie flüssig aus

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot import MyCobot
import math
import time
import threading


class SmoothTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('smooth_trajectory_executor')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 1000000)
        # Geschwindigkeit 1-100, höher = schneller aber ungenauer
        self.declare_parameter('speed', 60)

        port  = self.get_parameter('port').value
        baud  = self.get_parameter('baud').value
        self.speed = self.get_parameter('speed').value

        self.mc = MyCobot(port, baud)
        time.sleep(0.5)  # kurz warten bis Serial bereit
        self.get_logger().info(f'Verbunden: {port} @ {baud}')

        # Action Server — genau die Action die MoveIt2 aufruft
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda req: CancelResponse.ACCEPT,
        )
        self.get_logger().info('Action Server bereit.')

    async def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        points     = trajectory.points

        self.get_logger().info(
            f'Trajektorie empfangen: {len(points)} Stützpunkte'
        )

        if not points:
            goal_handle.abort()
            return FollowJointTrajectory.Result()

        # Zeitpunkte aus der Trajektorie lesen
        t_start = self.get_clock().now()

        for i, point in enumerate(points):
            # Zielwinkel Rad → Grad
            angles_deg = [math.degrees(a) for a in point.positions]

            # Geschwindigkeit dynamisch aus Zeitdifferenz ableiten
            if i == 0:
                dt = point.time_from_start.nanoseconds / 1e9
            else:
                dt = (
                    point.time_from_start.nanoseconds -
                    points[i - 1].time_from_start.nanoseconds
                ) / 1e9

            # Geschwindigkeit: kürzere dt = schnellere Bewegung nötig
            # myCobot speed 1-100, grob: speed = clamp(30/dt, 10, 100)
            dynamic_speed = int(min(100, max(10, 30 / dt))) if dt > 0 else self.speed

            self.mc.send_angles(angles_deg, dynamic_speed)

            # Auf nächsten Zeitpunkt warten (relativ zu t_start)
            t_target = t_start + rclpy.duration.Duration(
                nanoseconds=point.time_from_start.nanoseconds
            )
            now = self.get_clock().now()
            remaining = (t_target - now).nanoseconds / 1e9

            if remaining > 0.005:  # nur warten wenn >5ms übrig
                time.sleep(remaining)

            # Abbruch prüfen
            if goal_handle.is_cancel_requested:
                self.mc.stop()
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            self.get_logger().debug(
                f'Punkt {i+1}/{len(points)} | '
                f'Winkel: {[f"{a:.1f}" for a in angles_deg]} | '
                f'Speed: {dynamic_speed} | dt: {dt:.3f}s'
            )

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Trajektorie abgeschlossen.')
        return result


def main():
    rclpy.init()
    node = SmoothTrajectoryExecutor()
    # MultiThreadedExecutor damit Action Server nicht blockiert
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()