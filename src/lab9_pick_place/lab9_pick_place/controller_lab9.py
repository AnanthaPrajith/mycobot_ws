"""Lab 9 controller for the Pi: copy of the course mycobot_controller/controller.py with two fixes (2026-10-05).
The course file stays unchanged; run THIS instead of it (never both: one serial port, one set of GPIO pins).

  1. pump on/off also switches the release valve GPIO 21 (as in Lab 8), so objects are released
  2. smoother trajectory execution: one waypoint per >= 0.25 s, speed 40..90, wait until the arm reached the last point

Same node name, topics and action as the course controller, so MoveIt and both brains use it unchanged.
Run on the Pi: ros2 run lab9_pick_place controller_lab9
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280
import RPi.GPIO as GPIO

# Lab 9 (10-05): smoother execution. The original sent every MoveIt waypoint at speed 100 x waypoint velocity
# (often 5-20), so the arm got a new slow target every few ms and moved stop-and-go. Labs 7/8 sent 1 command at 50.
MIN_DT = 0.25                    # s between commands to the robot (always including the last waypoint)
SPEED_MIN, SPEED_MAX = 40, 90    # send_radians speed range
ARRIVE_TOL = 0.03                # rad (~1.7 deg): last waypoint reached
ARRIVE_TIMEOUT = 5.0             # s to wait for the arm to arrive before reporting success


def t_of(point):
    return point.time_from_start.sec + point.time_from_start.nanosec * 1e-9

class Controller(Node):
    def __init__(self):
        super().__init__("controller")
        # initialize MyCobot instance
        self.mc = MyCobot280("/dev/serial0", 1000000)
        
        # initialize joint state publisher
        self.state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.get_logger().info("Joint state publisher ready!")
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        # initialize pump topic subscriber and GPIO pin (pump off)
        self.pump_subscriber = self.create_subscription(String, "/pump_controller", self.control_pump, 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(20, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.output(20, 1)
        GPIO.setup(21, GPIO.OUT, initial=GPIO.HIGH)                    # Lab 9: release valve (active low), as in Lab 8
        GPIO.output(21, 1)
        self.get_logger().info("Pump ready!")

        # initiate trajectory action server
        self.trajectory_server = ActionServer(self, FollowJointTrajectory, "/arm_group_controller/follow_joint_trajectory", execute_callback=self.execute_trajectory)
        self.get_logger().info("FJT action server ready!")

        self.get_logger().info(f"Current coordinates: {self.mc.get_coords()}")

        # this is just for fun
        self.rejoice_subscriber = self.create_subscription(Bool, "/rejoice", self.rejoice, 10)


    # publish joint angles (with 20 Hz)
    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.header.frame_id = "g_base"
        joint_state_msg.name = [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6"
            ]
        joint_state_msg.position = self.mc.get_radians()

        self.state_publisher.publish(joint_state_msg)


    # control pump (on/off)
    def control_pump(self, state_handle):
        if state_handle.data == "on":               # bool state didnt work here,
            GPIO.output(20, 0)                 # because almost every input was interpreted as 'true'
            GPIO.output(21, 0)                 # Lab 9: valve closed while sucking (Lab 8 pump_on)
        elif state_handle.data == "off":
            GPIO.output(20, 1)
            time.sleep(0.3)
            GPIO.output(21, 1)                 # Lab 9: open the valve so the object is released (Lab 8 pump_off)
        else:
            self.get_logger().error(f"Invalid pump state: {state_handle}. Provide 'on' to turn on or 'off' to turn off.")
        self.get_logger().info(f"Pump is now {state_handle.data}!")


    # execute trajectory calculated by moveit
    def execute_trajectory(self, goal_handle):
        points = goal_handle.request.trajectory.points
        # Lab 9: keep one waypoint every MIN_DT seconds plus the last one
        waypoints, t_last = [], -1e9
        for i, point in enumerate(points):
            if i == len(points) - 1 or t_of(point) - t_last >= MIN_DT:
                waypoints.append(point)
                t_last = t_of(point)
        self.get_logger().info(f"Executing trajectory: {len(waypoints)} of {len(points)} waypoints...")
        for i, point in enumerate(waypoints):
            # send joint angles; speed from the max. single-joint velocity defined by moveit, limited to 40..90
            vel = np.max(np.abs(point.velocities)) if len(point.velocities) else 0.0
            speed = int(np.clip(100 * vel, SPEED_MIN, SPEED_MAX))
            self.mc.send_radians(list(point.positions), speed)

            # sleep duration till next waypoint
            if i < (len(waypoints) - 1):
                duration = t_of(waypoints[i + 1]) - t_of(point)
                if duration > 0.01:
                    time.sleep(duration)

        # Lab 9: wait until the arm is at the last waypoint, so the next plan starts from the real pose
        target = np.array(waypoints[-1].positions)
        t0 = time.time()
        while time.time() - t0 < ARRIVE_TIMEOUT:
            q = self.mc.get_radians()
            if isinstance(q, list) and len(q) == 6 and np.max(np.abs(np.array(q) - target)) < ARRIVE_TOL:
                break
            time.sleep(0.1)
        else:
            self.get_logger().warn(f"arm not at the last waypoint after {ARRIVE_TIMEOUT} s")

        result = FollowJointTrajectory.Result()
        result.error_code = 0
        self.get_logger().info("Trajectory successfully executed!")
        self.get_logger().info(f"Current coordinates: {self.mc.get_coords()}")
        goal_handle.succeed()
        return result
    

    def rejoice(self, handle):
        if handle.data == True:
            self.mc.send_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 40)
            time.sleep(2)
            self.mc.send_angles([0.0, -60, 15.0, -45.0, 0.0, 0.0], 30)
            time.sleep(3)
            self.mc.send_angles([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 50)



def main():
    rclpy.init()
    node = Controller()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == "__main__":
    main()