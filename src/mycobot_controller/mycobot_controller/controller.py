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
        GPIO.setup(20, GPIO.OUT)
        GPIO.output(20, 1)
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
        elif state_handle.data == "off":
            GPIO.output(20, 1)
        else:
            self.get_logger().error(f"Invalid pump state: {state_handle}. Provide 'on' to turn on or 'off' to turn off.")
        self.get_logger().info(f"Pump is now {state_handle.data}!")


    # execute trajectory calculated by moveit
    def execute_trajectory(self, goal_handle):
        self.get_logger().info("Executing trajectory...")
        waypoints = goal_handle.request.trajectory.points
        print(waypoints)
        for i, point in enumerate(waypoints):
            # send joint angles
            speed = int(100 * np.max(np.abs(point.velocities)))         # this sets the speed to the max. single-joint-velocity defined by moveit
            self.mc.send_radians(point.positions, speed)

            # sleep duration till next waypoint
            if i < (len(waypoints) - 1):                                # this insures that there is no error on the last waypoint
                duration = ((waypoints[i+1].time_from_start.sec + waypoints[i+1].time_from_start.nanosec * 1e-9) 
                - (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9))
                if duration > 0.01:                                     # this ensures that there is no error for extremely short waypoint durations 
                    time.sleep(duration)                                # (shouldnt happen anyways)

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