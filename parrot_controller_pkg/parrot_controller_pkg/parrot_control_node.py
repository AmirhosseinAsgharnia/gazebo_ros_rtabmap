#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from actuator_msgs.msg import Actuators
from tf_transformations import euler_from_quaternion


class FullDroneController(Node):
    def __init__(self):
        super().__init__('full_drone_controller')
        
        # Subscribe to drone pose (position + orientation)
        self.pose_subscriber = self.create_subscription(
            PoseArray,
            '/model/parrot_bebop_2/pose',
            self.pose_callback,
            100)
        
        # Publish motor speed commands
        self.motor_speed_publisher = self.create_publisher(
            Actuators,
            '/parrot_bebop_2/command/motor_speed',
            100)
        
        # Target position & orientation
        self.target_x = 0.0  # Keep drone at X = 0
        self.target_y = 0.0  # Keep drone at Y = 0
        self.target_z = 1.0  # Maintain altitude at 1 meter
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0  # Keep facing the same direction

        # PID gains for Altitude (Z)
        self.kp_alt = 2
        self.ki_alt = 0
        self.kd_alt = 2

        # PID gains for X and Y movement
        self.kp_x = 0
        self.ki_x = 0
        self.kd_x = 0

        self.kp_y = 0
        self.ki_y = 0
        self.kd_y = 0

        # PID gains for roll, pitch, and yaw
        self.kp_roll = 0
        self.kd_roll = 0

        self.kp_pitch = 0
        self.kd_pitch = 0

        self.kp_yaw = 0
        self.kd_yaw = 0

        # State variables for PID controllers
        self.last_error_alt = 0.0
        self.integral_alt = 0.0

        self.last_error_x = 0.0
        self.integral_x = 0.0

        self.last_error_y = 0.0
        self.integral_y = 0.0

        self.last_error_roll = 0.0
        self.last_error_pitch = 0.0
        self.last_error_yaw = 0.0

        self.last_time = self.get_clock().now()
    
    def pose_callback(self, msg: PoseArray):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty PoseArray, ignoring...")
            return

        # Extract drone state
        current_x = msg.poses[0].position.x
        current_y = msg.poses[0].position.y
        current_z = msg.poses[0].position.z
        roll, pitch, yaw = euler_from_quaternion([msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w])

        current_roll = roll  # Assuming x-axis represents roll
        current_pitch = pitch  # Assuming y-axis represents pitch
        current_yaw = yaw  # Assuming z-axis represents yaw

        # Compute time step (dt)
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            dt = 1e-6  # Prevent division by zero

        # **Altitude Control (Z-axis)**
        error_alt = self.target_z - current_z
        self.integral_alt += error_alt * dt
        derivative_alt = (error_alt - self.last_error_alt) / dt

        total_thrust = 300.0 + (self.kp_alt * error_alt + self.ki_alt * self.integral_alt + self.kd_alt * derivative_alt)

        # **X Position PID Control**
        error_x = self.target_x - current_x
        self.integral_x += error_x * dt
        derivative_x = (error_x - self.last_error_x) / dt

        pitch_adjustment = self.kp_x * error_x + self.ki_x * self.integral_x + self.kd_x * derivative_x

        # **Y Position PID Control**
        error_y = self.target_y - current_y
        self.integral_y += error_y * dt
        derivative_y = (error_y - self.last_error_y) / dt

        roll_adjustment = self.kp_y * error_y + self.ki_y * self.integral_y + self.kd_y * derivative_y

        # **Roll & Pitch Stabilization**
        error_roll = self.target_roll - current_roll
        derivative_roll = (error_roll - self.last_error_roll) / dt
        roll_stabilization = self.kp_roll * error_roll + self.kd_roll * derivative_roll

        error_pitch = self.target_pitch - current_pitch
        derivative_pitch = (error_pitch - self.last_error_pitch) / dt
        pitch_stabilization = self.kp_pitch * error_pitch + self.kd_pitch * derivative_pitch

        # **Yaw Control (Z-axis Rotation)**
        error_yaw = self.target_yaw - current_yaw
        derivative_yaw = (error_yaw - self.last_error_yaw) / dt
        yaw_adjustment = self.kp_yaw * error_yaw + self.kd_yaw * derivative_yaw

        # **Compute motor speeds (X Configuration)**
        motor_1 = max(0.0, total_thrust - roll_adjustment + pitch_adjustment + yaw_adjustment)  # Front-left
        motor_2 = max(0.0, total_thrust + roll_adjustment + pitch_adjustment - yaw_adjustment)  # Front-right
        motor_3 = max(0.0, total_thrust + roll_adjustment - pitch_adjustment + yaw_adjustment)  # Rear-left
        motor_4 = max(0.0, total_thrust - roll_adjustment - pitch_adjustment - yaw_adjustment)  # Rear-right
        print(motor_1)
        # Publish motor speeds
        command_msg = Actuators()
        command_msg.velocity = [motor_1, motor_2, motor_3, motor_4]
        self.motor_speed_publisher.publish(command_msg)

        # Log for debugging
        self.get_logger().info(
            f"X: {current_x:.2f}, Y: {current_y:.2f}, Z: {current_z:.2f}, "
            f"Roll: {current_roll:.2f}, Pitch: {current_pitch:.2f}, Yaw: {current_yaw:.2f}, "
            f"M1: {motor_1:.2f}, M2: {motor_2:.2f}, M3: {motor_3:.2f}, M4: {motor_4:.2f}"
        )

        # Update previous errors and time
        self.last_error_alt = error_alt
        self.last_error_x = error_x
        self.last_error_y = error_y
        self.last_error_roll = error_roll
        self.last_error_pitch = error_pitch
        self.last_error_yaw = error_yaw
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = FullDroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down full drone controller...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
