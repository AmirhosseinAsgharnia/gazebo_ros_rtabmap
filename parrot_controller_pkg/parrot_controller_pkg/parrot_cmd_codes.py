import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
class ClosedLoopZigZag(Node):
    def __init__(self):
        super().__init__('bebop_cmd')
        self.cmd_pub = self.create_publisher(Twist, '/bebop/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.current_pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.waypoints = [
            (0.0, 0.0, 0.5),
            (0.0, 2.5, 0.5),
            (-1.5, 2.5, 0.5),
            (-1.5, -2.5, 0.5),
            (1.5, -2.5, 0.5),
            (1.5, 2.5, 0.5),
            (0.0, 2.5, 0.5),
            (0.0, 0.0, 0.5),
            (0.0, 0.0, 0.0)
        ]
        self.waypoint_index = 0
        self.tolerance = 0.1
        time.sleep(10)

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def distance_and_yaw_to_target(self, target):
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        dz = target[2] - self.current_pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)

        yaw_target = math.atan2(dy, dx)
        q = self.current_pose.orientation
        yaw_current = self.get_yaw_from_quaternion(q)

        yaw_error = self.normalize_angle(yaw_target - yaw_current)
        return distance, dz, yaw_error

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def timer_callback(self):
        if self.current_pose is None or self.waypoint_index >= len(self.waypoints):
            return

        twist = Twist()
        target = self.waypoints[self.waypoint_index]
        dist, dz, yaw_err = self.distance_and_yaw_to_target(target)

        if abs(yaw_err) > 0.1:
            twist.angular.z = 1.5 * yaw_err
        else:
            if abs(dz) > self.tolerance:
                twist.linear.z = 0.5 * dz
            else:
                dx = target[0] - self.current_pose.position.x
                dy = target[1] - self.current_pose.position.y
                planar_dist = math.sqrt(dx**2 + dy**2)
                if planar_dist > self.tolerance:
                    twist.linear.x = 0.5 * planar_dist
                else:
                    self.get_logger().info(f"Reached waypoint {self.waypoint_index}: {target}")
                    self.waypoint_index += 1
                    return  # Wait until next timer tick before commanding next step

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ClosedLoopZigZag()
    rclpy.spin(node)
    rclpy.shutdown()
