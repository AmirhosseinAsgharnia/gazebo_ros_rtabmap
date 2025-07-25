import rclpy
from rclpy.node import Node
import subprocess
import os

class GazeboLauncher(Node):
    def __init__(self):
        super().__init__('gazebo_launcher')
        self.get_logger().info("Launching Gazebo with world...")

        # Get the world file path
        world_file_path = os.path.join(
            os.path.expanduser('~/ros_arise_drone_ws/src/arise_parrot_pkg/world/'),
            'arise.world'
        )

        # Launch Gazebo
        subprocess.Popen(['gz', 'sim', world_file_path ])

def main(args=None):
    rclpy.init(args=args)
    node = GazeboLauncher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()