import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/bebop/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'],
            output='screen',
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/parrot_bebop_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
            output='screen',
            remappings=[
                ('/model/parrot_bebop_2/odometry', '/odom')
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/parrot_bebop_2/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators'],
            output='screen'
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/bebop/camera/image@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
            remappings=[
                ('/bebop/camera/image', '/bebop/camera/image_raw')
            ]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/bebop/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen'
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/bebop/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
            output='screen'
        ),
        
    ])
