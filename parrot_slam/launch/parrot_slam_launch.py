from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'subscribe_depth': True,
                'subscribe_odom': True,
                'subscribe_rgbd': False,
                'subscribe_rgb': True,  
                'Reg/Force3DoF': 'False',
                'Vis/MinInliers': '20',
                'Optimizer/GravitySigma': '0.3',
                'RGBD/ProximityBySpace': 'True',
                'RGBD/LinearUpdate': '0.1',
                'Mem/RehearsalSimilarity': '0.3',
                'sync_queue_size': 30,
                'Kp/DetectorStrategy': '1',
                'Kp/MaxFeatures': '500',
                'Rtabmap/VisualOdomUsed': 'false',
                'Rtabmap/DetectionRate': '5.0',    
                'Rtabmap/PublishTf': 'False',
                # # 👇 OctoMap settings
                'Grid/FromDepth': 'true',
                'Grid/3D': 'true',
                'Grid/MaxDepth': 5.0,
                'Grid/RangeMax': 10.0,
                'Grid/CellSize': 0.05,
                'Grid/DepthDecimation': 1,
                'Grid/NoiseFilteringRadius': 0.1,
                'Grid/NoiseFilteringMinNeighbors': 5,
            }],

            remappings=[
                ('rgb/image', '/bebop/camera/image_raw'),
                ('rgb/camera_info', '/bebop/camera/camera_info'),
                ('depth/image', '/bebop/camera/depth_image'),
                ('/odom', '/odom')
            ]
        ),
       
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmapviz',
            output='screen',
            remappings=[
                ('rgb/image', '/bebop/camera/image_raw'),
                ('rgb/camera_info', '/bebop/camera/camera_info')
            ]
        )
    ])
