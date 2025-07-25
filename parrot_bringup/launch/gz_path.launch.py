import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # Path to the models directory
    models_path = os.path.join(
        os.path.expanduser('~/ros_arise_drone_ws/src/arise_parrot_pkg/world')
    )

    return LaunchDescription([
        # Set GZ_SIM_RESOURCE_PATH environment variable
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'$GZ_SIM_RESOURCE_PATH:{models_path}'
        ),
    ])