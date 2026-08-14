"""Start the complete real-sensor mapping stack.

Requires the Nano firmware to be uploaded, the encoder scale to be calibrated,
and the L298N/motor wiring to have passed the lifted-wheel safety test.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    use_rviz = LaunchConfiguration('rviz')

    base_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'robot_bringup.launch.py')
        ),
        launch_arguments={
            'rviz': use_rviz,
            'use_arduino': 'true',
            'use_ekf': 'true',
            'publish_joint_states': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        base_bringup,
    ])