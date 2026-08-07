import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('arduino_base')
    config = LaunchConfiguration('config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=os.path.join(package_share, 'config', 'arduino_bridge.yaml'),
            description='Arduino bridge parameter file.',
        ),
        Node(
            package='arduino_base',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[config],
            output='screen',
        ),
    ])
