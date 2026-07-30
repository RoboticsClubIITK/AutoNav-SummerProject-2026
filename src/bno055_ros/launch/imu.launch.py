from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bno055_ros',
            executable='imu_node',
            name='bno055_imu_node',
            output='screen',
            parameters=[{'frame_id': 'imu_link'}],
        )
    ])
