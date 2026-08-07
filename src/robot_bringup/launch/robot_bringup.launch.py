"""Launch the live RPLIDAR-only SLAM compatibility test.

This starts the physical RPLIDAR and BNO055. The optional Arduino bridge is
disabled by default; EKF and motor nodes are not started. The fixed odom-to-base
transform only supplies the TF chain required for a
stationary LiDAR test and scan-matching demonstration; it is not odometry.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    bringup_share = get_package_share_directory('robot_bringup')
    description_share = get_package_share_directory('my_robot_description')
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    use_rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    publish_joint_states = LaunchConfiguration('publish_joint_states')
    use_imu = LaunchConfiguration('use_imu')
    use_arduino = LaunchConfiguration('use_arduino')
    arduino_config = LaunchConfiguration('arduino_config')

    with open(os.path.join(description_share, 'urdf', 'my_robot.urdf'), encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'publish_robot_description': True,
        }],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': robot_description}],
        condition=IfCondition(publish_joint_states),
        output='screen',
    )

    test_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='test_odom_to_base_footprint_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'odom', '--child-frame-id', 'base_footprint',
        ],
        output='screen',
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen',
    )

    imu_node = Node(
        package='bno055_ros',
        executable='imu_node',
        name='bno055_imu_node',
        parameters=[{'frame_id': 'imu_link'}],
        condition=IfCondition(use_imu),
        output='screen',
    )

    arduino_bridge = Node(
        package='arduino_base',
        executable='serial_bridge',
        name='serial_bridge',
        parameters=[arduino_config],
        condition=IfCondition(use_arduino),
        output='screen',
    )

    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'slam_toolbox.yaml')],
    )

    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            )],
        )
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_imu',
            default_value='true',
            description='Start the physical BNO055 driver and publish /imu/data.',
        ),
        DeclareLaunchArgument(
            'use_arduino',
            default_value='false',
            description='Start the Arduino serial bridge after firmware and encoder calibration are ready.',
        ),
        DeclareLaunchArgument(
            'arduino_config',
            default_value=os.path.join(
                get_package_share_directory('arduino_base'),
                'config',
                'arduino_bridge.yaml'),
            description='Arduino bridge parameter file.',
        ),
        DeclareLaunchArgument(
            'publish_joint_states',
            default_value='true',
            description='Publish zero wheel joint states until encoder feedback is available.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(bringup_share, 'config', 'lidar_slam.rviz'),
        ),
        robot_state_publisher,
        joint_state_publisher,
        test_odom_tf,
        rplidar_node,
        imu_node,
        arduino_bridge,
        slam_toolbox_node,
        configure_slam,
        activate_slam,
        rviz,
    ])