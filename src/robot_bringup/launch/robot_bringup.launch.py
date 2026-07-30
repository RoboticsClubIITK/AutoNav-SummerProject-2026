import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, EmitEvent, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import matches_action
from launch_ros.actions import Node, LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import Transition
from launch.actions import TimerAction

def generate_launch_description():

    # --- Static transform: base_link -> imu_link ---
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link',
        ],
    )

    # --- Static transform: base_link -> laser ---
    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    # --- IMU node ---
    imu_node = Node(
        package='bno055_ros',
        executable='imu_node',
        name='bno055_imu_node',
        output='screen',
        parameters=[{'frame_id': 'imu_link'}],
    )

    # --- EKF (robot_localization) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'config',
                'ekf.yaml'
            )
        ],
    )

    # --- SLAM Toolbox (lifecycle node) ---
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('robot_bringup'),
                'config',
                'slam_toolbox.yaml'
            )
        ]
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    # --- RPLIDAR node (invoked directly, not via included launch file,
    # so it can be reliably wrapped in a TimerAction) ---
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/rplidar',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        output='screen'
    )

    # Delay RPLIDAR startup by 15s so it doesn't compete for USB/CPU
    # resources with the other nodes launching simultaneously.
    delayed_rplidar_launch = TimerAction(
        period=15.0,
        actions=[rplidar_node]
    )

    return LaunchDescription([
        imu_static_tf,
        laser_static_tf,
        imu_node,
        delayed_rplidar_launch,
        ekf_node,
        slam_toolbox_node,
        configure_event,
        activate_event,
    ])