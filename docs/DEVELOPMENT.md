# Development and Validation

## Build

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build only the active packages during development:

```bash
colcon build --packages-select bno055_ros my_robot_description robot_bringup
```

To build the Arduino bridge as well:

```bash
colcon build --packages-select arduino_base
```

## Launch current hardware

```bash
ros2 launch robot_bringup robot_bringup.launch.py rviz:=true
```

Useful launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `serial_port` | `/dev/ttyUSB0` | RPLIDAR serial device |
| `serial_baudrate` | `115200` | RPLIDAR baud rate |
| `use_imu` | `true` | Start the BNO055 node |
| `rviz` | `false` | Open configured RViz |
| `publish_joint_states` | `true` | Publish zero wheel positions until real encoder joint states exist |

## Validation checklist

```bash
ros2 topic echo --once /scan
ros2 topic echo --once /imu/data
ros2 topic info /map
ros2 run tf2_ros tf2_echo odom laser
ros2 run tf2_ros tf2_echo base_link imu_link
```

Expected current results:

- `/scan` has frame ID `laser`.
- `/imu/data` has frame ID `imu_link`.
- SLAM Toolbox publishes `/map` and `map → odom`.
- `odom → base_footprint` is temporary and static.

## Remote RViz

Run the hardware bringup on the Pi without RViz. On a laptop on the same LAN,
source the same ROS distribution and workspace, use the same `ROS_DOMAIN_ID`,
and run:

```bash
ros2 launch robot_bringup rviz.launch.py
```

Set `ROS_LOCALHOST_ONLY=0` on both machines. Wi-Fi client isolation must be
disabled for ROS 2 discovery to work.
