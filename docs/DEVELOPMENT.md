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

## Launch full real-sensor mapping

After the Nano firmware is uploaded, the L298N has passed the lifted-wheel
safety test, and encoder scale is verified, start:

```bash
ros2 launch robot_bringup mapping.launch.py
```

This launch enables the Arduino bridge and EKF, disables the temporary wheel
joint publisher, and removes the temporary static odometry transform. The EKF
fuses `/wheel/odom` and `/imu/data`, then publishes the dynamic `odom →
base_footprint` TF used by SLAM Toolbox. Add `rviz:=false` when rendering on a
remote laptop instead.

Useful launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `serial_port` | Stable RPLIDAR `/dev/serial/by-id/...` path | RPLIDAR serial device |
| `serial_baudrate` | `115200` | RPLIDAR baud rate |
| `use_imu` | `true` | Start the BNO055 node |
| `rviz` | `false` | Open configured RViz |
| `publish_joint_states` | `true` | Publish zero wheel positions until real encoder joint states exist |

If the RPLIDAR device number changes, override `serial_port`. Prefer its stable
`/dev/serial/by-id/...` symlink when available; see
[Hardware and calibration](HARDWARE.md#usb-serial-device-paths).

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

RViz can run on a laptop while the Pi is physically mounted on the robot. The
laptop subscribes to the Pi's ROS 2 topics over Wi-Fi; no SSH desktop or display
connected to the Pi is needed. This moves RViz rendering to the laptop, but does
not move Gazebo simulation CPU/GPU work if Gazebo is running on the Pi.

Connect both devices to the same Wi-Fi router or the same hotspot. The network
must allow device-to-device traffic; disable Wi-Fi client isolation / AP
isolation if it is enabled.

### Pi terminal

Set the ROS discovery settings and start hardware without RViz:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lidar_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 launch robot_bringup robot_bringup.launch.py rviz:=false
```

### Laptop terminal

Install ROS 2 Jazzy and RViz on the laptop. Copy or clone this workspace, build
the `robot_bringup` package, and use the identical discovery settings:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lidar_ws/install/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 launch robot_bringup rviz.launch.py
```

If the workspace is not present on the laptop, copy
`src/robot_bringup/config/lidar_slam.rviz` there and run `rviz2 -d` with that
file instead. The `RobotModel` display still receives `/robot_description` from
the Pi.

### Verify ROS discovery

On the laptop, these commands should show the Pi's nodes and data:

```bash
ros2 node list
ros2 topic list
ros2 topic echo --once /scan
ros2 topic echo --once /imu/data
```

If no topics appear, confirm both machines use exactly the same
`ROS_DOMAIN_ID`, are on the same IP subnet, have `ROS_LOCALHOST_ONLY=0`, and
are not using a VPN. Test with a phone hotspot or simple home router before
changing ROS configuration further.
