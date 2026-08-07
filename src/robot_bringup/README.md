# Live RPLIDAR SLAM test

This package tests the real RPLIDAR with `slam_toolbox` and starts the physical
BNO055 driver. It does not start Gazebo, the Arduino interface, motor control,
or the EKF by default. Enable the Arduino bridge only after firmware upload and
encoder calibration.

## What starts

`robot_bringup.launch.py` starts:

1. `robot_state_publisher`, which publishes the fixed robot and `laser` TF
   frames from the URDF.
2. `joint_state_publisher`, which sends zero wheel positions so RViz can show
   the complete model before encoder feedback exists.
3. `rplidar_ros`, which publishes `sensor_msgs/msg/LaserScan` on `/scan`.
4. `bno055_ros`, which publishes `sensor_msgs/msg/Imu` on `/imu/data`.
5. A **temporary fixed** `odom` to `base_footprint` transform.
6. `slam_toolbox`, which consumes `/scan` and creates `/map`.
7. Optionally, `arduino_base`, which bridges Nano encoder data after it is
   calibrated and enabled with `use_arduino:=true`.

The fixed odometry transform is only for this LiDAR-only compatibility test.
It allows SLAM Toolbox to receive the complete `map -> odom -> base_link ->
laser` transform chain. It is not real robot odometry. Mapping while carrying
the robot slowly can demonstrate scan matching, but it will not be accurate
enough for autonomous navigation until encoders and the IMU are fused by the
EKF.

## Start the LiDAR-only test on the Pi

The connected CP2102 USB adapter is exposed as `/dev/ttyUSB0`. The current robot
model is RPLIDAR A1, which uses 115200 baud.

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arduino_base bno055_ros my_robot_description robot_bringup
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py
```

The launch does not open RViz by default, which keeps the Pi light. Confirm the
live laser scan locally before starting SLAM debugging:

```bash
ros2 topic echo --once /scan
ros2 run tf2_ros tf2_echo base_link laser
```

If the LiDAR serial device changes, override the port without editing code:

```bash
ros2 launch robot_bringup robot_bringup.launch.py serial_port:=/dev/ttyUSB0
```

Use `ls -l /dev/ttyUSB* /dev/ttyACM*` to find the assigned device. Prefer the
RPLIDAR's stable `/dev/serial/by-id/...` symlink when available, because a
`ttyUSB` number can change after moving USB ports or connecting another serial
device.

## View from the laptop with RViz

Connect the Pi and laptop to the same LAN. On both devices use the same ROS 2
network values:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
```

On the laptop, first verify it can see the Pi topics:

```bash
ros2 topic list
ros2 topic echo --once /scan
```

Then start RViz:

```bash
ros2 launch robot_bringup rviz.launch.py
```

The supplied configuration opens with fixed frame `map` and shows `/map`,
`/scan`, the TF tree, grid, and robot model from `/robot_description`. To open
this configured RViz on the Pi with the LiDAR bringup, add `rviz:=true` to the
main launch command.

Move the robot slowly around a room with visible walls and corners. If `/scan`
appears but `/map` does not, use the commands above to confirm the `base_link`
to `laser` transform and check the `slam_toolbox` terminal output.

## Next hardware phase

Do not change the SLAM topic names later. The Arduino bridge subscribes to
`/cmd_vel` and publishes encoder odometry to `/wheel/odom`. Then configure
`robot_localization` to fuse `/wheel/odom` and the already-running `/imu/data`,
replacing the temporary fixed odometry transform with the EKF `odom` to
`base_link` transform.

When the Arduino publishes real `sensor_msgs/msg/JointState` feedback, start
the launch with `publish_joint_states:=false` to disable the temporary zero
joint-state publisher.

After uploading the MDDS10 Nano firmware and setting the encoder tick scale,
enable the serial bridge with `use_arduino:=true`.

Before enabling motors, implement a command timeout / emergency stop in the
Arduino firmware and verify encoder directions at low speed.
