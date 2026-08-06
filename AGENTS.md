# Workspace Guide for Contributors and Agents

## Scope

This is a ROS 2 Jazzy workspace for a differential-drive autonomous robot. Read
[README.md](README.md) and [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) before
changing runtime behavior.

## Package responsibilities

- `bno055_ros`: physical I2C BNO055 driver. Preserve frame ID `imu_link` and
  topic `/imu/data` unless all consumers are updated.
- `my_robot_description`: the source of truth for physical geometry and fixed
  TF frames. Update sensor mounting transforms when hardware positions are
  measured.
- `rplidar_ros`: vendored upstream driver. Avoid unrelated edits to this
  package.
- `robot_bringup`: integration layer for launch files, configuration, RViz, and
  documentation.
- `nav2_ros`: reserved for the future navigation integration.

## Topic and TF conventions

Keep these stable:

```text
/scan        sensor_msgs/msg/LaserScan, frame laser
/imu/data    sensor_msgs/msg/Imu, frame imu_link
/wheel/odom  nav_msgs/msg/Odometry from the future Arduino bridge
/cmd_vel     geometry_msgs/msg/Twist to the future Arduino bridge
```

The final TF ownership is:

```text
robot_state_publisher: fixed robot frames
robot_localization:    odom → base_link
slam_toolbox:          map → odom
```

Do not introduce duplicate publishers for those dynamic transforms.

## Current temporary behavior

`robot_bringup` currently publishes a static `odom → base_footprint` transform
for LiDAR-only SLAM testing and zero wheel joint states for RViz. Remove or
disable both when real encoder odometry and joint states are integrated.

## Build and validation

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select bno055_ros my_robot_description robot_bringup
source install/setup.bash
```

Validate real hardware only when it is connected. Stop temporary sensor launch
processes after tests so they do not hold `/dev/ttyUSB0` or `/dev/i2c-1`.

## Repository hygiene

- Keep build outputs out of version control.
- Do not commit local shell, editor, or assistant artifacts.
- Add documentation whenever topics, frames, hardware assumptions, or launch
  behavior change.
- Do not modify vendored RPLIDAR sources unless the change is required and
  documented.
