# Autonomous Robot ROS 2 Workspace

ROS 2 Jazzy workspace for a small differential-drive autonomous robot running on
a Raspberry Pi. The currently validated hardware is an RPLIDAR A1 and a BNO055
connected directly to the Pi. Motor encoders and motor control will be added
through an Arduino Nano.

## Current capability

The current bringup launches the live RPLIDAR, BNO055, robot model, RViz-ready
TF tree, and SLAM Toolbox mapping.

```text
RPLIDAR  ── /scan ───────────────┐
                                  ├── SLAM Toolbox ── /map, map → odom
TF: odom → base_link → laser ────┘

BNO055 ── /imu/data ── ready for future EKF fusion
```

The present `odom → base_footprint` transform is intentionally fixed for the
LiDAR-only mapping test. It must be replaced by EKF output once encoder
odometry exists.

## Quick start

```bash
cd ~/lidar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch robot_bringup robot_bringup.launch.py rviz:=true
```

The RPLIDAR is expected at `/dev/ttyUSB0`; override it when necessary:

```bash
ros2 launch robot_bringup robot_bringup.launch.py serial_port:=/dev/ttyUSB0
```

## Packages

| Package | Responsibility |
| --- | --- |
| `bno055_ros` | BNO055 I2C driver; publishes `/imu/data`. |
| `my_robot_description` | URDF robot geometry and fixed sensor frames. |
| `rplidar_ros` | Vendor RPLIDAR ROS 2 driver; publishes `/scan`. |
| `robot_bringup` | Launches current hardware, TF, RViz, and SLAM Toolbox. |
| `arduino_base` | Nano firmware and future USB serial bridge for MDDS10, encoders, and `/cmd_vel`. |
| `nav2_ros` | Placeholder for the future Nav2 integration. |

## Documentation

- [Architecture](docs/ARCHITECTURE.md)
- [Hardware and calibration](docs/HARDWARE.md)
- [Temporary Nano and motor-driver setup](docs/MOTOR_NANO_SETUP.md)
- [Development and validation](docs/DEVELOPMENT.md)
- [Agent conventions](AGENTS.md)
- [Bringup-specific guide](src/robot_bringup/README.md)

## Next milestone

Implement Arduino Nano encoder firmware and a Pi serial bridge that publishes
`/wheel/odom`. Then use `robot_localization` to fuse `/wheel/odom` and
`/imu/data`, producing the dynamic `odom → base_link` transform required for
reliable SLAM and navigation.
