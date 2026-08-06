# System Architecture

## Current runtime graph

```text
RPLIDAR A1
  └─ rplidar_ros ── /scan (sensor_msgs/msg/LaserScan)
                                      │
Robot description ── /tf_static ─────┼── SLAM Toolbox
Temporary odom TF ── /tf ────────────┘       ├─ /map
                                              └─ map → odom

BNO055
  └─ bno055_ros ── /imu/data (sensor_msgs/msg/Imu)
```

`slam_toolbox` directly consumes `/scan` and the `odom → base_link → laser` TF
relationship. It does **not** subscribe to `/imu/data` or wheel odometry
directly.

## Frame tree

```text
map → odom → base_footprint → base_link → laser
                                      ├─ imu_link
                                      ├─ left_wheel
                                      └─ right_wheel
```

| Frame | Meaning | Current publisher |
| --- | --- | --- |
| `map` | SLAM-corrected global map frame | SLAM Toolbox |
| `odom` | Local continuous odometry frame | Static test TF; future EKF |
| `base_footprint` | Ground projection of the robot | URDF / test TF |
| `base_link` | Robot body reference frame | URDF |
| `laser` | RPLIDAR measurement frame | URDF + RPLIDAR message header |
| `imu_link` | BNO055 mounting frame | URDF + BNO055 message header |

## Required topic contracts

| Producer | Topic | Type | Consumer |
| --- | --- | --- | --- |
| RPLIDAR | `/scan` | `sensor_msgs/msg/LaserScan` | SLAM Toolbox |
| BNO055 | `/imu/data` | `sensor_msgs/msg/Imu` | Future EKF |
| Arduino bridge | `/wheel/odom` | `nav_msgs/msg/Odometry` | Future EKF |
| EKF | `/odometry/filtered` | `nav_msgs/msg/Odometry` | Monitoring / Nav2 |
| EKF | `odom → base_link` | TF | SLAM Toolbox / Nav2 |
| SLAM Toolbox | `/map` | `nav_msgs/msg/OccupancyGrid` | RViz / Nav2 |
| SLAM Toolbox | `map → odom` | TF | RViz / Nav2 |
| Nav2 | `/cmd_vel` | `geometry_msgs/msg/Twist` | Future Arduino bridge |

## Future motion-estimation graph

```text
/wheel/odom ─┐
             ├─ robot_localization EKF ── /odometry/filtered
/imu/data ───┘                              └─ odom → base_link

/scan + odom → base_link ── SLAM Toolbox ── /map, map → odom
```

The future encoder bridge must not publish a second `odom → base_link` TF if
the EKF publishes it. Configure the encoder odometry publisher with TF disabled
and let `robot_localization` own that transform.
