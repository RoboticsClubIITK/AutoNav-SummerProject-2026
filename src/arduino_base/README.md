# Arduino Nano base interface

This package bridges an Arduino Nano to ROS 2 over USB serial. The supplied
firmware targets an L298N in ENA/IN1/IN2 and ENB/IN3/IN4 mode.

## Safety prerequisites

- Do not connect a motor directly to the Nano.
- Use a fused 12 V battery supply for L298N `+12V` / GND.
- Join Nano GND, L298N GND, battery negative, and encoder GND.
- The L298N is a short low-speed bench-test driver only for these motors; it
  is not recommended for sustained ground driving at their 3.5 A stall current.
- Keep motor power disconnected until encoder direction and command timeout
  behavior have been tested.

## Serial protocol

Pi to Nano at 115200 baud:

```text
CMD <linear_mps> <angular_radps>\n
```

Nano to Pi at 50 Hz:

```text
ENC <millis> <left_ticks> <right_ticks>\n
```

The Nano stops both motors if no valid `CMD` arrives within 250 ms.

## Required calibration

The DCGM-3865-12V-EN-240RPM manufacturer specification is 13 PPR × 42:1 = 546
PPR. The supplied configuration uses `546.0`, matching the firmware's
rising-edge A-channel decoder. Verify this by manually turning a wheel exactly
one full revolution before trusting odometry.

## ROS interface

| Direction | ROS topic | Type |
| --- | --- | --- |
| Pi to Nano | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Nano to Pi | `/wheel/odom` | `nav_msgs/msg/Odometry` |
| Nano to Pi | `/joint_states` | `sensor_msgs/msg/JointState` |

The bridge intentionally does not publish TF. The future EKF will own the
`odom → base_link` transform after it fuses `/wheel/odom` and `/imu/data`.
