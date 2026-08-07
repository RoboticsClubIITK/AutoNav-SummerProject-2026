# Arduino Nano base interface

This package bridges an Arduino Nano to ROS 2 over USB serial. The supplied
firmware targets a Cytron SmartDriveDuo-10 (MDDS10) in PWM + direction mode.

## Safety prerequisites

- Do not connect a motor directly to the Nano.
- Use a fused 12 V battery supply for MDDS10 `B+` / `B-`.
- Join Nano GND, MDDS10 GND, battery negative, and encoder GND.
- Verify motor stall current is within the MDDS10 limit.
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

Set `encoder_ticks_per_wheel_revolution` in `config/arduino_bridge.yaml` before
publishing real odometry. The supplied value of `0` deliberately disables
`/wheel/odom` rather than publishing incorrect odometry.

## ROS interface

| Direction | ROS topic | Type |
| --- | --- | --- |
| Pi to Nano | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| Nano to Pi | `/wheel/odom` | `nav_msgs/msg/Odometry` |
| Nano to Pi | `/joint_states` | `sensor_msgs/msg/JointState` |

The bridge intentionally does not publish TF. The future EKF will own the
`odom → base_link` transform after it fuses `/wheel/odom` and `/imu/data`.
