# Hardware and Calibration

## Verified connections

| Device | Interface | Current address / device | ROS output |
| --- | --- | --- | --- |
| RPLIDAR A1 | USB serial | `/dev/ttyUSB0`, 115200 baud | `/scan` |
| BNO055 | I2C bus 1 | `0x28` | `/imu/data` |

The BNO055 driver uses combined I2C register transactions with retries. This is
important for reliable communication with the connected module.

## BNO055 checks

Before enabling the full bringup, confirm that the IMU is visible at `0x28` and
that `/imu/data` contains non-zero orientation and acceleration data. If the
BNO055 drops off I2C, power-cycle it and check 3.3 V, GND, SDA, and SCL wiring.

The physical orientation of the board must match the URDF `imu_link` transform.
Confirm that turning the robot left produces a positive ROS yaw after the final
axis convention is configured. Calibrate the BNO055 before tuning the EKF.

## Encoder and motor requirements

The Arduino Nano must:

1. Read both wheel encoders and maintain signed tick counts.
2. Apply a command timeout that stops motors when commands stop arriving.
3. Send timestamped encoder data to the Pi.
4. Receive velocity commands from the Pi.

The Pi-side bridge will publish `/wheel/odom` and subscribe to `/cmd_vel`.

Before integrating the bridge, measure and record:

- Encoder ticks per wheel revolution
- Wheel radius
- Wheel separation
- Left/right encoder sign
- Maximum safe linear and angular velocity

Do not connect motor power for autonomous movement until the firmware timeout
and emergency-stop behavior have been tested at low speed.
