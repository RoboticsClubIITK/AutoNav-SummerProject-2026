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

## Arduino, motor, and encoder requirements

### Cytron SmartDriveDuo-10 (MDDS10)

The selected motor driver is a Cytron SmartDriveDuo-10 (MDDS10), not an L298N.
It controls two brushed DC motors independently and accepts a 7–35 V motor
supply. The two 12 V DCGM-3865 motors connect one per channel.

| Connection | Connect to |
| --- | --- |
| Battery positive through an appropriately rated fuse | MDDS10 `B+` |
| Battery negative | MDDS10 `B-` / GND |
| Left motor `M` wires | MDDS10 `M1A`, `M1B` |
| Right motor `M` wires | MDDS10 `M2A`, `M2B` |
| Nano GND | MDDS10 signal GND |
| Nano D9 | MDDS10 `PWM1` |
| Nano D7 | MDDS10 `DIR1` |
| Nano D10 | MDDS10 `PWM2` |
| Nano D8 | MDDS10 `DIR2` |

Configure the MDDS10 for **independent PWM + direction MCU control** according
to the switch table printed on the back of the board. Connect by the printed
input labels, not by connector position. The Nano must be powered by USB or a
separate regulated supply; never power it from the MDDS10 motor terminals.

Verify each DCGM-3865 motor's stall current is within the MDDS10 rating before
connecting motor power. Start with wheels lifted, an inline battery fuse, and
a low command speed.

### Encoder wiring

Each motor connector is labelled `M V A B G M`:

| Motor connector label | Connection |
| --- | --- |
| `M` / `M` | Corresponding MDDS10 motor channel output |
| `V` | Nano regulated 5 V |
| `G` | Nano GND |
| Left `A` / `B` | Nano D2 / D4 |
| Right `A` / `B` | Nano D3 / D5 |

All grounds must be common: battery negative, MDDS10 signal ground, Nano GND,
and both encoder grounds.

The Arduino Nano must:

1. Read both wheel encoders and maintain signed tick counts.
2. Apply a command timeout that stops motors when commands stop arriving.
3. Send timestamped encoder data to the Pi.
4. Receive velocity commands from the Pi.

The Pi-side bridge will publish `/wheel/odom` and subscribe to `/cmd_vel`.

The implementation is in [arduino_base](../src/arduino_base/README.md). Upload
`firmware/robot_base.ino` to the Nano, then set the measured encoder scale in
`config/arduino_bridge.yaml` before launching the bridge.

Follow [the temporary Nano and motor-driver setup guide](MOTOR_NANO_SETUP.md)
for the safe wiring, firmware upload, tick-calibration, and teleoperation
sequence.

Before integrating the bridge, measure and record:

- Encoder ticks per wheel revolution
- Wheel radius
- Wheel separation
- Left/right encoder sign
- Maximum safe linear and angular velocity

Do not connect motor power for autonomous movement until the firmware timeout
and emergency-stop behavior have been tested at low speed.
