# Hardware and Calibration

## Verified connections

| Device | Interface | Current address / device | ROS output |
| --- | --- | --- | --- |
| RPLIDAR A1 | USB serial | stable CP2102 `/dev/serial/by-id/...`, currently `ttyUSB0` | `/scan` |
| BNO055 | I2C bus 1 | `0x28` | `/imu/data` |
| Arduino Nano (CH340) | USB serial | stable `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`, currently `ttyUSB1` | encoder protocol |

### USB serial device paths

The RPLIDAR USB adapter usually appears as `/dev/ttyUSB0`, but Linux may assign
`/dev/ttyUSB1` or another number when it is plugged into a different port or
when another USB serial device is connected first. The driver works normally as
long as `serial_port` names the active device.

Check current serial devices with:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

Start the RPLIDAR on a changed device path, for example:

```bash
ros2 launch robot_bringup robot_bringup.launch.py serial_port:=/dev/ttyUSB1
```

For a stable path that survives USB port changes, inspect:

```bash
ls -l /dev/serial/by-id/
```

Use the RPLIDAR's full `/dev/serial/by-id/...` path as `serial_port`. The Nano
usually appears separately as `/dev/ttyACM0` for the Arduino bridge.

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

### L298N dual H-bridge

The current motor driver is an L298N dual H-bridge. It controls the two brushed
DC motors through channel A (left) and channel B (right). The two 12 V
DCGM-3865 motors connect one per channel.

The DCGM-3865-12V-EN-240RPM manufacturer specifications are:

| Property | Value |
| --- | --- |
| Working voltage | 12 V |
| No-load speed | 240 rpm |
| Rated speed | 125 rpm |
| Rated current | 1.0 A maximum |
| Locked-rotor current | 3.5 A maximum |
| Encoder resolution | 13 PPR × 42:1 = 546 PPR |
| Encoder supply | 3.3 V or 5 V |

The motor locked-rotor current is 3.5 A maximum. A typical L298N is not a good
long-term match for that current because of its high voltage drop and limited
thermal capacity. Use it only for short, lifted-wheel, low-speed checks. Use a
higher-current H-bridge for reliable ground driving.

| Connection | Connect to |
| --- | --- |
| Battery positive through an appropriately rated fuse | L298N `+12V` / `Vs` |
| Battery negative | L298N `GND` |
| Left motor `M` wires | L298N `OUT1`, `OUT2` |
| Right motor `M` wires | L298N `OUT3`, `OUT4` |
| Nano GND | L298N `GND` |
| Nano D9 | L298N `ENA` (remove ENA jumper) |
| Nano D7 | L298N `IN1` |
| Nano D8 | L298N `IN2` |
| Nano D10 | L298N `ENB` (remove ENB jumper) |
| Nano D11 | L298N `IN3` |
| Nano D12 | L298N `IN4` |

Remove the L298N `ENA` and `ENB` jumpers so the Nano controls speed with PWM.
The Nano must be powered by USB or a separate regulated supply; never power it
from a motor output. Start with wheels lifted, an inline battery fuse, and the
low command speed configured in the firmware.

### Encoder wiring

Each motor connector is labelled `M V A B G M`:

| Motor connector label | Connection |
| --- | --- |
| `M` / `M` | Corresponding L298N motor output |
| `V` | Nano regulated 5 V |
| `G` | Nano GND |
| Left `A` / `B` | Nano D2 / D4 |
| Right `A` / `B` | Nano D3 / D5 |

All grounds must be common: battery negative, L298N ground, Nano GND,
and both encoder grounds.

For the PH2.0 six-pin connector, the manufacturer pin order is `M2`, `V`, `B`,
`A`, `G`, `M1`. Connect by label, not connector position.

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
