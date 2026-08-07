# Temporary Nano, MDDS10, Motor, and Encoder Setup

This guide is the temporary setup procedure for bench-testing the Arduino Nano,
Cytron SmartDriveDuo-10 (MDDS10), two DCGM-3865 12 V motors, and their
encoder feedback. It is intentionally separate from EKF and autonomous
navigation setup.

> **Safety:** Do not connect motors directly to the Nano. Start with wheels off
the floor, a fused battery, and a clear physical emergency disconnect. The Nano
firmware stops motors after 250 ms without a command, but that is not a
replacement for a battery disconnect.

## 1. Parts and current roles

| Part | Role |
| --- | --- |
| Raspberry Pi | ROS 2, LiDAR, BNO055, SLAM, and USB serial bridge |
| Arduino Nano | Encoder counting, command timeout, and MDDS10 control signals |
| Cytron MDDS10 | Two-channel 12 V brushed DC motor H-bridge |
| Left/right DCGM-3865 | Drive motors with encoder leads `M V A B G M` |
| 12 V battery | Motor power only |

## 2. MDDS10 power and motor wiring

| Wire | Destination |
| --- | --- |
| Battery positive through a correctly rated inline fuse | MDDS10 `B+` |
| Battery negative | MDDS10 `B-` / GND |
| Left motor `M` and `M` | MDDS10 `M1A` and `M1B` |
| Right motor `M` and `M` | MDDS10 `M2A` and `M2B` |

The exact polarity of each motor does not matter for the first wiring. If a
wheel turns backward during testing, first stop power, then either swap that
motor's two `M` wires or change the corresponding direction constant in the
firmware.

Never power the Nano from `B+`, `B-`, or a motor output. Power the Nano from
its USB cable or a separate regulated supply.

## 3. Common ground and encoder wiring

All these grounds must be connected together:

```text
Battery negative = MDDS10 signal GND = Nano GND = both encoder G wires
```

Each motor cable is labelled `M V A B G M`:

| Encoder wire | Left motor Nano pin | Right motor Nano pin |
| --- | ---: | ---: |
| `V` | Nano 5 V | Nano 5 V |
| `G` | Nano GND | Nano GND |
| `A` | D2 | D3 |
| `B` | D4 | D5 |
| `M`, `M` | MDDS10 `M1A`, `M1B` | MDDS10 `M2A`, `M2B` |

The encoder `V` line must be compatible with 5 V. Do not connect an unknown
encoder supply directly until its datasheet or labelling confirms the voltage.

## 4. Nano-to-MDDS10 control wiring

Set the MDDS10 to **independent PWM + direction MCU control** using the
DIP-switch table printed on the rear of the MDDS10. Connect by the printed
header names:

| Nano pin | MDDS10 input |
| ---: | --- |
| GND | GND |
| D9 | `PWM1` — left motor channel |
| D7 | `DIR1` — left motor channel |
| D10 | `PWM2` — right motor channel |
| D8 | `DIR2` — right motor channel |

The firmware pin definitions are in
[src/arduino_base/firmware/robot_base.ino](../src/arduino_base/firmware/robot_base.ino).

## 5. Upload Nano firmware

1. Open [robot_base.ino](../src/arduino_base/firmware/robot_base.ino) in Arduino IDE.
2. Select **Arduino Nano** and its USB serial port.
3. Select the correct processor/bootloader for the Nano clone if required.
4. Upload while motor battery power is disconnected.
5. Open Serial Monitor at **115200 baud**. The firmware sends messages like:

   ```text
   ENC 1250 0 0
   ```

The Nano receives motor commands only in this form:

```text
CMD <linear_mps> <angular_radps>
```

For example, `CMD 0.05 0.00` requests a slow forward movement. Do not send this
until the wheels are lifted and the MDDS10 wiring has been checked.

## 6. Calibrate encoder ticks

The firmware counts rising edges of encoder channel A. Measure the tick change
for **one complete wheel revolution** for each wheel. The left and right counts
should be close; investigate a large mismatch before driving.

Set the measured value in
[src/arduino_base/config/arduino_bridge.yaml](../src/arduino_base/config/arduino_bridge.yaml):

```yaml
encoder_ticks_per_wheel_revolution: REPLACE_WITH_MEASURED_VALUE
```

The default is `0`, which intentionally prevents `/wheel/odom` from being
published. This prevents incorrect navigation data before calibration.

## 7. Start the ROS bridge

Connect the Nano by USB to the Pi. Find its port, normally `/dev/ttyACM0`:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

The RPLIDAR normally uses `/dev/ttyUSB*` while the Nano normally uses
`/dev/ttyACM*`. These device numbers can change after reconnecting USB devices.
Use `/dev/serial/by-id/...` when available for a stable device path.

Then run the bridge by itself:

```bash
source /opt/ros/jazzy/setup.bash
source ~/lidar_ws/install/setup.bash
ros2 launch arduino_base arduino_bridge.launch.py
```

After the encoder scale is set, verify:

```bash
ros2 topic echo /joint_states
ros2 topic echo /wheel/odom
```

The bridge publishes `/wheel/odom` and `/joint_states`; it deliberately does
**not** publish an `odom → base_link` transform. The future EKF owns that TF.

## 8. Temporary teleoperation bench test

After verifying tick direction and with wheels raised:

```bash
ros2 launch robot_bringup robot_bringup.launch.py \
  use_arduino:=true \
  publish_joint_states:=false
```

From another ROS 2 terminal or a laptop on the same ROS domain:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The command flow is:

```text
teleop_twist_keyboard → /cmd_vel → serial bridge → Nano → MDDS10 → motors
```

Expected checks:

1. Forward command turns both wheels forward.
2. Left turn turns left wheel slower or backward and right wheel forward.
3. Forward wheel rotation increases both tick counts in the configured positive
direction.
4. Releasing commands stops motors within 250 ms.
5. Pulling the Nano USB cable or stopping the bridge also stops motors by the
firmware timeout.

If wheel or tick directions disagree, correct the firmware direction constants
or encoder signs before using odometry.

## 9. What is still temporary

The current `robot_bringup` launch contains a static test transform:

```text
odom → base_footprint
```

It is suitable only for LiDAR/SLAM visualization. Do **not** use it for moving
autonomous navigation. The next integration step is:

```text
/wheel/odom + /imu/data → robot_localization EKF → dynamic odom → base_link
```

After the EKF works, disable the temporary zero joint publisher and remove the
static test odometry transform. SLAM Toolbox and Nav2 can then use reliable
motion estimation.
