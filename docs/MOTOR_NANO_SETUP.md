# Temporary Nano, L298N, Motor, and Encoder Setup

This guide is the temporary setup procedure for bench-testing the Arduino Nano,
an L298N dual H-bridge, two DCGM-3865 12 V motors, and their
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
| Arduino Nano | Encoder counting, command timeout, and L298N control signals |
| L298N | Dual-channel 12 V brushed DC motor H-bridge |
| Left/right DCGM-3865 | Drive motors with encoder leads `M V A B G M` |
| 12 V battery | Motor power only |

## 2. L298N power and motor wiring

| Wire | Destination |
| --- | --- |
| Battery positive through a correctly rated inline fuse | L298N `+12V` / `Vs` |
| Battery negative | L298N `GND` |
| Left motor `M` and `M` | L298N `OUT1` and `OUT2` |
| Right motor `M` and `M` | L298N `OUT3` and `OUT4` |

The exact polarity of each motor does not matter for the first wiring. If a
wheel turns backward during testing, first stop power, then either swap that
motor's two `M` wires or change the corresponding direction constant in the
firmware.

Never power the Nano from `B+`, `B-`, or a motor output. Power the Nano from
its USB cable or a separate regulated supply.

The DCGM-3865-12V-EN-240RPM is specified for 12 V, 1.0 A maximum rated current,
and 3.5 A maximum locked-rotor current. A typical L298N is not a safe
long-term driver for this current. Use only short, wheels-lifted, low-speed
tests. Replace it with a higher-current driver before sustained ground driving.

## 3. Common ground and encoder wiring

All these grounds must be connected together:

```text
Battery negative = L298N GND = Nano GND = both encoder G wires
```

Each motor cable is labelled `M V A B G M`:

| Encoder wire | Left motor Nano pin | Right motor Nano pin |
| --- | ---: | ---: |
| `V` | Nano 5 V | Nano 5 V |
| `G` | Nano GND | Nano GND |
| `A` | D2 | D3 |
| `B` | D4 | D5 |
| `M`, `M` | L298N `OUT1`, `OUT2` | L298N `OUT3`, `OUT4` |

The encoder `V` line must be compatible with 5 V. Do not connect an unknown
encoder supply directly until its datasheet or labelling confirms the voltage.
For this motor, the manufacturer specifies 3.3 V or 5 V encoder supply; Nano
5 V is appropriate.

## 4. Nano-to-L298N control wiring

Remove the L298N `ENA` and `ENB` jumpers. Connect by the printed labels:

| Nano pin | L298N input |
| ---: | --- |
| GND | GND |
| D9 | `ENA` — left PWM |
| D7 | `IN1` — left forward/reverse |
| D8 | `IN2` — left forward/reverse |
| D10 | `ENB` — right PWM |
| D11 | `IN3` — right forward/reverse |
| D12 | `IN4` — right forward/reverse |

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
until the wheels are lifted and the L298N wiring has been checked.

## 6. Calibrate encoder ticks

The firmware counts rising edges of encoder channel A. For this motor, the
manufacturer specifies 13 PPR × 42:1 gearbox = **546 ticks per complete wheel
revolution** with this decoding method. Manually turn each wheel one full
revolution and verify the measured change is near 546 before driving.

Set the measured value in
[src/arduino_base/config/arduino_bridge.yaml](../src/arduino_base/config/arduino_bridge.yaml):

```yaml
encoder_ticks_per_wheel_revolution: 546.0
```

Adjust this value only if the measured wheel-revolution count differs from 546.

## 7. Start the ROS bridge

Connect the Nano by USB to the Pi. The current CH340 Nano uses the stable path
`/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` and currently appears as
`/dev/ttyUSB1`. Check paths with:

```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

The RPLIDAR currently uses `/dev/ttyUSB0` and the CH340 Nano currently uses
`/dev/ttyUSB1`. These device numbers can change after reconnecting USB devices.
Use `/dev/serial/by-id/...` whenever available for a stable device path.

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
teleop_twist_keyboard → /cmd_vel → serial bridge → Nano → L298N → motors
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
