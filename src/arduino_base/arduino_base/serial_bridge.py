"""ROS 2 bridge for the Arduino Nano differential-drive firmware."""

import math
import time
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import serial
    from serial import SerialException
except ImportError as error:  # pragma: no cover - reported when the node starts.
    serial = None
    SerialException = Exception
    SERIAL_IMPORT_ERROR = error
else:
    SERIAL_IMPORT_ERROR = None


class SerialBridge(Node):
    """Translate the Nano CMD/ENC serial protocol to ROS 2 topics."""

    def __init__(self) -> None:
        super().__init__('serial_bridge')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.23)
        self.declare_parameter('encoder_ticks_per_wheel_revolution', 0.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('max_linear_velocity', 0.05)
        self.declare_parameter('max_angular_velocity', 0.5)

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_wheel_rev = self.get_parameter(
            'encoder_ticks_per_wheel_revolution').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        self.command_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.command_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/wheel/odom', 20)
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 20)
        self.poll_timer = self.create_timer(0.01, self.poll_serial)
        self.reconnect_timer = self.create_timer(1.0, self.connect)

        self.serial: Optional[Any] = None
        self.last_left_ticks: Optional[int] = None
        self.last_right_ticks: Optional[int] = None
        self.last_stamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.invalid_ticks_reported = False
        self.last_command_log_time = 0.0
        self.last_command_signature = None

        if SERIAL_IMPORT_ERROR is not None:
            self.get_logger().error(
                f'pyserial is unavailable. Install python3-serial: {SERIAL_IMPORT_ERROR}')
        else:
            self.connect()

    def connect(self) -> None:
        """Open the serial device if it is not already available."""
        if self.serial is not None or serial is None:
            return
        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=0)
            self.serial.reset_input_buffer()
            self.get_logger().info(
                f'Connected to Arduino on {self.serial_port} at {self.baudrate} baud.')
        except (SerialException, OSError) as error:
            self.get_logger().debug(
                f'Arduino not available on {self.serial_port}: {error}')

    def disconnect(self, reason: Exception) -> None:
        """Close a failed connection and retry from the reconnect timer."""
        self.get_logger().warning(f'Arduino serial connection lost: {reason}')
        if self.serial is not None:
            self.serial.close()
        self.serial = None
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_stamp = None

    def command_callback(self, message: Twist) -> None:
        """Forward a differential-drive velocity command to the Nano."""
        if self.serial is None:
            return
        if not math.isfinite(message.linear.x) or not math.isfinite(message.angular.z):
            self.get_logger().warning('Rejected non-finite /cmd_vel and sent a stop command.')
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = max(
                -self.max_linear_velocity,
                min(message.linear.x, self.max_linear_velocity))
            angular_velocity = max(
                -self.max_angular_velocity,
                min(message.angular.z, self.max_angular_velocity))
        command = f'CMD {linear_velocity:.4f} {angular_velocity:.4f}\n'
        try:
            self.serial.write(command.encode('ascii'))
            self.log_motor_command(linear_velocity, angular_velocity)
        except (SerialException, OSError) as error:
            self.disconnect(error)

    def log_motor_command(self, linear_velocity: float, angular_velocity: float) -> None:
        """Log the wheel commands forwarded to the Nano without flooding output."""
        half_track = self.wheel_separation * 0.5
        left_velocity = linear_velocity - angular_velocity * half_track
        right_velocity = linear_velocity + angular_velocity * half_track
        left_direction = self.direction_name(left_velocity)
        right_direction = self.direction_name(right_velocity)
        signature = (left_direction, right_direction)
        now = time.monotonic()

        if signature != self.last_command_signature or now - self.last_command_log_time >= 1.0:
            self.get_logger().info(
                'Forwarding motor command: '
                f'left {left_direction} ({left_velocity:.3f} m/s), '
                f'right {right_direction} ({right_velocity:.3f} m/s).')
            self.last_command_signature = signature
            self.last_command_log_time = now

    @staticmethod
    def direction_name(velocity: float) -> str:
        """Return a readable wheel direction for command logging."""
        if velocity > 1e-4:
            return 'forward'
        if velocity < -1e-4:
            return 'reverse'
        return 'stopped'

    def poll_serial(self) -> None:
        """Consume all complete encoder lines currently waiting on USB serial."""
        if self.serial is None:
            return
        try:
            while self.serial.in_waiting:
                line = self.serial.readline().decode('ascii', errors='replace').strip()
                if line:
                    self.handle_line(line)
        except (SerialException, OSError) as error:
            self.disconnect(error)

    def handle_line(self, line: str) -> None:
        """Parse `ENC <millis> <left_ticks> <right_ticks>` from the Nano."""
        fields = line.split()
        if len(fields) != 4 or fields[0] != 'ENC':
            self.get_logger().debug(f'Ignoring Nano serial line: {line}')
            return
        try:
            left_ticks = int(fields[2])
            right_ticks = int(fields[3])
        except ValueError:
            self.get_logger().warning(f'Invalid encoder line from Nano: {line}')
            return

        stamp = self.get_clock().now().to_msg()
        self.publish_joint_states(stamp, left_ticks, right_ticks)
        self.publish_odometry(stamp, left_ticks, right_ticks)

    def publish_joint_states(self, stamp, left_ticks: int, right_ticks: int) -> None:
        """Publish wheel joint positions once encoder scale is configured."""
        if self.ticks_per_wheel_rev <= 0.0:
            return
        radians_per_tick = 2.0 * math.pi / self.ticks_per_wheel_rev
        message = JointState()
        message.header.stamp = stamp
        message.name = ['left_wheel_joint', 'right_wheel_joint']
        message.position = [left_ticks * radians_per_tick, right_ticks * radians_per_tick]
        self.joint_state_publisher.publish(message)

    def publish_odometry(self, stamp, left_ticks: int, right_ticks: int) -> None:
        """Integrate encoder changes into differential-drive wheel odometry."""
        if self.ticks_per_wheel_rev <= 0.0:
            if not self.invalid_ticks_reported:
                self.get_logger().error(
                    'encoder_ticks_per_wheel_revolution must be greater than zero; '
                    'not publishing /wheel/odom.')
                self.invalid_ticks_reported = True
            return

        now_seconds = stamp.sec + stamp.nanosec * 1e-9
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.last_stamp = now_seconds
            return

        delta_left_ticks = left_ticks - self.last_left_ticks
        delta_right_ticks = right_ticks - self.last_right_ticks
        delta_left = (delta_left_ticks / self.ticks_per_wheel_rev) * 2.0 * math.pi * self.wheel_radius
        delta_right = (delta_right_ticks / self.ticks_per_wheel_rev) * 2.0 * math.pi * self.wheel_radius
        distance = 0.5 * (delta_left + delta_right)
        delta_yaw = (delta_right - delta_left) / self.wheel_separation

        self.x += distance * math.cos(self.yaw + 0.5 * delta_yaw)
        self.y += distance * math.sin(self.yaw + 0.5 * delta_yaw)
        self.yaw = math.atan2(math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw))

        elapsed = max(now_seconds - self.last_stamp, 1e-6)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = self.odom_frame
        message.child_frame_id = self.base_frame
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = math.sin(0.5 * self.yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self.yaw)
        message.twist.twist.linear.x = distance / elapsed
        message.twist.twist.angular.z = delta_yaw / elapsed
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.1
        message.twist.covariance[0] = 0.1
        message.twist.covariance[35] = 0.2
        self.odom_publisher.publish(message)

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        self.last_stamp = now_seconds


def main() -> None:
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial is not None:
            try:
                node.serial.write(b'CMD 0.0000 0.0000\n')
            except (SerialException, OSError):
                pass
            node.serial.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
