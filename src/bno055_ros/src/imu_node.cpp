#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "bno055_ros/bno055.hpp"

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node
{
public:
  ImuNode()
  : Node("bno055_imu_node"),
    imu_("/dev/i2c-1", 0x28)
  {
    this->declare_parameter<std::string>("frame_id", "imu_link");
    frame_id_ = this->get_parameter("frame_id").as_string();

    if (!imu_.initialize()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize BNO055. Shutting down.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "BNO055 initialized.");

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    timer_ = this->create_wall_timer(
      20ms, std::bind(&ImuNode::publishImu, this));  // ~50 Hz
  }

private:
  void publishImu()
  {
    bno055_ros::Vector3 accel, gyro;
    bno055_ros::Quaternion quat;

    bool ok = true;
    ok &= imu_.readAccelerometer(accel);
    ok &= imu_.readGyroscope(gyro);
    ok &= imu_.readQuaternion(quat);

    if (!ok) {
      RCLCPP_WARN(this->get_logger(), "Failed to read from BNO055.");
      return;
    }

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    msg.orientation.w = quat.w;
    msg.orientation.x = quat.x;
    msg.orientation.y = quat.y;
    msg.orientation.z = quat.z;

    msg.angular_velocity.x = gyro.x;
    msg.angular_velocity.y = gyro.y;
    msg.angular_velocity.z = gyro.z;

    msg.linear_acceleration.x = accel.x;
    msg.linear_acceleration.y = accel.y;
    msg.linear_acceleration.z = accel.z;

    // Covariances: BNO055 datasheet does not give exact noise density,
    // so these are reasonable placeholder estimates. Tune later using
    // static-sensor variance measurements.
    for (int i = 0; i < 9; ++i) {
      msg.orientation_covariance[i] = 0.0;
      msg.angular_velocity_covariance[i] = 0.0;
      msg.linear_acceleration_covariance[i] = 0.0;
    }
    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[8] = 0.01;

    msg.angular_velocity_covariance[0] = 0.001;
    msg.angular_velocity_covariance[4] = 0.001;
    msg.angular_velocity_covariance[8] = 0.001;

    msg.linear_acceleration_covariance[0] = 0.01;
    msg.linear_acceleration_covariance[4] = 0.01;
    msg.linear_acceleration_covariance[8] = 0.01;

    publisher_->publish(msg);
  }

  bno055_ros::Bno055 imu_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
