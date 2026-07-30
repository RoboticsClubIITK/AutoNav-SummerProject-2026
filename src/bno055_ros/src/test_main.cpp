#include <cstdio>
#include <thread>
#include <chrono>

#include "bno055_ros/bno055.hpp"

int main()
{
  bno055_ros::Bno055 imu("/dev/i2c-1", 0x28);

  if (!imu.initialize()) {
    fprintf(stderr, "Failed to initialize BNO055\n");
    return 1;
  }
  printf("BNO055 initialized successfully.\n");

  for (int i = 0; i < 50; ++i) {
    bno055_ros::Vector3 accel, gyro;
    bno055_ros::Quaternion quat;
    uint8_t sys, g, a, m;

    imu.readAccelerometer(accel);
    imu.readGyroscope(gyro);
    imu.readQuaternion(quat);
    imu.readCalibration(sys, g, a, m);

    printf(
      "accel[m/s^2] x=%.3f y=%.3f z=%.3f | "
      "gyro[rad/s] x=%.3f y=%.3f z=%.3f | "
      "quat w=%.3f x=%.3f y=%.3f z=%.3f | "
      "calib sys=%d gyro=%d accel=%d mag=%d\n",
      accel.x, accel.y, accel.z,
      gyro.x, gyro.y, gyro.z,
      quat.w, quat.x, quat.y, quat.z,
      sys, g, a, m);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  return 0;
}
