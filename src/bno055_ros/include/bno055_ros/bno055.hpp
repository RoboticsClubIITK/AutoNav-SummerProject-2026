#ifndef BNO055_ROS__BNO055_HPP_
#define BNO055_ROS__BNO055_HPP_

#include <cstdint>
#include <string>

namespace bno055_ros
{

// BNO055 register addresses (Page 0)
enum class Register : uint8_t
{
  CHIP_ID       = 0x00,
  OPR_MODE      = 0x3D,
  PWR_MODE      = 0x3E,
  SYS_TRIGGER   = 0x3F,
  UNIT_SEL      = 0x3B,
  CALIB_STAT    = 0x35,
  TEMP          = 0x34,

  ACC_DATA_X_LSB = 0x08,
  GYR_DATA_X_LSB = 0x14,
  QUA_DATA_W_LSB = 0x20,
};

// Operation modes
enum class OprMode : uint8_t
{
  CONFIG_MODE = 0x00,
  NDOF        = 0x0C,   // fused, absolute orientation mode
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

class Bno055
{
public:
  explicit Bno055(const std::string & i2c_bus, uint8_t address = 0x28);
  ~Bno055();

  // Opens the I2C device, verifies chip ID, sets NDOF mode.
  // Returns true on success.
  bool initialize();

  bool readAccelerometer(Vector3 & out);
  bool readGyroscope(Vector3 & out);
  bool readQuaternion(Quaternion & out);
  bool readCalibration(uint8_t & sys, uint8_t & gyro, uint8_t & accel, uint8_t & mag);
  bool readTemperature(int8_t & out);

private:
  bool writeByte(uint8_t reg, uint8_t value);
  bool readBytes(uint8_t reg, uint8_t * buffer, size_t length);

  std::string i2c_bus_;
  uint8_t address_;
  int fd_;
};

}  // namespace bno055_ros

#endif  // BNO055_ROS__BNO055_HPP_
