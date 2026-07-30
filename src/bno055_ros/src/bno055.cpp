#include "bno055_ros/bno055.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <cstring>
#include <cstdio>
#include <thread>
#include <chrono>

namespace bno055_ros
{

Bno055::Bno055(const std::string & i2c_bus, uint8_t address)
: i2c_bus_(i2c_bus), address_(address), fd_(-1)
{
}

Bno055::~Bno055()
{
  if (fd_ >= 0) {
    close(fd_);
  }
}

bool Bno055::writeByte(uint8_t reg, uint8_t value)
{
  uint8_t buf[2] = {reg, value};
  if (write(fd_, buf, 2) != 2) {
    perror("bno055: write failed");
    return false;
  }
  return true;
}

bool Bno055::readBytes(uint8_t reg, uint8_t * buffer, size_t length)
{
  if (write(fd_, &reg, 1) != 1) {
    perror("bno055: register select failed");
    return false;
  }
  if (read(fd_, buffer, length) != static_cast<ssize_t>(length)) {
    perror("bno055: read failed");
    return false;
  }
  return true;
}

bool Bno055::initialize()
{
  fd_ = open(i2c_bus_.c_str(), O_RDWR);
  if (fd_ < 0) {
    perror("bno055: failed to open i2c bus");
    return false;
  }

  if (ioctl(fd_, I2C_SLAVE, address_) < 0) {
    perror("bno055: failed to set i2c slave address");
    return false;
  }

  uint8_t chip_id = 0;
  if (!readBytes(static_cast<uint8_t>(Register::CHIP_ID), &chip_id, 1)) {
    return false;
  }
  if (chip_id != 0xA0) {
    fprintf(stderr, "bno055: unexpected chip id 0x%02X (expected 0xA0)\n", chip_id);
    return false;
  }

  // Ensure config mode before changing settings
  if (!writeByte(static_cast<uint8_t>(Register::OPR_MODE),
      static_cast<uint8_t>(OprMode::CONFIG_MODE)))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // Normal power mode
  if (!writeByte(static_cast<uint8_t>(Register::PWR_MODE), 0x00)) {
    return false;
  }

  // Use internal oscillator, no reset, no self test
  if (!writeByte(static_cast<uint8_t>(Register::SYS_TRIGGER), 0x00)) {
    return false;
  }

  // Switch to NDOF fusion mode
  if (!writeByte(static_cast<uint8_t>(Register::OPR_MODE),
      static_cast<uint8_t>(OprMode::NDOF)))
  {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  return true;
}

bool Bno055::readAccelerometer(Vector3 & out)
{
  uint8_t raw[6];
  if (!readBytes(static_cast<uint8_t>(Register::ACC_DATA_X_LSB), raw, 6)) {
    return false;
  }
  int16_t x = static_cast<int16_t>(raw[0] | (raw[1] << 8));
  int16_t y = static_cast<int16_t>(raw[2] | (raw[3] << 8));
  int16_t z = static_cast<int16_t>(raw[4] | (raw[5] << 8));

  // Accelerometer unit: 1 m/s^2 = 100 LSB (default unit setting)
  out.x = x / 100.0;
  out.y = y / 100.0;
  out.z = z / 100.0;
  return true;
}

bool Bno055::readGyroscope(Vector3 & out)
{
  uint8_t raw[6];
  if (!readBytes(static_cast<uint8_t>(Register::GYR_DATA_X_LSB), raw, 6)) {
    return false;
  }
  int16_t x = static_cast<int16_t>(raw[0] | (raw[1] << 8));
  int16_t y = static_cast<int16_t>(raw[2] | (raw[3] << 8));
  int16_t z = static_cast<int16_t>(raw[4] | (raw[5] << 8));

  // Gyroscope unit: 1 rad/s = 900 LSB (default unit setting is dps;
  // we convert to rad/s here since ROS uses SI units)
  constexpr double kDegToRad = 0.0174533;
  out.x = (x / 16.0) * kDegToRad;
  out.y = (y / 16.0) * kDegToRad;
  out.z = (z / 16.0) * kDegToRad;
  return true;
}

bool Bno055::readQuaternion(Quaternion & out)
{
  uint8_t raw[8];
  if (!readBytes(static_cast<uint8_t>(Register::QUA_DATA_W_LSB), raw, 8)) {
    return false;
  }
  int16_t w = static_cast<int16_t>(raw[0] | (raw[1] << 8));
  int16_t x = static_cast<int16_t>(raw[2] | (raw[3] << 8));
  int16_t y = static_cast<int16_t>(raw[4] | (raw[5] << 8));
  int16_t z = static_cast<int16_t>(raw[6] | (raw[7] << 8));

  // Quaternion scale factor: 1 unit = 2^14 LSB
  constexpr double kScale = 1.0 / 16384.0;
  out.w = w * kScale;
  out.x = x * kScale;
  out.y = y * kScale;
  out.z = z * kScale;
  return true;
}

bool Bno055::readCalibration(uint8_t & sys, uint8_t & gyro, uint8_t & accel, uint8_t & mag)
{
  uint8_t raw = 0;
  if (!readBytes(static_cast<uint8_t>(Register::CALIB_STAT), &raw, 1)) {
    return false;
  }
  sys   = (raw >> 6) & 0x03;
  gyro  = (raw >> 4) & 0x03;
  accel = (raw >> 2) & 0x03;
  mag   = raw & 0x03;
  return true;
}

bool Bno055::readTemperature(int8_t & out)
{
  uint8_t raw = 0;
  if (!readBytes(static_cast<uint8_t>(Register::TEMP), &raw, 1)) {
    return false;
  }
  out = static_cast<int8_t>(raw);
  return true;
}

}  // namespace bno055_ros
