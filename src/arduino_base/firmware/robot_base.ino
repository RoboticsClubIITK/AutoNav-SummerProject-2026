// Arduino Nano firmware for a two-wheel differential-drive robot.
// Motor driver: Cytron SmartDriveDuo-10 (MDDS10), PWM + direction mode.
// Serial protocol:
//   Pi -> Nano: CMD <linear_mps> <angular_radps>\n
//   Nano -> Pi: ENC <millis> <left_ticks> <right_ticks>\n
//
// IMPORTANT: Configure the MDDS10 for independent PWM + direction MCU control
// according to its manual. Connect these Nano pins to the matching labels on
// the MDDS10 input header, not by connector position.

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// MDDS10 channel 1 drives the left motor.
constexpr uint8_t kLeftPwmPin = 9;   // MDDS10 PWM1
constexpr uint8_t kLeftDirPin = 7;   // MDDS10 DIR1
// MDDS10 channel 2 drives the right motor.
constexpr uint8_t kRightPwmPin = 10; // MDDS10 PWM2
constexpr uint8_t kRightDirPin = 8;  // MDDS10 DIR2

// Encoder wiring: M, V, A, B, G, M. Power V from regulated Nano 5 V and join
// all grounds. D2/D3 are the Nano's external interrupt inputs.
constexpr uint8_t kLeftEncoderAPin = 2;
constexpr uint8_t kLeftEncoderBPin = 4;
constexpr uint8_t kRightEncoderAPin = 3;
constexpr uint8_t kRightEncoderBPin = 5;

// Measured physical values. Keep these synchronized with arduino_bridge.yaml.
constexpr float kWheelRadiusMeters = 0.03F;
constexpr float kWheelSeparationMeters = 0.23F;
constexpr float kMaxWheelSpeedMetersPerSecond = 0.45F;
constexpr unsigned long kCommandTimeoutMs = 250;
constexpr unsigned long kEncoderPublishPeriodMs = 20;

// Change either value only if a positive forward command drives that side
// backward after the physical wiring has been checked.
constexpr bool kLeftForwardDirHigh = true;
constexpr bool kRightForwardDirHigh = true;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
unsigned long lastCommandMs = 0;
unsigned long lastEncoderPublishMs = 0;
char commandBuffer[64];
uint8_t commandLength = 0;

void onLeftEncoderA() {
  const bool a = digitalRead(kLeftEncoderAPin);
  const bool b = digitalRead(kLeftEncoderBPin);
  leftTicks += (a == b) ? 1 : -1;
}

void onRightEncoderA() {
  const bool a = digitalRead(kRightEncoderAPin);
  const bool b = digitalRead(kRightEncoderBPin);
  rightTicks += (a == b) ? 1 : -1;
}

void setMotor(uint8_t pwmPin, uint8_t dirPin, float speedMetersPerSecond,
              bool forwardDirHigh) {
  const float limited = constrain(
      speedMetersPerSecond, -kMaxWheelSpeedMetersPerSecond,
      kMaxWheelSpeedMetersPerSecond);
  const bool forward = limited >= 0.0F;
  const bool directionHigh = forward ? forwardDirHigh : !forwardDirHigh;
  const uint8_t pwm = static_cast<uint8_t>(
      255.0F * fabs(limited) / kMaxWheelSpeedMetersPerSecond);

  digitalWrite(dirPin, directionHigh ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(kLeftPwmPin, 0);
  analogWrite(kRightPwmPin, 0);
}

void applyVelocityCommand(float linearMetersPerSecond,
                          float angularRadiansPerSecond) {
  const float halfTrack = kWheelSeparationMeters * 0.5F;
  const float leftSpeed = linearMetersPerSecond - angularRadiansPerSecond * halfTrack;
  const float rightSpeed = linearMetersPerSecond + angularRadiansPerSecond * halfTrack;

  setMotor(kLeftPwmPin, kLeftDirPin, leftSpeed, kLeftForwardDirHigh);
  setMotor(kRightPwmPin, kRightDirPin, rightSpeed, kRightForwardDirHigh);
  lastCommandMs = millis();
}

void processCommand(const char * command) {
  float linear = 0.0F;
  float angular = 0.0F;
  if (sscanf(command, "CMD %f %f", &linear, &angular) == 2) {
    applyVelocityCommand(linear, angular);
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());
    if (received == '\n' || received == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      commandLength = 0;
    }
  }
}

void publishEncoderTicks() {
  const unsigned long now = millis();
  if (now - lastEncoderPublishMs < kEncoderPublishPeriodMs) {
    return;
  }
  lastEncoderPublishMs = now;

  noInterrupts();
  const long left = leftTicks;
  const long right = rightTicks;
  interrupts();

  Serial.print(F("ENC "));
  Serial.print(now);
  Serial.print(' ');
  Serial.print(left);
  Serial.print(' ');
  Serial.println(right);
}

void setup() {
  pinMode(kLeftPwmPin, OUTPUT);
  pinMode(kLeftDirPin, OUTPUT);
  pinMode(kRightPwmPin, OUTPUT);
  pinMode(kRightDirPin, OUTPUT);
  stopMotors();

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), onLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), onRightEncoderA, RISING);

  Serial.begin(115200);
  lastCommandMs = millis();
}

void loop() {
  readSerialCommands();

  if (millis() - lastCommandMs > kCommandTimeoutMs) {
    stopMotors();
  }

  publishEncoderTicks();
}
