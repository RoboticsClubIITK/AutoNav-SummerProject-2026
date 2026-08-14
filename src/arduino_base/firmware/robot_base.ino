// Arduino Nano firmware for a two-wheel differential-drive robot.
// Motor driver: L298N dual H-bridge, ENA/IN1/IN2 and ENB/IN3/IN4 mode.
// Serial protocol:
//   Pi -> Nano: CMD <linear_mps> <angular_radps>\n
//   Nano -> Pi: ENC <millis> <left_ticks> <right_ticks>\n
//
// IMPORTANT: remove the ENA and ENB jumpers from the L298N module so Nano PWM
// controls the enable pins. The L298N is marginal for the 3.5 A locked-rotor
// current of these motors; use only brief lifted-wheel tests until a
// higher-current driver is available.

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

// L298N channel A drives the left motor.
constexpr uint8_t kLeftPwmPin = 9;   // ENA
constexpr uint8_t kLeftIn1Pin = 7;   // IN1
constexpr uint8_t kLeftIn2Pin = 8;   // IN2
// L298N channel B drives the right motor.
constexpr uint8_t kRightPwmPin = 10; // ENB
constexpr uint8_t kRightIn3Pin = 11; // IN3
constexpr uint8_t kRightIn4Pin = 12; // IN4

// Encoder wiring: M, V, A, B, G, M. Power V from regulated Nano 5 V and join
// all grounds. D2/D3 are the Nano's external interrupt inputs.
// DCGM-3865 encoder scale is 13 PPR * 42:1 gearbox = 546 ticks per output
// wheel revolution with the rising-edge A-channel decoder used below.
constexpr uint8_t kLeftEncoderAPin = 2;
constexpr uint8_t kLeftEncoderBPin = 4;
constexpr uint8_t kRightEncoderAPin = 3;
constexpr uint8_t kRightEncoderBPin = 5;

// Measured physical values. Keep these synchronized with arduino_bridge.yaml.
constexpr float kWheelRadiusMeters = 0.03F;
constexpr float kWheelSeparationMeters = 0.23F;
// Conservative initial limit for lifted-wheel and first ground tests. Increase
// only after direction, encoder, braking, and emergency-stop validation.
constexpr float kMaxWheelSpeedMetersPerSecond = 0.10F;
// Temporary drive floor for the L298N bench setup; the motors needed a higher
// PWM than the linear speed mapping would otherwise produce at low commands.
constexpr uint8_t kMinDrivePwm = 150;
constexpr unsigned long kCommandTimeoutMs = 250;
constexpr unsigned long kEncoderPublishPeriodMs = 20;

// Change either value only if a positive forward command drives that side
// backward after the physical wiring has been checked.
constexpr bool kLeftIn1HighForForward = true;
constexpr bool kRightIn3HighForForward = true;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
unsigned long lastCommandMs = 0;
unsigned long lastEncoderPublishMs = 0;
char commandBuffer[64];
uint8_t commandLength = 0;

void clearCommandBuffer() {
  commandLength = 0;
  commandBuffer[0] = '\0';
}

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

void setMotor(uint8_t pwmPin, uint8_t forwardPin, uint8_t reversePin,
              float speedMetersPerSecond, bool forwardPinHighForForward) {
  if (!isfinite(speedMetersPerSecond)) {
    analogWrite(pwmPin, 0);
    digitalWrite(forwardPin, LOW);
    digitalWrite(reversePin, LOW);
    return;
  }
  const float limited = constrain(
      speedMetersPerSecond, -kMaxWheelSpeedMetersPerSecond,
      kMaxWheelSpeedMetersPerSecond);
  if (fabs(limited) < 1e-4F) {
    analogWrite(pwmPin, 0);
    digitalWrite(forwardPin, LOW);
    digitalWrite(reversePin, LOW);
    return;
  }
  const bool forward = limited > 0.0F;
  const bool forwardPinHigh = forward ? forwardPinHighForForward : !forwardPinHighForForward;
  const float scaledPwm = kMinDrivePwm +
      (255.0F - kMinDrivePwm) * fabs(limited) / kMaxWheelSpeedMetersPerSecond;
  const uint8_t pwm = static_cast<uint8_t>(scaledPwm);

  digitalWrite(forwardPin, forwardPinHigh ? HIGH : LOW);
  digitalWrite(reversePin, forwardPinHigh ? LOW : HIGH);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(kLeftPwmPin, 0);
  analogWrite(kRightPwmPin, 0);
  digitalWrite(kLeftIn1Pin, LOW);
  digitalWrite(kLeftIn2Pin, LOW);
  digitalWrite(kRightIn3Pin, LOW);
  digitalWrite(kRightIn4Pin, LOW);
}

void applyVelocityCommand(float linearMetersPerSecond,
                          float angularRadiansPerSecond) {
  const float halfTrack = kWheelSeparationMeters * 0.5F;
  const float leftSpeed = linearMetersPerSecond - angularRadiansPerSecond * halfTrack;
  const float rightSpeed = linearMetersPerSecond + angularRadiansPerSecond * halfTrack;

  setMotor(kLeftPwmPin, kLeftIn1Pin, kLeftIn2Pin, leftSpeed, kLeftIn1HighForForward);
  setMotor(kRightPwmPin, kRightIn3Pin, kRightIn4Pin, rightSpeed, kRightIn3HighForForward);
  lastCommandMs = millis();
}

bool parseCommand(const char *command, float *linearMetersPerSecond,
                  float *angularRadiansPerSecond) {
  if (strncmp(command, "CMD", 3) != 0) {
    return false;
  }

  const char *cursor = command + 3;
  while (*cursor == ' ') {
    ++cursor;
  }

  char *endPointer = nullptr;
  const float linear = static_cast<float>(strtod(cursor, &endPointer));
  if (endPointer == cursor) {
    return false;
  }

  cursor = endPointer;
  while (*cursor == ' ') {
    ++cursor;
  }

  const float angular = static_cast<float>(strtod(cursor, &endPointer));
  if (endPointer == cursor) {
    return false;
  }

  while (*endPointer == ' ') {
    ++endPointer;
  }

  if (*endPointer != '\0' || !isfinite(linear) || !isfinite(angular)) {
    return false;
  }

  *linearMetersPerSecond = linear;
  *angularRadiansPerSecond = angular;
  return true;
}

void processCommand(const char *command) {
  float linear = 0.0F;
  float angular = 0.0F;

  if (parseCommand(command, &linear, &angular)) {
    applyVelocityCommand(linear, angular);
    return;
  }

  if (strcmp(command, "STOP") == 0 || strcmp(command, "stop") == 0) {
    stopMotors();
    lastCommandMs = millis();
    return;
  }

  stopMotors();
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());
    if (received == '\n' || received == '\r') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        clearCommandBuffer();
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = received;
    } else {
      clearCommandBuffer();
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
  pinMode(kLeftIn1Pin, OUTPUT);
  pinMode(kLeftIn2Pin, OUTPUT);
  pinMode(kRightPwmPin, OUTPUT);
  pinMode(kRightIn3Pin, OUTPUT);
  pinMode(kRightIn4Pin, OUTPUT);
  stopMotors();

  pinMode(kLeftEncoderAPin, INPUT_PULLUP);
  pinMode(kLeftEncoderBPin, INPUT_PULLUP);
  pinMode(kRightEncoderAPin, INPUT_PULLUP);
  pinMode(kRightEncoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kLeftEncoderAPin), onLeftEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(kRightEncoderAPin), onRightEncoderA, RISING);

  Serial.begin(115200);
  lastCommandMs = millis();
  clearCommandBuffer();
}

void loop() {
  readSerialCommands();

  if (millis() - lastCommandMs > kCommandTimeoutMs) {
    stopMotors();
  }

  publishEncoderTicks();
}
