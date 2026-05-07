#include "motors.h"
#include "settings.h"
#include <Arduino.h>

// Last speed
static uint32_t motorSpeed = 0;

// Speed getter
uint32_t getMotorSpeed()
{
  return motorSpeed;
}

// Motor setup
void motorsBegin()
{
#if MOTORS_ENABLED
  // Dir pins
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  // PWM speed
  ledcSetup(LEFT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(RIGHT_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(LEFT_PWM_PIN, LEFT_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_CHANNEL);
#endif

  // Start safe
  stopMotors();
}

// Stop motors
void stopMotors()
{
  motorSpeed = 0;
#if MOTORS_ENABLED
  ledcWrite(LEFT_CHANNEL, 0);
  ledcWrite(RIGHT_CHANNEL, 0);
#else
  Serial.println("  [MOTOR] STOP");
#endif
}

// Go forward
void driveForward(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  // Forward PWM
  digitalWrite(LEFT_DIR_PIN, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_FORWARD);
  ledcWrite(LEFT_CHANNEL, speed);
  ledcWrite(RIGHT_CHANNEL, speed);
#else
  Serial.printf("[MOTOR] FORWARD speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Go back
void driveReverse(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  // Reverse PWM
  digitalWrite(LEFT_DIR_PIN, LEFT_REVERSE);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_REVERSE);
  ledcWrite(LEFT_CHANNEL, speed);
  ledcWrite(RIGHT_CHANNEL, speed);
#else
  Serial.printf("[MOTOR] REVERSE speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Turn right
void turnRight(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  digitalWrite(LEFT_DIR_PIN, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_FORWARD);
  ledcWrite(LEFT_CHANNEL, speed);
  ledcWrite(RIGHT_CHANNEL, 0);
#else
  Serial.printf("[MOTOR] TURN RIGHT speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Turn left
void turnLeft(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  digitalWrite(LEFT_DIR_PIN, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_FORWARD);
  ledcWrite(LEFT_CHANNEL, 0);
  ledcWrite(RIGHT_CHANNEL, speed);
#else
  Serial.printf("[MOTOR] TURN LEFT speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Pivot right
void pivotRight(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  digitalWrite(LEFT_DIR_PIN, LEFT_FORWARD);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_REVERSE);
  ledcWrite(LEFT_CHANNEL, speed);
  ledcWrite(RIGHT_CHANNEL, speed);
#else
  Serial.printf("[MOTOR] PIVOT RIGHT speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Pivot left
void pivotLeft(uint32_t speed)
{
  motorSpeed = speed;
#if MOTORS_ENABLED
  digitalWrite(LEFT_DIR_PIN, LEFT_REVERSE);
  digitalWrite(RIGHT_DIR_PIN, RIGHT_FORWARD);
  ledcWrite(LEFT_CHANNEL, speed);
  ledcWrite(RIGHT_CHANNEL, speed);
#else
  Serial.printf("[MOTOR] PIVOT LEFT speed=%u (%.0f%%)\n",
                speed, speed * 100.0f / 4095.0f);
#endif
}

// Speed ramp
void rampSpeedTo(uint32_t target)
{
  // Minimum push
  if (target > 0 && target < MIN_SPEED) target = MIN_SPEED;

  if (motorSpeed < target)
  {
    // Speed up
    if (motorSpeed < MIN_SPEED) motorSpeed = MIN_SPEED;
    uint32_t next = motorSpeed + ACCEL_STEP;
    motorSpeed = (next > target) ? target : next;
  }
  else if (motorSpeed > target)
  {
    // Slow down
    if (motorSpeed >= BRAKE_STEP && motorSpeed - BRAKE_STEP > target)
      motorSpeed -= BRAKE_STEP;
    else
      motorSpeed = target;
  }

  // Send speed
  driveForward(motorSpeed);
}

// Stop maths
Millimeters dynamicStopDistance(uint32_t commandedSpeed)
{
  if (commandedSpeed > FULL_SPEED) commandedSpeed = FULL_SPEED;
  return STOP_DISTANCE
       + (Millimeters)(SPEED_BRAKE_MARGIN * (float)commandedSpeed / (float)FULL_SPEED);
}

// Speed maths
uint32_t distanceToSpeed(Millimeters dist)
{
  if (dist <= STOP_DISTANCE) return 0;
  if (dist >= FULL_SPEED_DIST) return FULL_SPEED;

  // Simple scale
  uint32_t span = FULL_SPEED_DIST - STOP_DISTANCE;
  uint32_t into = dist - STOP_DISTANCE;
  return MIN_SPEED + (uint32_t)((FULL_SPEED - MIN_SPEED) * into / span);
}
