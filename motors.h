#pragma once
#include "data_types.h"

// Motor header

// Motor setup
void motorsBegin();

// Basic moves
void stopMotors();
void driveForward(uint32_t speed);
void driveReverse(uint32_t speed);

// Tank turn
void turnRight  (uint32_t speed);
void turnLeft   (uint32_t speed);

// Pivot turn
void pivotRight (uint32_t speed);
void pivotLeft  (uint32_t speed);

// Speed ramp
void rampSpeedTo(uint32_t target);

// Helper bits
uint32_t getMotorSpeed();
uint32_t distanceToSpeed(Millimeters dist);

// Stop maths
Millimeters dynamicStopDistance(uint32_t commandedSpeed);
