#pragma once
#include <Arduino.h>

// Millimetres
using Millimeters = uint32_t;

// Smaller one
inline Millimeters mmMin(Millimeters a, Millimeters b)
{
  return a < b ? a : b;
}

// Sensor reading
struct SensorData
{
  // Good read
  bool valid = false;

  // Raw value
  Millimeters rawDistance = 0;

  // Smooth value
  Millimeters smoothDistance = 0;

  // Final value
  Millimeters distance = 0;
};

enum Behavior
{
  DRIVING_FORWARD,
  REVERSING,
  TURNING,
  DRIVING_AROUND,
  STRAIGHTENING,
  ESCAPING
};

// Turn choice
enum TurnDirection { TURN_RIGHT, TURN_LEFT };

// Sensor health
struct SensorHealth
{
  uint32_t lastValidMs = 0; // Last good
  uint32_t failStreak = 0; // Bad count

  // Sensor bad
  bool degraded = false;
};
