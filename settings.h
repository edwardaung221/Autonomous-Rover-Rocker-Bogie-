#pragma once
#include "data_types.h"

#define MOTORS_ENABLED 1
#define SENSORS_ENABLED 1
#define SENSOR_TEST_MODE 0

// Loop time
constexpr int LOOP_DELAY = 20; // 50Hz loop


constexpr int SAFETY_TIMER_TIMEOUT_S = 3;

// Laser pins
constexpr int SDA_PIN = 22;
constexpr int SCL_PIN = 21;

// Ultra pins
constexpr int TRIG1_PIN = 19;
constexpr int ECHO1_PIN = 23; // Right ultra
constexpr int TRIG2_PIN = 5;
constexpr int ECHO2_PIN = 18; // Left ultra

// Motor pins
constexpr int LEFT_PWM_PIN = 27; // PWM2
constexpr int LEFT_DIR_PIN = 33;
constexpr int RIGHT_PWM_PIN = 25; // PWM1
constexpr int RIGHT_DIR_PIN = 26;

// PWM channels
constexpr int LEFT_CHANNEL = 0;
constexpr int RIGHT_CHANNEL = 1;

// PWM setup
constexpr int PWM_FREQUENCY = 10000;
constexpr int PWM_RESOLUTION = 12;

// Dir levels
// Flip these
constexpr int LEFT_FORWARD = LOW;
constexpr int RIGHT_FORWARD = LOW;
constexpr int LEFT_REVERSE = HIGH;
constexpr int RIGHT_REVERSE = HIGH;

// Speed values
// PWM range
constexpr uint32_t FULL_SPEED = 2500;
constexpr uint32_t MIN_SPEED = 700;
constexpr uint32_t REVERSE_SPEED = 1500;
constexpr uint32_t PIVOT_SPEED = 1600; // Pivot speed
constexpr uint32_t ESCAPE_SPEED = 1400; // Escape speed

// Smooth speed
constexpr uint32_t ACCEL_STEP = 150;
constexpr uint32_t BRAKE_STEP = 200;

// Distance limits
// Rover choices
constexpr Millimeters LASER_MIN = 50;
constexpr Millimeters LASER_MAX = 550;
constexpr Millimeters ULTRA_MIN = 20;
constexpr Millimeters ULTRA_MAX = 2000;
constexpr Millimeters STOP_DISTANCE = 150;
constexpr Millimeters RESUME_DISTANCE = 200;
constexpr Millimeters FULL_SPEED_DIST = 500;
constexpr Millimeters TOO_CLOSE_DIST = 80; // Too close
constexpr Millimeters SPEED_BRAKE_MARGIN = 100;

// Laser smooth
constexpr float SMOOTHING_FACTOR = 0.40f;

// Laser fails
constexpr uint32_t LASER_TIMING_BUDGET = 33000;
constexpr int MAX_LASER_FAILS = 3;

// Fail streak
constexpr uint32_t SENSOR_DEGRADED_FAIL_STREAK = 10; // Fail limit

// Avoid times
// Real time
constexpr uint32_t REVERSE_DURATION_MS = 1000;
constexpr uint32_t TURN_DURATION_MS = 750;
constexpr uint32_t DRIVE_AROUND_MS = 1000;
constexpr uint32_t STRAIGHTEN_DURATION_MS = 750;
constexpr uint32_t COOL_DOWN_TIMEOUT_MS = 3000;

// Escape times
// Pivot first
constexpr uint32_t ESCAPE_PIVOT_MS = 800;
constexpr uint32_t ESCAPE_FORWARD_MS = 1200;

// Debounce counts
// Double check
constexpr int OBSTACLE_CONFIRM_TICKS = 2;
constexpr int ABORT_CONFIRM_TICKS = 2;

// Ultra timing
constexpr uint32_t ULTRA_ECHO_TIMEOUT = 12000; // Echo limit
constexpr uint32_t ULTRA_INTER_SENSOR_DELAY = 5; // Small gap

// Stuck check
// Escape trigger
constexpr int MAX_ABORTS_BEFORE_ESCAPE = 4;
constexpr uint32_t ABORT_WINDOW_MS = 8000;

// Cal tables
// Elsewhere data
extern const Millimeters laserRawCalib[];
extern const Millimeters laserTrueCalib[];
extern const size_t LASER_CALIB_COUNT;

extern const Millimeters ultraRawCalib[];
extern const Millimeters ultraTrueCalib[];
extern const size_t ULTRA_CALIB_COUNT;
