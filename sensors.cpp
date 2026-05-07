#include "sensors.h"
#include "settings.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <string.h>
#include <math.h>

// Sensor code

// Private stuff

static VL53L0X laser;
static bool laserWorking = false;
static int laserFailCount = 0; // Safe start
static bool laserSmoothingReady = false;
static float laserSmoothed = 0.0f;

static Millimeters ultra1Readings[5] = {};
static int ultra1Count = 0;
static int ultra1Index = 0;

static Millimeters ultra2Readings[5] = {};
static int ultra2Count = 0;
static int ultra2Index = 0;


// Helper bits

// Limit reading
static Millimeters limitDistance(int32_t value, Millimeters minVal, Millimeters maxVal)
{
  if (value < (int32_t)minVal) return minVal;
  if (value > (int32_t)maxVal) return maxVal;
  return (Millimeters)value;
}

static Millimeters calibrate(Millimeters raw,
                             const Millimeters *rawPts,
                             const Millimeters *truePts,
                             size_t count)
{
  if (count < 2) return raw;
  if (raw <= rawPts[0]) return truePts[0];
  if (raw >= rawPts[count - 1]) return truePts[count - 1];

  for (size_t i = 0; i + 1 < count; i++)
  {
    if (raw >= rawPts[i] && raw <= rawPts[i + 1])
    {
      int32_t num = (int32_t)(raw - rawPts[i]) * (int32_t)(truePts[i + 1] - truePts[i]);
      int32_t den = (int32_t)(rawPts[i + 1] - rawPts[i]);
      return (Millimeters)((int32_t)truePts[i] + num / den);
    }
  }
  return raw;
}

// Ultra filter
static Millimeters medianOf5(const Millimeters *values)
{
  Millimeters sorted[5];
  memcpy(sorted, values, sizeof(sorted));

  for (int i = 0; i < 4; i++)
  {
    for (int j = i + 1; j < 5; j++)
    {
      if (sorted[j] < sorted[i])
      {
        Millimeters t = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = t;
      }
    }
  }
  return sorted[2];
}

// Ultra read
static bool readUltrasonic(int trigPin, int echoPin, Millimeters *result)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  uint32_t start = micros();
  while (digitalRead(echoPin) == LOW)
  {
    if ((uint32_t)(micros() - start) > ULTRA_ECHO_TIMEOUT) return false;
  }

  uint32_t echoStart = micros();
  uint32_t echoEnd = echoStart;
  while (digitalRead(echoPin) == HIGH)
  {
    echoEnd = micros();
    if ((uint32_t)(echoEnd - echoStart) > ULTRA_ECHO_TIMEOUT) return false;
  }

  *result = (Millimeters)((echoEnd - echoStart) * 343 / 2000);
  return true;
}

static void configureLaser()
{
  laser.setSignalRateLimit(0.1f);
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  laser.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
  laser.setMeasurementTimingBudget(LASER_TIMING_BUDGET);
  laser.startContinuous();
}

// I2C fix
static void recoverI2CBus()
{
  Wire.end();
  pinMode(SCL_PIN, OUTPUT);
  pinMode(SDA_PIN, INPUT_PULLUP);

  for (int i = 0; i < 16; i++) // Clock pulses
  {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }

  digitalWrite(SCL_PIN, HIGH);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(2);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Serial.println("[I2C] bus recovery attempted");
}

// Public calls

void sensorsBegin()
{
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // Fast I2C
  laser.setTimeout(100);

  pinMode(TRIG1_PIN, OUTPUT);
  pinMode(TRIG2_PIN, OUTPUT);
  pinMode(ECHO1_PIN, INPUT);
  pinMode(ECHO2_PIN, INPUT);
  digitalWrite(TRIG1_PIN, LOW); // Known state
  digitalWrite(TRIG2_PIN, LOW);

  laserWorking = laser.init();
  if (!laserWorking) Serial.println("[ERROR] Laser sensor failed to start");
  else configureLaser();
}

void readAllSensors(SensorData &laserOut, SensorData &u1Out, SensorData &u2Out)
{
  laserOut = SensorData{};
  u1Out = SensorData{};
  u2Out = SensorData{};

  // Laser bit
  if (laserWorking)
  {
    uint16_t raw = laser.readRangeContinuousMillimeters();
    // Bad laser
    laserOut.valid = !laser.timeoutOccurred() && raw < 8190 && raw >= LASER_MIN;

    if (!laserOut.valid)
    {
      if (++laserFailCount >= MAX_LASER_FAILS)
      {
        laserWorking = false;
        laserFailCount = 0;
        laserSmoothingReady = false;
      }
    }
    else
    {
      laserFailCount = 0;
      laserOut.rawDistance = raw;
      laserOut.distance = limitDistance(
        (int32_t)calibrate(raw, laserRawCalib, laserTrueCalib, LASER_CALIB_COUNT),
        LASER_MIN, LASER_MAX);

      if (!laserSmoothingReady)
      {
        laserSmoothed = (float)laserOut.distance;
        laserSmoothingReady = true;
      }
      else
      {
        laserSmoothed = SMOOTHING_FACTOR * (float)laserOut.distance
                      + (1.0f - SMOOTHING_FACTOR) * laserSmoothed;
      }
      laserOut.smoothDistance = (Millimeters)(laserSmoothed + 0.5f);
    }
  }
  else
  {
    // Retry laser
    static int laserReinitFails = 0;
    laserWorking = laser.init();
    if (laserWorking)
    {
      configureLaser();
      laserReinitFails = 0;
    }
    else if (++laserReinitFails >= 2)
    {
      recoverI2CBus();
      laserReinitFails = 0;
    }
  }

  // Ultra one
  Millimeters u1Raw = 0;
  u1Out.valid = readUltrasonic(TRIG1_PIN, ECHO1_PIN, &u1Raw);
  if (u1Out.valid && u1Raw < ULTRA_MIN) u1Out.valid = false; // Motor noise
  if (!u1Out.valid)
  {
    ultra1Count = 0;
    ultra1Index = 0;
  }
  if (u1Out.valid)
  {
    u1Out.rawDistance = u1Raw;
    ultra1Readings[ultra1Index] = u1Raw;
    ultra1Index = (ultra1Index + 1) % 5;
    if (ultra1Count < 5) ultra1Count++;
    u1Out.smoothDistance = (ultra1Count == 5) ? medianOf5(ultra1Readings) : u1Raw;
    u1Out.distance = limitDistance(
      (int32_t)calibrate(u1Out.smoothDistance, ultraRawCalib, ultraTrueCalib, ULTRA_CALIB_COUNT),
      ULTRA_MIN, ULTRA_MAX);
  }

  delay(ULTRA_INTER_SENSOR_DELAY); // Sound gap

  // Ultra two
  Millimeters u2Raw = 0;
  u2Out.valid = readUltrasonic(TRIG2_PIN, ECHO2_PIN, &u2Raw);
  if (u2Out.valid && u2Raw < ULTRA_MIN) u2Out.valid = false;
  if (!u2Out.valid)
  {
    ultra2Count = 0;
    ultra2Index = 0;
  }
  if (u2Out.valid)
  {
    u2Out.rawDistance = u2Raw;
    ultra2Readings[ultra2Index] = u2Raw;
    ultra2Index = (ultra2Index + 1) % 5;
    if (ultra2Count < 5) ultra2Count++;
    u2Out.smoothDistance = (ultra2Count == 5) ? medianOf5(ultra2Readings) : u2Raw;
    u2Out.distance = limitDistance(
      (int32_t)calibrate(u2Out.smoothDistance, ultraRawCalib, ultraTrueCalib, ULTRA_CALIB_COUNT),
      ULTRA_MIN, ULTRA_MAX);
  }
}
