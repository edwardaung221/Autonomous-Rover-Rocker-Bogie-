#include "safety_watch.h"
#include "settings.h"
#include <Arduino.h>

// Safety watch


static uint32_t abortStamps[MAX_ABORTS_BEFORE_ESCAPE] = {0};
static int abortHead = 0;
static int abortCount = 0;

// Sensor health
static SensorHealth laserH, u1H, u2H;

// Health update
static void updateOne(SensorHealth &h, bool valid, uint32_t now)
{
  if (valid)
  {
    // Good read
    h.lastValidMs = now;
    h.failStreak = 0;

    // Sensor back
    if (h.degraded)
    {
      Serial.println("[HEALTH] sensor recovered");
      h.degraded = false;
    }
  }
  else
  {
    // Bad read
    h.failStreak++;

    // Sensor failed
    if (!h.degraded && h.failStreak >= SENSOR_DEGRADED_FAIL_STREAK)
    {
      h.degraded = true;
      Serial.printf("[HEALTH] sensor degraded — %u consecutive failures\n",
                    (unsigned)h.failStreak);
    }
  }
}

// Clear memory
void safetyWatchBegin()
{
  for (int i = 0; i < MAX_ABORTS_BEFORE_ESCAPE; i++) abortStamps[i] = 0;
  abortHead = 0;
  abortCount = 0;
  laserH = u1H = u2H = SensorHealth{};
}

// Save abort
void recordAbort()
{
  abortStamps[abortHead] = millis();

  // Next slot
  abortHead = (abortHead + 1) % MAX_ABORTS_BEFORE_ESCAPE;
  if (abortCount < MAX_ABORTS_BEFORE_ESCAPE) abortCount++;
}

// Clear aborts
void clearAborts()
{
  for (int i = 0; i < MAX_ABORTS_BEFORE_ESCAPE; i++) abortStamps[i] = 0;
  abortHead = 0;
  abortCount = 0;
}

// Stuck check
bool isStuckInLoop()
{
  // Full first
  if (abortCount < MAX_ABORTS_BEFORE_ESCAPE) return false;

  uint32_t now = millis();
  // Too old
  for (int i = 0; i < MAX_ABORTS_BEFORE_ESCAPE; i++)
  {
    if (now - abortStamps[i] > ABORT_WINDOW_MS) return false;
  }
  return true;
}

// All health
void updateSensorHealth(bool laserValid, bool u1Valid, bool u2Valid)
{
  uint32_t now = millis();
  updateOne(laserH, laserValid, now);
  updateOne(u1H, u1Valid, now);
  updateOne(u2H, u2Valid, now);
}

// Count healthy
int countHealthySensors()
{
  int n = 0;
  if (!laserH.degraded) n++;
  if (!u1H.degraded) n++;
  if (!u2H.degraded) n++;
  return n;
}

// Health getters
const SensorHealth &getLaserHealth()
{
  return laserH;
}

const SensorHealth &getUltra1Health()
{
  return u1H;
}

const SensorHealth &getUltra2Health()
{
  return u2H;
}
