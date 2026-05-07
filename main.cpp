#include <Arduino.h>
#include "esp_task_wdt.h"
#include "settings.h"
#include "sensors.h"
#include "motors.h"
#include "safety_watch.h"
#include "behavior.h"

// Main file

void setup()
{
  // Serial start
  Serial.begin(115200);

#if MOTORS_ENABLED
  // Motors live
  Serial.println("[START] Motors ON");
#else
  // Motors off
  Serial.println("[START] STRAGHT DRIVE MODE");
  Serial.println("[START] Set MOTORS_ENABLED 1 in settings.h when wired");
#endif

#if SENSORS_ENABLED
  // Sensors on
  Serial.println("[START] Sensors ON");
#if SENSOR_TEST_MODE
  Serial.println("[START] Sensor test mode");
#endif
#else
  // Ramp mode
  Serial.println("[START] Sensors OFF - straight drive only");
  Serial.println("[START] Set SENSORS_ENABLED 1 in settings.h for obstacle avoidance");
#endif

  // Start parts
  motorsBegin();

#if SENSORS_ENABLED
  sensorsBegin();
  safetyWatchBegin();
  behaviorBegin();
#else

  Serial.println("[START] No sensor safety checks in this mode");
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t safetyCfg =
  {
    .timeout_ms = SAFETY_TIMER_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&safetyCfg);
#else
  esp_task_wdt_init(SAFETY_TIMER_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
  Serial.printf("[START] Safety timer armed (%ds)\n", SAFETY_TIMER_TIMEOUT_S);
}

void loop()
{
  esp_task_wdt_reset(); // Safety timer feed — tells it the loop is alive

#if SENSORS_ENABLED
  // Sensor boxes
  SensorData laserRead, u1Read, u2Read;

  // Sensor read
  readAllSensors(laserRead, u1Read, u2Read);

  // Health check
  updateSensorHealth(laserRead.valid, u1Read.valid, u2Read.valid);

#if SENSOR_TEST_MODE
  static uint32_t lastSensorPrintMs = 0;
  if (millis() - lastSensorPrintMs >= 250)
  {
    lastSensorPrintMs = millis();
    Serial.printf("[SENSOR] laser:%s raw:%u smooth:%u dist:%u | right:%s raw:%u dist:%u | left:%s raw:%u dist:%u\n",
                  laserRead.valid ? "OK" : "BAD",
                  (unsigned)laserRead.rawDistance,
                  (unsigned)laserRead.smoothDistance,
                  (unsigned)laserRead.distance,
                  u1Read.valid ? "OK" : "BAD",
                  (unsigned)u1Read.rawDistance,
                  (unsigned)u1Read.distance,
                  u2Read.valid ? "OK" : "BAD",
                  (unsigned)u2Read.rawDistance,
                  (unsigned)u2Read.distance);
  }
#else
  // Brain update
  updateBehavior(laserRead, u1Read, u2Read);
#endif
#else
  // Straight mode
  if (getMotorSpeed() < FULL_SPEED)
  {
    rampSpeedTo(FULL_SPEED);
  }
#endif

  // Tiny wait
  delay(LOOP_DELAY);
}
