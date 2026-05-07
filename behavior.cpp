#include "behavior.h"
#include "settings.h"
#include "motors.h"
#include "safety_watch.h"
#include <Arduino.h>

// Rover brain

static Behavior currentBehavior = DRIVING_FORWARD;
static uint32_t behaviorStartedMs = 0; // Start time
static uint32_t behaviorDurationMs = 0; // State time
static TurnDirection chosenTurnDirection = TURN_RIGHT;
static bool justFinishedAvoiding = false;
static uint32_t coolDownStartedMs = 0;

// Debounce counters
static int tooCloseTickCount = 0;
static int obstacleCloseTickCount = 0;

// Debug counter
static int printCounter = 0;

// Escape steps
enum EscapePhase { ESC_PIVOT, ESC_FORWARD };
static EscapePhase escapePhase = ESC_PIVOT;

// Change state
static inline void switchToBehavior(Behavior b, uint32_t durationMs)
{
  currentBehavior = b;
  behaviorStartedMs = millis();
  behaviorDurationMs = durationMs;
  tooCloseTickCount = 0;
  obstacleCloseTickCount = 0;
}

// Timer done
static inline bool behaviorExpired()
{
  return (millis() - behaviorStartedMs) >= behaviorDurationMs;
}

// Pick side
static TurnDirection chooseTurnDirection(const SensorData &u1, const SensorData &u2)
{
  Millimeters leftSpace = u2.valid ? u2.distance : ULTRA_MAX;
  Millimeters rightSpace = u1.valid ? u1.distance : ULTRA_MAX;
  return (rightSpace >= leftSpace) ? TURN_LEFT : TURN_RIGHT;
}

// Pivot chosen
static void pivotInDirection(TurnDirection dir, uint32_t speed)
{
  if (dir == TURN_RIGHT) pivotRight(speed);
  else pivotLeft(speed);
}

// Pivot back
static void pivotOpposite(TurnDirection dir, uint32_t speed)
{
  if (dir == TURN_RIGHT) pivotLeft(speed);
  else pivotRight(speed);
}

// Start brain
void behaviorBegin()
{
  switchToBehavior(DRIVING_FORWARD, 0);
  chosenTurnDirection = TURN_RIGHT;
  justFinishedAvoiding = false;
  coolDownStartedMs = 0;
  printCounter = 0;
  escapePhase = ESC_PIVOT;
}

// Brain loop
void updateBehavior(const SensorData &laserRead,
                    const SensorData &ultra1Read,
                    const SensorData &ultra2Read)
{
  // Sensor check
  bool anySensorValid = laserRead.valid || ultra1Read.valid || ultra2Read.valid;
  bool enoughSensors = countHealthySensors() >= 2;

  // All sensors
  Millimeters closestDist = ULTRA_MAX;
  if (ultra1Read.valid) closestDist = mmMin(closestDist, ultra1Read.distance);
  if (ultra2Read.valid) closestDist = mmMin(closestDist, ultra2Read.distance);
  if (laserRead.valid) closestDist = mmMin(closestDist, laserRead.smoothDistance);

  // Side sensors
  Millimeters closestUltra = ULTRA_MAX;
  if (ultra1Read.valid) closestUltra = mmMin(closestUltra, ultra1Read.distance);
  if (ultra2Read.valid) closestUltra = mmMin(closestUltra, ultra2Read.distance);

  // Stop gap
  uint32_t cmdSpeed = (currentBehavior == DRIVING_FORWARD) ? getMotorSpeed() : 0;
  Millimeters stopDist = dynamicStopDistance(cmdSpeed);

  // Obstacle check
  bool obstacleClose = (closestDist < stopDist);

  // Open path
  bool pathIsOpen = !anySensorValid || (closestDist > FULL_SPEED_DIST);

  bool tooCloseToContinue = (closestUltra < TOO_CLOSE_DIST);

  // Stuck escape
  if (currentBehavior != ESCAPING &&
      currentBehavior != DRIVING_FORWARD &&
      isStuckInLoop())
  {
    Serial.println("[SAFETY] Stuck in loop — entering ESCAPING");
    clearAborts();
    escapePhase = ESC_PIVOT;
    switchToBehavior(ESCAPING, ESCAPE_PIVOT_MS);

    // Start escape
    chosenTurnDirection = chooseTurnDirection(ultra1Read, ultra2Read);
    pivotInDirection(chosenTurnDirection, PIVOT_SPEED);
    return;
  }

  // Log gap
  printCounter++;

  // State machine
  switch (currentBehavior)
  {

    // Forward state
    case DRIVING_FORWARD:
    {
      // Confirm obstacle
      obstacleCloseTickCount = obstacleClose ? obstacleCloseTickCount + 1 : 0;
      bool confirmedObstacleClose = (obstacleCloseTickCount >= OBSTACLE_CONFIRM_TICKS);

      // Avoid cooldown
      if (justFinishedAvoiding)
      {
        bool timedOut = (millis() - coolDownStartedMs) > COOL_DOWN_TIMEOUT_MS;
        bool pathRecovered = anySensorValid && (closestDist > RESUME_DISTANCE);
        if (pathRecovered || timedOut)
        {
          justFinishedAvoiding = false;
          obstacleCloseTickCount = 0;
          confirmedObstacleClose = false;
          if (timedOut)
            Serial.println("[WARN] Cool-down timed out — forcing avoidance retry");
        }
      }

      // Health gate
      if (!anySensorValid || !enoughSensors)
      {
        rampSpeedTo(FULL_SPEED);
        if (printCounter % 20 == 0)
          Serial.println("[OPEN] No sensor data — cruising forward");
        break;
      }

      if (confirmedObstacleClose && !justFinishedAvoiding)
      {
        // Choose space
        Millimeters leftSpace = ultra2Read.valid ? ultra2Read.distance : ULTRA_MAX;
        Millimeters rightSpace = ultra1Read.valid ? ultra1Read.distance : ULTRA_MAX;

        // Laser only
        bool laserOnly = laserRead.valid
                       && laserRead.smoothDistance < stopDist
                       && leftSpace > stopDist
                       && rightSpace > stopDist;

        // Pick turn
        if (laserOnly) chosenTurnDirection = TURN_LEFT;
        else chosenTurnDirection = (rightSpace >= leftSpace) ? TURN_LEFT : TURN_RIGHT;

        // Reverse first
        switchToBehavior(REVERSING, REVERSE_DURATION_MS);
        driveReverse(REVERSE_SPEED);
        Serial.printf("[AVOID] Obstacle at %umm (stop:%umm) — REVERSING | turning %s%s (L:%umm R:%umm laser:%umm)\n",
                      (unsigned)closestDist, (unsigned)stopDist,
                      chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT",
                      laserOnly ? " [laser-only]" : "",
                      (unsigned)leftSpace, (unsigned)rightSpace,
                      laserRead.valid ? (unsigned)laserRead.smoothDistance : 0);
      }
      else if (pathIsOpen)
      {
        // Speed up
        rampSpeedTo(FULL_SPEED);
        if (printCounter % 10 == 0)
          Serial.printf("[OK] FORWARD speed:%u | laser:%u u1:%u u2:%u closest:%u\n",
                        getMotorSpeed(),
                        (unsigned)laserRead.distance,
                        (unsigned)ultra1Read.distance,
                        (unsigned)ultra2Read.distance,
                        (unsigned)closestDist);
      }
      else
      {
        // Slow down
        if (anySensorValid) rampSpeedTo(distanceToSpeed(closestDist));
        else rampSpeedTo(FULL_SPEED);
        if (printCounter % 10 == 0)
          Serial.printf("[SLOW] speed:%u | laser:%u u1:%u u2:%u closest:%u stop:%u\n",
                        getMotorSpeed(),
                        (unsigned)laserRead.distance,
                        (unsigned)ultra1Read.distance,
                        (unsigned)ultra2Read.distance,
                        (unsigned)closestDist, (unsigned)stopDist);
      }
      break;
    }

    // Reverse state
    case REVERSING:
    {
      driveReverse(REVERSE_SPEED);

      // Print time
      uint32_t remaining = behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs);
      if (printCounter % 5 == 0)
        Serial.printf("[AVOID] REVERSING — %ums left | closest:%u laser:%u u1:%u u2:%u\n",
                      (unsigned)remaining, (unsigned)closestDist,
                      laserRead.valid ? (unsigned)laserRead.smoothDistance : 0,
                      ultra1Read.valid ? (unsigned)ultra1Read.distance : 0,
                      ultra2Read.valid ? (unsigned)ultra2Read.distance : 0);
      if (behaviorExpired())
      {
        // Start turning
        switchToBehavior(TURNING, TURN_DURATION_MS);
      }
      break;
    }

    // Turn state
    case TURNING:
    {
      // Abort check
      tooCloseTickCount = tooCloseToContinue ? tooCloseTickCount + 1 : 0;
      bool confirmedAbort = (tooCloseTickCount >= ABORT_CONFIRM_TICKS);

      if (confirmedAbort)
      {
        // Retry avoid
        recordAbort();
        chosenTurnDirection = chooseTurnDirection(ultra1Read, ultra2Read);
        switchToBehavior(REVERSING, REVERSE_DURATION_MS);
        driveReverse(REVERSE_SPEED);
        Serial.printf("[AVOID] TURN ABORTED — closest:%umm u1:%u u2:%u | retry %s\n",
                      (unsigned)closestDist,
                      ultra1Read.valid ? (unsigned)ultra1Read.distance : 0,
                      ultra2Read.valid ? (unsigned)ultra2Read.distance : 0,
                      chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT");
      }
      else
      {
        // Keep turning
        pivotInDirection(chosenTurnDirection, PIVOT_SPEED);
        if (printCounter % 5 == 0)
          Serial.printf("[AVOID] PIVOT %s — %ums left\n",
                        chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT",
                        (unsigned)(behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs)));
        if (behaviorExpired())
        {
          // Drive around
          switchToBehavior(DRIVING_AROUND, DRIVE_AROUND_MS);
        }
      }
      break;
    }

    // Around state
    case DRIVING_AROUND:
    {
      // Abort check
      tooCloseTickCount = tooCloseToContinue ? tooCloseTickCount + 1 : 0;
      bool confirmedAbort = (tooCloseTickCount >= ABORT_CONFIRM_TICKS);

      if (confirmedAbort)
      {
        // Retry avoid
        recordAbort();
        chosenTurnDirection = chooseTurnDirection(ultra1Read, ultra2Read);
        switchToBehavior(REVERSING, REVERSE_DURATION_MS);
        driveReverse(REVERSE_SPEED);
        Serial.printf("[AVOID] DRIVE-AROUND ABORTED — u1:%umm u2:%umm | retry %s\n",
                      (unsigned)ultra1Read.distance,
                      (unsigned)ultra2Read.distance,
                      chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT");
      }
      else
      {
        // Safety check
        if (!anySensorValid || !enoughSensors)
        {
          stopMotors();
          switchToBehavior(DRIVING_FORWARD, 0);
          Serial.printf("[WARN] Not enough healthy sensors (healthy=%d) — stopped\n",
                        countHealthySensors());
          break;
        }

        // Go around
        rampSpeedTo(FULL_SPEED);
        if (printCounter % 5 == 0)
          Serial.printf("[AVOID] DRIVING AROUND — %ums left | closest:%umm\n",
                        (unsigned)(behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs)),
                        (unsigned)closestDist);
        if (behaviorExpired())
        {
          // Straighten next
          switchToBehavior(STRAIGHTENING, STRAIGHTEN_DURATION_MS);
        }
      }
      break;
    }

    // Straight state
    case STRAIGHTENING:
    {
      // Abort check
      tooCloseTickCount = tooCloseToContinue ? tooCloseTickCount + 1 : 0;
      bool confirmedAbort = (tooCloseTickCount >= ABORT_CONFIRM_TICKS);

      if (confirmedAbort)
      {
        // Retry avoid
        recordAbort();
        switchToBehavior(REVERSING, REVERSE_DURATION_MS);
        driveReverse(REVERSE_SPEED);
        Serial.printf("[AVOID] STRAIGHTEN ABORTED — obstacle at %umm! Reversing\n",
                      (unsigned)closestDist);
      }
      else
      {
        // Turn back
        pivotOpposite(chosenTurnDirection, PIVOT_SPEED);
        if (printCounter % 5 == 0)
          Serial.printf("[AVOID] STRAIGHTENING — %ums left\n",
                        (unsigned)(behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs)));
        if (behaviorExpired())
        {
          // Avoid done
          switchToBehavior(DRIVING_FORWARD, 0);
          stopMotors();
          justFinishedAvoiding = true;
          coolDownStartedMs = millis();
          Serial.printf("[OK] Avoidance complete — resuming (cool-down until %umm or %ums)\n",
                        (unsigned)RESUME_DISTANCE, (unsigned)COOL_DOWN_TIMEOUT_MS);
        }
      }
      break;
    }

    // Escape state
    case ESCAPING:
    {
      // Escape phases
      if (escapePhase == ESC_PIVOT)
      {
        // Keep pivoting
        pivotInDirection(chosenTurnDirection, PIVOT_SPEED);
        if (printCounter % 5 == 0)
          Serial.printf("[ESCAPE] pivoting %s — %ums left\n",
                        chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT",
                        (unsigned)(behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs)));
        if (behaviorExpired())
        {
          // Drive next
          escapePhase = ESC_FORWARD;
          behaviorStartedMs = millis();
          behaviorDurationMs = ESCAPE_FORWARD_MS;
        }
      }
      else
      {
        // Block check
        if (closestUltra < STOP_DISTANCE || closestDist < STOP_DISTANCE)
        {
          // Flip turn
          chosenTurnDirection = (chosenTurnDirection == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;
          escapePhase = ESC_PIVOT;
          behaviorStartedMs = millis();
          behaviorDurationMs = ESCAPE_PIVOT_MS;
          stopMotors();
          Serial.printf("[ESCAPE] still blocked — flipping to %s\n",
                        chosenTurnDirection == TURN_RIGHT ? "RIGHT" : "LEFT");
        }
        else
        {
          // Safety check
          if (!anySensorValid || !enoughSensors)
          {
            stopMotors();
            switchToBehavior(DRIVING_FORWARD, 0);
            Serial.printf("[WARN] Not enough healthy sensors (healthy=%d) — stopped\n",
                          countHealthySensors());
            break;
          }

          // Escape drive
          rampSpeedTo(ESCAPE_SPEED);
          if (printCounter % 5 == 0)
            Serial.printf("[ESCAPE] driving — %ums left | u1:%u u2:%u\n",
                          (unsigned)(behaviorDurationMs - mmMin(millis() - behaviorStartedMs, behaviorDurationMs)),
                          (unsigned)ultra1Read.distance, (unsigned)ultra2Read.distance);
          if (behaviorExpired())
          {
            // Escape done
            switchToBehavior(DRIVING_FORWARD, 0);
            stopMotors();
            justFinishedAvoiding = true;
            coolDownStartedMs = millis();
            Serial.println("[ESCAPE] complete — resuming forward drive");
          }
        }
      }
      break;
    }

    // Backup state
    default:
      Serial.printf("[ERROR] Unknown behaviour %d — resetting\n", (int)currentBehavior);
      switchToBehavior(DRIVING_FORWARD, 0);
      stopMotors();
      break;
  }
}
