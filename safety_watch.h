#pragma once
#include "data_types.h"


// Safety setup
void safetyWatchBegin();

// Save abort
void recordAbort();

// Clear aborts
void clearAborts();

// Stuck check
bool isStuckInLoop();

// Health update
void updateSensorHealth(bool laserValid, bool u1Valid, bool u2Valid);

// Count healthy
int countHealthySensors();

// Health read
const SensorHealth &getLaserHealth();
const SensorHealth &getUltra1Health();
const SensorHealth &getUltra2Health();
