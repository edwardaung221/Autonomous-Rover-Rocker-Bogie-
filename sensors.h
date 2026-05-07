#pragma once
#include "data_types.h"

// Sensor header

// Sensor setup
void sensorsBegin();

// Sensor read
void readAllSensors(SensorData &laserOut,
                    SensorData &u1Out,
                    SensorData &u2Out);
