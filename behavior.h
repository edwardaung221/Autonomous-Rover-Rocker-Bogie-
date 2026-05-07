#pragma once
#include "data_types.h"

// Brain header

// Brain start
void behaviorBegin();

// Brain tick
void updateBehavior(const SensorData &laser,
                    const SensorData &u1,
                    const SensorData &u2);
