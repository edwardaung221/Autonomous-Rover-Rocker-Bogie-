#include "settings.h"

// Real values

// Laser cal
const Millimeters laserRawCalib[] = {79, 136, 237, 332, 413, 480, 536};
const Millimeters laserTrueCalib[] = {50, 100, 200, 300, 400, 500, 550};
const size_t LASER_CALIB_COUNT = sizeof(laserRawCalib) / sizeof(*laserRawCalib);

// Ultra cal
const Millimeters ultraRawCalib[] = {20, 49, 102, 189, 294, 390, 493, 580, 789, 985, 1970};
const Millimeters ultraTrueCalib[] = {20, 50, 100, 200, 300, 400, 500, 600, 800, 1000, 2000};
const size_t ULTRA_CALIB_COUNT = sizeof(ultraRawCalib) / sizeof(*ultraRawCalib);
