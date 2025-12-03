/**
 * @file MagneticSensorMT6835.cpp
 * @brief Implementation of MT6835 magnetic encoder sensor wrapper
 */

#include <MagneticSensorMT6835.h>

MagneticSensorMT6835::MagneticSensorMT6835(int nCS, SPISettings settings)
    : MT6835(settings, nCS) {
  // Initialize base class
}

MagneticSensorMT6835::~MagneticSensorMT6835() {
  // Cleanup handled by base class
}

float MagneticSensorMT6835::getSensorAngle() {
  return getCurrentAngle();
}

void MagneticSensorMT6835::init(SPIClass *_spi) {
  this->MT6835::init(_spi);
}

