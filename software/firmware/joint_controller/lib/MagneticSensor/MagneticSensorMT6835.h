/**
 * @file MagneticSensorMT6835.h
 * @brief Wrapper class for MT6835 magnetic encoder sensor
 *
 * Provides a simplified interface to the MT6835 magnetic encoder,
 * extending the base MT6835 class with sensor-specific methods.
 */

#pragma once

#include <MT6835.h>

/**
 * @class MagneticSensorMT6835
 * @brief Magnetic sensor interface for MT6835 encoder
 *
 * This class wraps the MT6835 driver to provide a clean sensor interface
 * for reading angular position from magnetic encoders.
 */
class MagneticSensorMT6835 : public MT6835 {
public:
  /**
   * @brief Constructor
   * @param nCS Chip select pin number (default: -1)
   * @param settings SPI settings for communication
   */
  MagneticSensorMT6835(int nCS = -1, SPISettings settings = MT6835SPISettings);
  
  /** @brief Destructor */
  virtual ~MagneticSensorMT6835();

  /**
   * @brief Get current sensor angle
   * @return Current angle in radians
   */
  virtual float getSensorAngle();

  /**
   * @brief Initialize the sensor with SPI interface
   * @param _spi Pointer to SPI interface (default: &SPI)
   */
  virtual void init(SPIClass *_spi = &SPI);
};

