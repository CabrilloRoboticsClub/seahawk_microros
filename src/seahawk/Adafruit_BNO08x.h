/*!
 *  @file Adafruit_BNO08x.h
 *
 * 	I2C Driver for the Adafruit BNO08x 9-DOF Orientation IMU Fusion Breakout
 *
 * 	This is a library for the Adafruit BNO08x breakout:
 * 	https://www.adafruit.com/products/4754
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_BNO08x_H
#define _ADAFRUIT_BNO08x_H

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "Adafruit_Sensor.h"

#define BNO08x_I2CADDR_DEFAULT 0x4A ///< The default I2C address

/* Additional Activities not listed in SH-2 lib */
#define PAC_ON_STAIRS 8 ///< Activity code for being on stairs
#define PAC_OPTION_COUNT 9 ///< The number of current options for the activity classifier

/*
FIX THIS: This is a dummy 
*/
class DummyI2CDevice {
  public:
  DummyI2CDevice(uint8_t addr /*bus devce here*/) {};
  uint8_t address(void) { return 0; };
  bool begin(bool addr_detect = true) { return false; };
  void end(void) {};
  bool detected(void) { return false; };

  bool read(uint8_t *buffer, size_t len, bool stop = true) { return false; };
  bool write(const uint8_t *buffer, size_t len, bool stop = true,
             const uint8_t *prefix_buffer = nullptr, size_t prefix_len = 0) { return false; };
  bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       bool stop = false) { return false; };
  bool setSpeed(uint32_t desiredclk) { return false; };

  /*!   @brief  How many bytes we can read in a transaction
   *    @return The size of the Wire receive/transmit buffer */
  size_t maxBufferSize() { return 0;}


};

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the BNO08x 9-DOF Orientation IMU Fusion Breakout
 */
class Adafruit_BNO08x {
public:
  Adafruit_BNO08x(int8_t reset_pin = -1);
  ~Adafruit_BNO08x();

  bool begin_I2C(uint8_t i2c_addr = BNO08x_I2CADDR_DEFAULT, int32_t sensor_id = 0);

  void hardwareReset(void);
  bool wasReset(void);

  bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000);
  bool getSensorEvent(sh2_SensorValue_t *value);

  sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor

protected:
  virtual bool _init(int32_t sensor_id);

  sh2_Hal_t
      _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};

#endif
