#ifndef Adafruit_I2CDevice_h
#define Adafruit_I2CDevice_h

#include <Arduino.h>
#include <Wire.h>

// This functionality is only supported on the ESP platform. Fail to compile if this gets enabled on other platforms
#if defined(ADAFRUIT_THREADSAFE_I2C) && !defined(ESP32) && !defined(ESP8266)
#error ADAFRUIT_THREADSAFE_I2C only supported on ESP32 or ESP8266
#endif

// When using I2C thread safety on the ESP platform, provide an optional timeout value. Default is no timeout. This can be tuned
// in the future as needed. This does nothing unless opted into I2C thread safety with ADAFRUIT_THREADSAFE_I2C
#ifndef ADAFRUIT_THREADSAFE_I2C_DEFAULT_TIMEOUT
#if defined(ESP32) || defined(ESP8266)
#define ADAFRUIT_THREADSAFE_I2C_DEFAULT_TIMEOUT portMAX_DELAY
#else
#define ADAFRUIT_THREADSAFE_I2C_DEFAULT_TIMEOUT 0
#endif
#endif

///< The class which defines how we will talk to this device over I2C
class Adafruit_I2CDevice {
public:
  Adafruit_I2CDevice(uint8_t addr, TwoWire *theWire = &Wire);
  uint8_t address(void);
  bool begin(bool addr_detect = true);
  void end(void);
  bool detected(void);

  bool read(uint8_t *buffer, size_t len, bool stop = true);
  bool write(const uint8_t *buffer, size_t len, bool stop = true,
             const uint8_t *prefix_buffer = NULL, size_t prefix_len = 0);
  bool write_then_read(const uint8_t *write_buffer, size_t write_len,
                       uint8_t *read_buffer, size_t read_len,
                       bool stop = false);
  bool setSpeed(uint32_t desiredclk);

  bool busy();

  /*!   @brief  How many bytes we can read in a transaction
   *    @return The size of the Wire receive/transmit buffer */
  size_t maxBufferSize() { return _maxBufferSize; }

private:
  uint8_t _addr;
  TwoWire *_wire;
  bool _begun;
  size_t _maxBufferSize;
  bool _read(uint8_t *buffer, size_t len, bool stop);

#ifdef ADAFRUIT_THREADSAFE_I2C
  SemaphoreHandle_t i2cSem = xSemaphoreCreateBinary();
#endif

  bool takeSemaphore(uint32_t timeout);
  bool giveSemaphore();
};

#endif // Adafruit_I2CDevice_h
