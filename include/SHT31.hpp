#include <sys/_stdint.h>
#include <cstddef>
#ifndef SHT31_HPP
#define SHT31_HPP

#include <Wire.h>
#include <stdint.h>

template<typename X>
concept NUMBER = (std::is_integral_v<X> || std::is_floating_point_v<X>);

class SHT31 {
private:
  // Static members
  static constexpr uint8_t MEAS_HIG[2]         = {0x24, 0x00};
  static constexpr uint8_t MEAS_HIG_STRECTH[2] = {0x2C, 0x06};
  static constexpr uint8_t MEAS_MED[2]         = {0x24, 0x0B};
  static constexpr uint8_t MEAS_MED_STRECTH[2] = {0x2C, 0x0D};
  static constexpr uint8_t MEAS_LOW[2]         = {0x24, 0x16};
  static constexpr uint8_t MEAS_LOW_STRECTH[2] = {0x2C, 0x10};
  static constexpr uint8_t SOFT_RESET[2]       = {0x30, 0xA2};

  uint8_t  buffer[6];
  TwoWire* wireHandle;
  uint8_t  i2cAddress;
  double   temperature = NAN;
  double   humidity    = NAN;

  bool    sensorRead();
  void    writeCommand(const uint8_t command[2]);

public:
  SHT31(TwoWire* handle = &Wire);

  void begin(uint8_t i2cAddress = 0x44);

  double readTemperature();
  double readHumidity();

  template<NUMBER T, NUMBER H>
  void   readBoth(T& t, H& h);

  // Static Public Functions
  static uint8_t crc8(uint8_t* encoded, size_t length);
};

#endif