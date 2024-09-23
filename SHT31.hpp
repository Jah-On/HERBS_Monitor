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
  static constexpr uint16_t MEAS_HIG         = 0x2400;
  static constexpr uint16_t MEAS_HIG_STRECTH = 0x2C06;
  static constexpr uint16_t MEAS_MED         = 0x240B;
  static constexpr uint16_t MEAS_MED_STRECTH = 0x2C0D;
  static constexpr uint16_t MEAS_LOW         = 0x2416;
  static constexpr uint16_t MEAS_LOW_STRECTH = 0x2C10;
  static constexpr uint16_t SOFT_RESET       = 0x30A2;

  uint8_t  buffer[6];
  TwoWire* wireHandle;
  uint8_t  i2cAddress;
  double   temperature = NAN;
  double   humidity    = NAN;

  bool    sensorRead();
  void    writeCommand(uint16_t CMD);

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