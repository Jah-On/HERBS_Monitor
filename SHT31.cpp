#include <math.h>
#include <string.h>
#include <algorithm>
#include "HardwareSerial.h"
#include <cstddef>
#include <sys/types.h>
#include "WString.h"
#include <sys/_stdint.h>
#include "esp32-hal.h"
#include "Wire.h"
#include "SHT31.hpp"

bool SHT31::sensorRead(){
  writeCommand(MEAS_HIG);

  delay(16);

  wireHandle->requestFrom(i2cAddress, 6);

  if (wireHandle->readBytes(buffer, 6) != 6        ) return false;
  if (crc8(buffer, 2)                  != buffer[2]) return false; // Temperature CRC
  if (crc8(buffer + 3, 2)              != buffer[5]) return false; // Humidity CRC
  
  uint16_t tempT = ((uint16_t)buffer[0] << 8) | buffer[1];
  uint16_t tempH = ((uint16_t)buffer[3] << 8) | buffer[4];
  
  temperature  = tempT;
  humidity     = tempH;

  temperature *= 175;
  temperature /= 0xFFFF;
  temperature -= 45;

  humidity    *= 100;
  humidity    /= 0xFFFF;

  return true;
}

void SHT31::writeCommand(uint16_t command){
  wireHandle->beginTransmission(i2cAddress);

  wireHandle->write(command >> 0x08);
  wireHandle->write(command &  0xFF);

  wireHandle->endTransmission();
}

/***************** PUBLIC FUNCTIONS ****************/
SHT31::SHT31(TwoWire* handle){
  this->wireHandle = handle;
}

void SHT31::begin(uint8_t i2cAddress){
  this->i2cAddress = i2cAddress;

  writeCommand(SOFT_RESET);
}

double SHT31::readTemperature(){
  if (!sensorRead()) return NAN;

  return temperature;
}

double SHT31::readHumidity(){
  if (!sensorRead()) return NAN;

  return humidity;
}

template<NUMBER T, NUMBER H>
void SHT31::readBoth(T& t, H& h){
  if (!sensorRead()){
    t = NAN;
    h = NAN;
    return;
  }

  t = temperature;
  h = humidity;
}

uint8_t SHT31::crc8(uint8_t* encoded, size_t length){
  uint8_t       result         = 0xFF;
  const uint8_t CRC_POLYNOMIAL = 0x31;

  for (int j = length; j; --j) {
    result ^= *encoded++;

    for (int i = 8; i; --i) {
      result = (result & 0x80)
        ? (result << 1) ^ CRC_POLYNOMIAL
        : (result << 1);
    }
  }

  return result;
}

template void SHT31::readBoth<int8_t, uint8_t>(int8_t&, uint8_t&);