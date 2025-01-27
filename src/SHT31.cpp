#include "esp32-hal.h"
#include <stdint.h>
#include <include/SHT31.hpp>

SHT31_Error_Codes SHT31::sensorRead(){
  writeCommand(MEAS_HIG);

  delay(40);

  wireHandle->requestFrom(i2cAddress, 6);

  if (wireHandle->readBytes(buffer, 6) != 6        ) 
    return SHT31_Error_Codes::BUFFER_NOT_FILLED;
  if (crc8(buffer,     2)              != buffer[2]) 
    return SHT31_Error_Codes::TEMP_CRC_FAILED; // Temperature CRC
  if (crc8(buffer + 3, 2)              != buffer[5]) 
    return SHT31_Error_Codes::HUMD_CRC_FAILED; // Humidity CRC
  
  uint16_t tempT = ((uint16_t)buffer[0] << 8) | buffer[1];
  uint16_t tempH = ((uint16_t)buffer[3] << 8) | buffer[4];
  
  temperature  = tempT;
  humidity     = tempH;

  temperature *= 175;
  temperature /= 0xFFFF;
  temperature -= 45;

  humidity    *= 100;
  humidity    /= 0xFFFF;

  return SHT31_Error_Codes::NONE;
}

void SHT31::writeCommand(const uint8_t command[2]){
  wireHandle->beginTransmission(i2cAddress);

  wireHandle->write(command[1]);
  wireHandle->write(command[0]);

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
  switch (sensorRead()) {
  case SHT31_Error_Codes::NONE:
    return temperature;
  default:
    return NAN;
  }
}

double SHT31::readHumidity(){
  switch (sensorRead()) {
  case SHT31_Error_Codes::NONE:
    return humidity;
  default:
    return NAN;
  }
}

template<NUMBER T, NUMBER H>
SHT31_Error_Codes SHT31::readBoth(T& t, H& h){
  SHT31_Error_Codes result = sensorRead();

  switch (result) {
  case SHT31_Error_Codes::NONE:
    break;
  default:
    t = -128;
    h =  255;
    return result;
  }

  // Rounding
  t = temperature + 0.5;
  h = humidity + 0.5;

  return SHT31_Error_Codes::NONE;
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

template SHT31_Error_Codes SHT31::readBoth<int8_t, uint8_t>(int8_t&, uint8_t&);