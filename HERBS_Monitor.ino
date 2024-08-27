/*
Code for the HERBS (Hive Environmental Reporting Broadcast System) Monitor(s)

Author(s): 
  John Schulz (john.schulz1@protonmail.com)
*/

// Comment out for actual use to save energy
// Otherwise it will print useful info over serial
#define DEBUG 

// STL Includes
#include <array>  // Fixed size lists
#include <chrono> // Time related
#include <list>   // Front/Back optimized lists

// Arduino Includes
#include <timers.h>

// External Includes
#include <float16.h>
#include <heltec_unofficial.h>
#include <mbedtls/aes.h>
#include <arduino-timer.h>

// Include keys
#include "secrets.h"

// LoRa constants
#define LORA_FREQUENCY_US        905.2

#define LORA_BANDWIDTH_7_8       7.8
#define LORA_BANDWIDTH_20_8      20.8
#define LORA_BANDWIDTH_62_5      62.5

#define LORA_SPREADING_FACTOR_5  5
#define LORA_SPREADING_FACTOR_6  6
#define LORA_SPREADING_FACTOR_7  7
#define LORA_SPREADING_FACTOR_8  8
#define LORA_SPREADING_FACTOR_9  9
#define LORA_SPREADING_FACTOR_10 10
#define LORA_SPREADING_FACTOR_11 11
#define LORA_SPREADING_FACTOR_12 12

#define LORA_CODING_RATE_4_5     5
#define LORA_CODING_RATE_4_6     6
#define LORA_CODING_RATE_4_7     7
#define LORA_CODING_RATE_4_8     8

#define MONITOR_ID 0x1234567890ABCDEF

#define PACKET_BUFFER_SIZE 16

typedef struct PacketData {
  uint64_t id          = MONITOR_ID;
  uint8_t  packetNumber;
  int8_t   temperature;  // Celcius
  uint8_t  humidity;     // Percentage from 0 to 100
  uint16_t preassure;    // Millibars
  float16  acoustics;    // Decibels
  uint16_t hiveMass;     // Grams
} PacketData;

const size_t packetSize = sizeof(PacketData);

std::array<PacketData, PACKET_BUFFER_SIZE> packetBuffer;

size_t currentPacket = 0;

size_t sentCount = 0;

Timer<8, millis> timer;

esp_aes_context aesContext;

void setup() {
  heltec_setup();

  for (size_t index = 0; index < PACKET_BUFFER_SIZE; index++){
    packetBuffer[index].packetNumber = index;
  }

  // AES Init
  esp_aes_init(&aesContext);
  esp_aes_setkey(&aesContext, AES_KEY, 128);

  // Setup display
  display.setFont(ArialMT_Plain_16);
  display.clear();

  // Init functions
  switch (LoRaInit()) {
  case true:
    display.println("LoRa Initialized");
    break;
  default:
    display.println("LoRa Failed");
    break;
  }

  display.display();

  timer.every(1e4, broadcastPacket);
  timer.every(1e3, updateTemperature);
  timer.every(1e2, updateDisplay);
}


void loop() {
  timer.tick();
  heltec_delay(1);
}

bool LoRaInit(){
  int16_t res = radio.begin(
    LORA_FREQUENCY_US,
    LORA_BANDWIDTH_20_8,
    LORA_SPREADING_FACTOR_5,
    LORA_CODING_RATE_4_5,
    0x12,
    22
  );

  return res == 0;
}

bool updateTemperature(void* cbData){
  packetBuffer[currentPacket].temperature = (int8_t)heltec_temperature();
  return true;
}

bool broadcastPacket(void* cbData){
  uint8_t encryptedBuffer[packetSize];
  
  esp_aes_crypt_cbc(
    &aesContext, 
    MBEDTLS_AES_ENCRYPT,
    16, 
    aesIV,
    (uint8_t*)&packetBuffer.at(currentPacket),
    encryptedBuffer
  );

#if defined(DEBUG)
  Serial.printf("Sending packet %d\n", packetBuffer.at(currentPacket).packetNumber);
#endif

  radio.transmit(encryptedBuffer, 16);

  sentCount++;

  currentPacket++;

  return true;
}

bool updateDisplay(void* cbData){
  display.clear();

  display.drawString(0, 0, "Sent: " + String(sentCount));

  display.display();

  return true;
}
