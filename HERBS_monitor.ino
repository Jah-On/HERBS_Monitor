/*
Code for the HERBS (Hive Environmental Reporting Broadcast System) Monitor(s)

Author(s): 
  John Schulz (john.schulz1@protonmail.com)
*/

// Comment out for actual use to save energy
// Otherwise it will print useful info over serial
#define DEBUG 

// STL Includes
#include <chrono> // Time related
#include <list>   // Front/Back optimized lists

// Arduino Includes
#include <timers.h>

// External Includes
#include <heltec_unofficial.h>
#include <arduino-timer.h>

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

typedef struct PacketData {
  uint64_t id;
  int8_t   temperature; // Celcius
  uint8_t  humidity;    // Percentage from 0 to 100
  uint16_t preassure;   // Millibars
} PacketData;

PacketData data = {
  0x1234567890ABCDEF,
  0,
  0,
  0
};

size_t sentCount = 0;

Timer<8, millis> timer;

void setup() {
  heltec_setup();

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
  data.temperature = (int8_t)heltec_temperature();
  return true;
}

bool broadcastPacket(void* cbData){
  radio.transmit((uint8_t*)&data, sizeof(PacketData));

  sentCount++;

  return true;
}

bool updateDisplay(void* cbData){
  display.clear();

  display.drawString(0, 0, "Sent: " + String(sentCount));

  display.display();

  return true;
}