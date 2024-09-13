/*
Code for the HERBS (Hive Environmental Reporting Broadcast System) Monitor(s)

Author(s): 
  John Schulz (john.schulz1@protonmail.com)
*/

// Comment out for actual use to save energy
// Otherwise it will print useful info over serial
// #define DEBUG 

// HERBS Data Types
#include "herbsTypes.h"

// STL Includes
#include <array>  // Fixed size lists
#include <chrono> // Time related
#include <list>   // Front/Back optimized lists

// External Includes
#include <arduino-timer.h>
#include <Crypto.h>
#include <ChaChaPoly.h>
#include <heltec_unofficial.h>
#include <SHT31.h>

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

#define PACKET_BUFFER_SIZE 2

std::array<Packet, PACKET_BUFFER_SIZE> packetBuffer;

size_t currentPacket = 0;

ChaChaPoly crypto = ChaChaPoly();

size_t sentCount = 0;

Timer<6, millis> timer;

Timer<6, millis>::Task send;

SHT31 sht31 = SHT31();

void setup() {
  for (size_t packet = 0; packet < packetBuffer.size(); ++packet){
    packetBuffer[packet].id = MONITOR_ID;
  }

  heltec_setup();

  // Init encryption
  if (!crypto.setKey(encryption.key, 16)) throw("Could not set key");
  if (!crypto.setIV(encryption.iv, 8))    throw("Could not set IV");

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

  sht31.begin();

  radio.setPacketReceivedAction(onRecieve);
  radio.startReceive();

  send = timer.every(3e5, sendDataPacket);

  // Sensor timed callbacks
  timer.every(1e3, updateTemperature);
  timer.every(1e3, updateHumidity);

  timer.every(1e2, updateDisplay);

  sendEventPacket(EventCode::NODE_ONLINE);
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
  DataPacket& data = packetBuffer[currentPacket].type.data;
  data.temperature = (int8_t)sht31.getTemperature();
  return true;
}

bool updateHumidity(void* cbData){
  DataPacket& data = packetBuffer[currentPacket].type.data;
  data.humidity = (uint8_t)sht31.getHumidity();
  return true;
}

void sendEventPacket(EventCode event){
  EventPacket& packet = packetBuffer[currentPacket].type.event;

  packet.eventCode = event;

  sendPacket(eventPacketSize);
}

bool sendDataPacket(void* cbData){
  sendPacket(dataPacketSize);

  send = timer.every(6e4, sendDataPacket);

#if defined(DEBUG)
  Serial.printf("Sent packet...\n");
#endif

  return false;
}

void sendPacket(size_t packetLength){
  radio.clearPacketReceivedAction();

  Packet& packet = packetBuffer[currentPacket];

  crypto.encrypt(
    packet.type.encrypted,
    (uint8_t*)&packet.type,
    packetLength - idSize - tagSize
  );

  crypto.computeTag(&packet.tag, tagSize);

  radio.transmit((uint8_t*)&packet, packetLength);

  sentCount++;

  radio.setPacketReceivedAction(onRecieve);
  radio.startReceive();
}

void onRecieve(){
  uint8_t dump;
  size_t  packetSize = radio.getPacketLength(true);

  switch (packetSize) {
  case eventPacketSize:
    break;
  default:

#ifdef DEBUG
  Serial.printf("Invalid packet size of %d recieved.\n", packetSize);
#endif

    radio.readData(&dump, 1);
    return;
  }

  uint8_t encryptedData[sizeof(EventPacket)];
  Packet  packet;

  radio.readData(
    (uint8_t*)&packet, 
    packetSize
  );

  memcpy(encryptedData, packet.type.encrypted, sizeof(EventPacket));

  if (packet.id != MONITOR_ID) return;

  crypto.decrypt(
    (uint8_t*)&packet.type, 
    encryptedData, 
    sizeof(EventPacket)
  );

  ChaChaPoly newCrypt;

  if (!crypto.checkTag(&packet.tag, tagSize)){
    if (!newCrypt.setKey(encryption.key, 16)) throw("Could not set key");
    if (!newCrypt.setIV(encryption.iv, 8))    throw("Could not set IV");

    newCrypt.decrypt(
      (uint8_t*)&packet.type, 
      encryptedData, 
      sizeof(EventPacket)
    );

    if (!newCrypt.checkTag(&packet.tag, tagSize)){

#ifdef DEBUG
  Serial.printf("Event packet tag failed.\n");
#endif
      
      newCrypt.clear();
      return;
    }
  }

#ifdef DEBUG
  Serial.printf("Code %d recieved.\n", packet.type.event.eventCode);
#endif

  switch (packet.type.event.eventCode) {
  case EventCode::DATA_RECVED:

#ifdef DEBUG
  Serial.printf("Gateway ACKED.\n");
#endif

    currentPacket ^= 1;
    timer.cancel(send);
    send = timer.every(3e5, sendDataPacket);
    break;
  case EventCode::NODE_ONLINE:

#ifdef DEBUG
  Serial.printf("Gateway back online.\n");
#endif

    crypto = newCrypt;
    break;
  default:
    return;
  }
}

bool updateDisplay(void* cbData){
  display.clear();

  display.drawString(0, 0, "Sent: " + String(sentCount));

  display.display();

  return true;
}
