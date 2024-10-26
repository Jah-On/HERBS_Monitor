/*
Code for the HERBS (Hive Environmental Reporting Broadcast System) Monitor(s)

Author(s): 
  John Schulz (john.schulz1@protonmail.com)
*/

// Comment out for actual use to save energy
// Otherwise it will print useful info over serial
// #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
#endif 

// HERBS Data Types
#include <include/Herbs.h>

// Include keys
#include <include/Secrets.h>

// LoRa constants
#include <include/LoRa.h>

// Pins
#include <include/Pins.h>

// STL Includes
#include <list>    // Front/Back optimized lists

// External Includes
#include <Adafruit_BMP3XX.h>
#include <arduino-timer.h>
#include <ChaChaPoly.h>
#include <heltec_unofficial.h>
#include <include/SHT31.hpp>

#define PACKET_BUFFER_SIZE 2

typedef Adafruit_BMP3XX BMP390;

Packet packetBuffer[PACKET_BUFFER_SIZE];

size_t currentPacket = 0;

ChaChaPoly crypto = ChaChaPoly();

size_t sentCount = 0;

Timer<6, millis> timer;

Timer<1, millis>::Task send;

TwoWire externI2C = TwoWire(0x01);

SHT31  sht31   = SHT31(&externI2C);
BMP390 bmp390  = BMP390();

uint16_t audioBuffer[32] = {0};
uint8_t  audioBufferSize = 0;

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);
  #endif

  for (Packet& packet : packetBuffer) packet.id = MONITOR_ID;

  heltec_setup();

  // Setup display
  display.setFont(ArialMT_Plain_16);
  display.clear();

  // Init encryption
  if (!crypto.setKey(encryption.key, 16)) return displayError("Could not set key!");
  if (!crypto.setIV(encryption.iv, 8))    return displayError("Could not set IV!");

  // Init functions
  if (!initLoRa())        return displayError("LoRa init failed!");
  if (!initPeripherals()) return displayError("Peripheral init \nfailed!");

  display.displayOff(); // If no errors then turn off the display

  radio.setPacketReceivedAction(onRecieve);
  radio.startReceive();

  send = timer.every(60e3, sendDataPacket);

  // Sensor read callbacks
  timer.every(1000, updateFromSHT31);
  timer.every(1000, updateFromBMP390);
  timer.every( 950, updateSound);

  #ifdef DEBUG
  timer.every(10e3, printData);
  #endif

  delay(500);

  sendEventPacket(EventCode::NODE_ONLINE);
}

void loop() {
  timer.tick();
  heltec_delay(1);
}

bool initLoRa() {
  bool res = radio.begin(
    LORA_FREQUENCY_US,
    LORA_BANDWIDTH_20_8,
    LORA_SPREADING_FACTOR_5,
    LORA_CODING_RATE_4_5,
    0x12,
    22
  );

  return res == 0;
}

bool initPeripherals() {
  externI2C.begin(I2C_SDA, I2C_SCL, 115200);
  externI2C.setBufferSize(10);
  
  sht31.begin();

  bmp390.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &externI2C);
  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp390.setPressureOversampling(BMP3_IIR_FILTER_COEFF_63);

  return true;
}

bool updateFromSHT31(void* cbData) {
  DataPacket& data = packetBuffer[currentPacket].type.data;

  sht31.readBoth(data.hive_temp, data.humidity);

  return true;
}

bool updateFromBMP390(void* cbData) {
  DataPacket& data = packetBuffer[currentPacket].type.data;

  bmp390.performReading();
  
  data.extern_temp = bmp390.temperature   + 0.5; // Round temperature
  data.pressure    = bmp390.pressure /100 + 0.5; // Pa to mbar and rounding

  return true;
}

bool updateSound(void* cbData) {
  DataPacket& data = packetBuffer[currentPacket].type.data;
  data.acoustics = 0;

  digitalWrite(SOUND_VCC, HIGH);
  delay(49);
  
  switch (audioBufferSize){
  case (31):
    memmove(&audioBuffer[0], &audioBuffer[1], 31*sizeof(uint16_t));
    break;
  default:
    audioBufferSize++;
    break;
  }

  audioBuffer[audioBufferSize - 1] = analogRead(SOUND_ADC);
  for (uint8_t index = 1; index < audioBufferSize; ++index){
    data.acoustics += abs(audioBuffer[index - 1] - audioBuffer[index]);
  }
  data.acoustics /= audioBufferSize + 1;

  digitalWrite(SOUND_VCC, LOW);

  return true;
}

bool printData(void* cbData) {
  DataPacket& data = packetBuffer[currentPacket].type.data;

  Serial.printf("Hive temperature is %i\n",    data.hive_temp);
  Serial.printf("Outside temperature is %i\n", data.extern_temp);
  Serial.printf("Humidity is %u\n",            data.humidity);
  Serial.printf("Pressure is %u\n",            data.pressure);
  Serial.printf("Sound level is %u\n",         data.acoustics);

  return true;
}

void sendEventPacket(EventCode event) {
  Packet packet;

  packet.id                   = MONITOR_ID;
  packet.type.event.eventCode = event;

  sendPacket(packet, eventPacketSize);
}

bool sendDataPacket(void* cbData) {
  sendPacket(packetBuffer[currentPacket], dataPacketSize);

  send = timer.every(1e3, sendDataPacket);

  DEBUG_PRINT("Packet sent.");

  return false;
}

void sendPacket(Packet& packet, size_t packetLength) {
  radio.clearPacketReceivedAction();

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

void onRecieve() {
  uint8_t dump;
  size_t packetSize = radio.getPacketLength(true);

  if (packetSize != eventPacketSize) {
    radio.readData(&dump, 1);
    return;
  }

  uint8_t encryptedData[sizeof(EventPacket)];
  Packet packet;

  radio.readData((uint8_t*)&packet, packetSize);

  memcpy(encryptedData, packet.type.encrypted, sizeof(EventPacket));

  if (packet.id != MONITOR_ID) return;

  crypto.decrypt(
    (uint8_t*)&packet.type,
    encryptedData,
    sizeof(EventPacket)
  );

  ChaChaPoly newCrypt;

  if (!crypto.checkTag(&packet.tag, tagSize)) {
    if (!newCrypt.setKey(encryption.key, 16)) throw("Could not set key");
    if (!newCrypt.setIV(encryption.iv, 8))    throw("Could not set IV");

    newCrypt.decrypt(
      (uint8_t*)&packet.type,
      encryptedData,
      sizeof(EventPacket)
    );

    if (!newCrypt.checkTag(&packet.tag, tagSize)) {
      DEBUG_PRINT("Event packet tag failed.\n");

      newCrypt.clear();
      return;
    }
  }

  switch (packet.type.event.eventCode) {
    case EventCode::DATA_RECVED:
      DEBUG_PRINT("Gateway acknowledged.");

      memset(
        &packetBuffer[currentPacket].type.encrypted, 
        0, 
        sizeof(DataPacket)
      );

      currentPacket ^= 1;

      timer.cancel(send);
      send = timer.every(10e3, sendDataPacket);
    
      break;
    case EventCode::NODE_ONLINE:
      DEBUG_PRINT("Gateway back online.");

      crypto = newCrypt;

      break;
    default:
      return;
  }
}

void displayError(const char* err){
  display.println(err);
  display.display();
}