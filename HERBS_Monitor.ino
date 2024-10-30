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
#include <queue>    // FILO optimized lists

// ESP32 Includes
#include <esp_bt.h>

// External Includes
#include <Adafruit_BMP3XX.h>
#include <arduino-timer.h>
#include <ChaChaPoly.h>
#include <heltec_unofficial.h>
#include <include/SHT31.hpp>
#include <include/CircularArray.hpp>

#define PACKET_BUFFER_SIZE 2
#define PACKET_SEND_RATE   60000 // in ms
#define PACKET_RETRY_RATE  60000 // in ms

typedef Adafruit_BMP3XX BMP390;

DataPacket latestData;
Packet     packetBuffer[PACKET_BUFFER_SIZE];

size_t currentPacket = 0;

ChaChaPoly baseCrypto = ChaChaPoly();
ChaChaPoly crypto;

uint8_t failedSends = 0;

Timer<6, millis> timer;

Timer<1, millis>::Task resend;

TwoWire externI2C = TwoWire(0x01);

SHT31  sht31   = SHT31(&externI2C);
BMP390 bmp390  = BMP390();

CircularArray<uint16_t, 32> audioSamples;

// Global variables for recieve callback
uint8_t  encryptedEventData[sizeof(EventPacket)];
Packet   rcvdPacket;

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
  if (!baseCrypto.setKey(encryption.key, 16))
    return displayError("Could not set key!");
  if (!baseCrypto.setIV(encryption.iv, 8))
    return displayError("Could not set IV!");

  crypto = ChaChaPoly(baseCrypto);

  // Init functions
  if (!initLoRa())        return displayError("LoRa init failed!");
  if (!initPeripherals()) return displayError("Peripheral init \nfailed!");

  // If no errors then turn off the display 
  display.displayOff();

  timer.in(PACKET_SEND_RATE, sendDataPacket);

  // Sensor read callbacks
  timer.every(5000, updateFromSHT31);
  timer.every(5000, updateFromBMP390);
  timer.every(4950, updateSound);

  #ifdef DEBUG
  timer.every(10e3, printData);
  #endif

  esp_sleep_enable_ulp_wakeup();
  esp_sleep_enable_timer_wakeup(50000);

  sendEventPacket(EventCode::NODE_ONLINE);

  radio.sleep();
}

void loop() {
  timer.tick();
  esp_light_sleep_start();
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
  sht31.readBoth(
    latestData.hive_temp, 
    latestData.humidity
  );

  return true;
}

bool updateFromBMP390(void* cbData) {
  bmp390.performReading();
  
  // + 0.5 for rounding with integer conversion
  latestData.extern_temp = bmp390.temperature   + 0.5;
  latestData.pressure    = bmp390.pressure /100 + 0.5; // Pa to mbar

  return true;
}

bool updateSound(void* cbData) {
  latestData.acoustics = 0;

  digitalWrite(SOUND_VCC, HIGH);
  delay(49);
  
  audioSamples.push(analogRead(SOUND_ADC));
  for (uint8_t index = 1; index < audioSamples.size(); ++index){
    latestData.acoustics += abs(audioSamples[index - 1] - audioSamples[index]);
  }
  latestData.acoustics /= audioSamples.size();

  digitalWrite(SOUND_VCC, LOW);

  return true;
}

bool printData(void* cbData) {
  Serial.printf("Hive temperature is %i\n",    latestData.hive_temp);
  Serial.printf("Outside temperature is %i\n", latestData.extern_temp);
  Serial.printf("Humidity is %u\n",            latestData.humidity);
  Serial.printf("Pressure is %u\n",            latestData.pressure);
  Serial.printf("Sound level is %u\n",         latestData.acoustics);

  return true;
}

void sendEventPacket(EventCode event) {
  Packet packet;

  packet.id              = MONITOR_ID;
  packet.event.eventCode = event;

  encryptPacket(packet, eventPacketSize);
  sendPacket(packet, eventPacketSize);
}

bool sendDataPacket(void* cbData) {
  memcpy(&packetBuffer[currentPacket].data, &latestData, sizeof(DataPacket));

  encryptPacket(packetBuffer[currentPacket], dataPacketSize);
  sendPacket(packetBuffer[currentPacket], dataPacketSize);

  resend = timer.every(PACKET_RETRY_RATE, resendDataPacket);

  DEBUG_PRINT("Packet sent.");

  return false;
}

bool resendDataPacket(void* cbData) {
  if (++failedSends == 10){
    radio.sleep();
    esp_deep_sleep(24u * 3600u * 1000u * 1000u);
    throw("Restarting...");
  }

  sendPacket(packetBuffer[currentPacket], dataPacketSize);

  DEBUG_PRINT("Packet re-sent.");

  return true;
}

void sendPacket(Packet& packet, size_t packetLength) {
  radio.clearPacketReceivedAction();

  radio.transmit((uint8_t*)&packet, packetLength);

  radio.setPacketReceivedAction(onRecieve);
  radio.startReceive();
}

void encryptPacket(Packet& packet, size_t packetLength){
  crypto.encrypt(
    packet.raw,
    (uint8_t*)&packet.raw,
    packetLength - idSize - tagSize
  );

  crypto.computeTag(&packet.tag, tagSize);
}

void onRecieve() {
  DEBUG_PRINT("In interrupt!");

  switch (radio.getPacketLength(true)) {
  case eventPacketSize:
    break;
  default:
    radio.readData(encryptedEventData, 1);
    return;
  }

  DEBUG_PRINT("Recieved packet size good.");

  radio.readData((uint8_t*)&rcvdPacket, eventPacketSize);

  if (rcvdPacket.id != MONITOR_ID) return;

  DEBUG_PRINT("Recieved packet is for node.");

  memcpy(encryptedEventData, rcvdPacket.raw, sizeof(EventPacket));

  crypto.decrypt(
    (uint8_t*)&rcvdPacket.event,
    encryptedEventData,
    sizeof(EventPacket)
  );

  ChaChaPoly newCrypt;

  if (!crypto.checkTag(&rcvdPacket.tag, tagSize)) {
    newCrypt = ChaChaPoly(baseCrypto);

    newCrypt.decrypt(
      (uint8_t*)&rcvdPacket.event,
      encryptedEventData,
      sizeof(EventPacket)
    );

    if (!newCrypt.checkTag(&rcvdPacket.tag, tagSize)) {
      DEBUG_PRINT("Event packet tag failed.\n");

      newCrypt.clear();
      return;
    }
  }

  switch (rcvdPacket.event.eventCode) {
    case EventCode::DATA_RECVED:
      DEBUG_PRINT("Gateway acknowledged.");

      timer.cancel(resend);
      timer.in(PACKET_SEND_RATE, sendDataPacket);

      currentPacket ^= 1;
      failedSends = 0;
    
      break;
    case EventCode::NODE_ONLINE:
      DEBUG_PRINT("Gateway back online.");

      crypto = newCrypt;

      break;
    default:
      return;
  }

  radio.sleep();
}

void displayError(const char* err){
  display.println(err);
  display.display();
}