/*
Code for the HERBS (Hive Environmental Reporting Broadcast System) Monitor(s)

Author(s): 
  John Schulz (john.schulz1@protonmail.com)
*/

// Comment out for actual use to save energy
// Otherwise it will print useful info over serial
#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.printf(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
#endif 

// HERBS Data Types
#include <include/Herbs.h>

// LoRa constants
#include <include/LoRa.h>

// Pins
#include <include/Pins.h>

// STL Includes
#include <deque> // FIFO optimized lists

// External Includes
#include <Adafruit_BMP3XX.h>
#include <arduino-timer.h>
#include <ChaChaPoly.h>
#include <heltec_unofficial.h>
#include <include/SHT31.hpp>
#include <include/CircularArray.hpp>

#define MINUTE_US  6e7
#define HOUR_US   36e8

#define PACKET_BUFFER_SIZE 2
#define PACKET_SEND_RATE   300e3 // in ms
#define PACKET_RETRY_RATE  300e3 // in ms

#define REDUCED_RATE_MULTIPLE 5

typedef Adafruit_BMP3XX BMP390;

DataPacket latestData;
Packet     packetBuffer[PACKET_BUFFER_SIZE];

size_t currentPacket = 0;

ChaChaPoly baseCrypto = ChaChaPoly();
ChaChaPoly crypto;

uint8_t failedSends = 0;

Timer<8, millis> timer;

Timer<1, millis>::Task resend;

TwoWire externI2C = TwoWire(0x01);

SHT31  sht31   = SHT31(&externI2C);
BMP390 bmp390  = BMP390();

CircularArray<uint16_t, 128> audioSamples;

// Global variables for recieved packets
uint8_t            encryptedEventData;
std::deque<Packet> recievedPackets;

size_t send_rate   = PACKET_SEND_RATE;
size_t resend_rate = PACKET_RETRY_RATE;

void setup() {
  Serial.begin(115200);
  
  if (!initPeripherals()) return initError("Peripheral init failed!");

  checkBatteryLevel();

  // Init encryption
  if (!baseCrypto.setKey(encryption.key, 16))
    return initError("Could not set key!");
  if (!baseCrypto.setIV(encryption.iv, 8))
    return initError("Could not set IV!");

  crypto = ChaChaPoly(baseCrypto);

  // Init functions
  if (!initLoRa())        return initError("LoRa init failed!");

  timer.in(send_rate, sendDataPacket);

  // Sensor read callbacks
  timer.every(10e3, updateFromI2C);
  
  turnOnSoundSensor(nullptr);

  #ifdef DEBUG
  timer.every(10e3, printData);
  #endif

  // Light sleep configuration
  esp_sleep_enable_ulp_wakeup();
  esp_sleep_enable_timer_wakeup(50e3);

  sendEventPacket(EventCode::NODE_ONLINE);

  radio.sleep();
}

void loop() {
  checkBatteryLevel();
  processRecievedPackets();

  timer.tick();

  esp_light_sleep_start();
}

bool initLoRa() {
  bool res = radio.begin(
    LORA_FREQUENCY_US,
    LORA_BANDWIDTH_125,
    LORA_SPREADING_FACTOR_9,
    LORA_CODING_RATE_4_8,
    0x12,
    22
  );

  return res == 0;
}

bool initPeripherals() {
  heltec_setup();

  pinMode(USER_LED, OUTPUT);

  // Power
  pinMode(SOUND_VCC, OUTPUT);
  pinMode(I2C_VCC,   OUTPUT);
  digitalWrite(SOUND_VCC, LOW);
  digitalWrite(I2C_VCC,   HIGH);

  pinMode(SOUND_ADC, INPUT);
  analogRead(SOUND_ADC);

  externI2C.begin(I2C_SDA, I2C_SCL, 100000);
  externI2C.setBufferSize(10);
  
  sht31.begin();

  bmp390.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &externI2C);
  bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp390.setPressureOversampling(BMP3_IIR_FILTER_COEFF_63);

  return true;
}

bool updateFromI2C(void* cbData){
  sht31.readBoth(
    latestData.hive_temp, 
    latestData.humidity
  );

  bmp390.performReading();
  
  // + 0.5 for rounding with integer conversion
  latestData.extern_temp = bmp390.temperature   + 0.5;
  latestData.pressure    = bmp390.pressure /100 + 0.5; // Pa to mbar

  return true;
}

bool updateSound(void* cbData) {
  latestData.acoustics = 0;
  
  audioSamples.push(analogRead(SOUND_ADC));
  for (uint8_t index = 1; index < audioSamples.capacity(); ++index){
    esp_light_sleep_start();
    audioSamples.push(analogRead(SOUND_ADC));
    latestData.acoustics += abs(audioSamples[index - 1] - audioSamples[index]);
  }
  latestData.acoustics /= audioSamples.capacity();

  digitalWrite(SOUND_VCC, LOW);

  timer.in(3599500, turnOnSoundSensor);

  return false;
}

void checkBatteryLevel() {
  latestData.battery = heltec_battery_percent();

  switch (latestData.battery) {
  case 0 ... 14:
    radio.sleep();

    digitalWrite(I2C_VCC,   LOW);
    digitalWrite(SOUND_VCC, LOW);

    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RC_FAST,    ESP_PD_OPTION_OFF);
    esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL,       ESP_PD_OPTION_OFF);

    esp_deep_sleep(HOUR_US);

    throw "*** BATTERY CRITICALLY LOW ***";
  case 15 ... 30:
    send_rate   = PACKET_SEND_RATE  * REDUCED_RATE_MULTIPLE;
    resend_rate = PACKET_RETRY_RATE * REDUCED_RATE_MULTIPLE;

    return;
  default:
    send_rate   = PACKET_SEND_RATE;
    resend_rate = PACKET_RETRY_RATE;
    
    return;
  }
}

bool printData(void* cbData) {
  Serial.printf("Battery percent is %u\n",     latestData.battery);
  Serial.printf("Hive temperature is %i\n",    latestData.hive_temp);
  Serial.printf("Outside temperature is %i\n", latestData.extern_temp);
  Serial.printf("Humidity is %u\n",            latestData.humidity);
  Serial.printf("Pressure is %u\n",            latestData.pressure);
  Serial.printf("Sound level is %u\n",         latestData.acoustics);

  return true;
}

void sendEventPacket(EventCode event) {
  Packet packet;

  packet.event.eventCode = event;

  encryptPacket(packet, sizeof(EventPacket));
  sendPacket(packet, eventPacketSize);
}

bool sendDataPacket(void* cbData) {
  memcpy(&packetBuffer[currentPacket].data, &latestData, sizeof(DataPacket));

  encryptPacket(packetBuffer[currentPacket], sizeof(DataPacket));
  sendPacket(packetBuffer[currentPacket], dataPacketSize);

  resend = timer.every(resend_rate, resendDataPacket);

  DEBUG_PRINT("Packet sent.\n");

  return false;
}

bool resendDataPacket(void* cbData) {
  if (++failedSends == 10){
    radio.sleep();
    esp_deep_sleep(HOUR_US);
    throw "*** NO RESPONSE FROM GATEWAY ***";
  }

  sendPacket(packetBuffer[currentPacket], dataPacketSize);

  DEBUG_PRINT("Packet re-sent.\n");

  return true;
}

void sendPacket(Packet& packet, size_t packetLength) {
  radio.standby();
  
  digitalWrite(USER_LED, HIGH);
  timer.in(300, turnOffLED);

  radio.clearPacketReceivedAction();

  radio.transmit((uint8_t*)&packet, packetLength);

  radio.setPacketReceivedAction(onRecieve);
  radio.startReceive();
}

void encryptPacket(Packet& packet, size_t dataLength){
  crypto.encrypt(
    packet.raw,
    (uint8_t*)&packet.raw,
    dataLength
  );

  crypto.computeTag(&packet.tag, tagSize);
}

void onRecieve() {
  switch (radio.getPacketLength(true)) {
  case eventPacketSize:
    break;
  default:
    radio.readData(&encryptedEventData, 1);
    encryptedEventData = 0;
    return;
  }

  recievedPackets.emplace_back();

  radio.readData((uint8_t*)&recievedPackets.back(), eventPacketSize);

  if (recievedPackets.back().id != MONITOR_ID) return recievedPackets.pop_back();
}

void processRecievedPackets(){
  while (recievedPackets.size()) {
    DEBUG_PRINT("Processing new packet...\n");

    Packet* packet = &recievedPackets.front();

    encryptedEventData = packet->raw[0];

    crypto.decrypt(
      packet->raw,
      &encryptedEventData,
      sizeof(EventPacket)
    );

    ChaChaPoly newCrypt;

    if (!crypto.checkTag(&packet->tag, tagSize)) {
      DEBUG_PRINT("Invalid tag; trying with reset counter.\n");

      newCrypt = ChaChaPoly(baseCrypto);

      newCrypt.decrypt(
        packet->raw,
        &encryptedEventData,
        sizeof(EventPacket)
      );

      if (!newCrypt.checkTag(&packet->tag, tagSize)) {
        newCrypt.clear();
        DEBUG_PRINT("Invalid tag with reset counter; ignoring.\n");
        return;
      }
    }

    switch (packet->event.eventCode) {
      case EventCode::DATA_RECVED:
        radio.sleep();

        timer.cancel(resend);
        timer.in(send_rate, sendDataPacket);

        currentPacket ^= 1;
        failedSends = 0;
      
        break;
      case EventCode::NODE_ONLINE:
        radio.sleep();

        timer.cancel(resend);
        timer.in(send_rate, sendDataPacket);

        crypto = newCrypt;

        break;
      default:
        return;
    }

    recievedPackets.pop_front();
  }
}

bool turnOnSoundSensor(void* cbData){
  digitalWrite(SOUND_VCC, HIGH);

  timer.in(500, updateSound);

  return false;
}

bool turnOffLED(void* cbData){
  digitalWrite(USER_LED, LOW);

  return false;
}

void initError(const char* err){
  Serial.println(err);

  delay(1000);

  esp_deep_sleep(HOUR_US);

  throw "*** INITIALIZATION ERROR ***";
}