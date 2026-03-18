#include <Arduino.h>

#include <ADS1256.h>

#include <struct.h>
#include <enums.h>


#if defined(ARDUINO_ARCH_RP2040)
  #if defined(USE_SPI1)
    #define SPI_MOSI 11
    #define SPI_MISO 12
    #define SPI_SCK  10
    #define USE_SPI  SPI1
    ADS1256 A(18, 20, 21, 19, 2.500, &USE_SPI);  // RP2040 Zero - SPI1
  #else
    #define SPI_MOSI 3
    #define SPI_MISO 4
    #define SPI_SCK  2
    #define USE_SPI  SPI
    ADS1256 A(7, ADS1256::PIN_UNUSED, 6, 5, 2.500, &USE_SPI);  // RP2040 Waveshare Mini - SPI0
  #endif

#elif defined(ARDUINO_ARCH_STM32)
  #if defined(USE_SPI2)
    #define USE_SPI spi2
    SPIClass spi2(PB15, PB14, PB13);
    ADS1256 A(PB10, PB11, ADS1256::PIN_UNUSED, PB12, 2.500, &USE_SPI);  // STM32 SPI2
  #else
    #define USE_SPI SPI
    ADS1256 A(PA2, ADS1256::PIN_UNUSED, ADS1256::PIN_UNUSED, PA4, 2.500, &USE_SPI);  // STM32 SPI1
  #endif

#elif defined(TEENSYDUINO)
  #if defined(USE_SPI2)
    #define USE_SPI SPI2
  #elif defined(USE_SPI1)
    #define USE_SPI SPI1
  #else
    #define USE_SPI SPI
  #endif
  ADS1256 A(7, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI);  // Teensy 4.0 / 4.1

#elif defined(ARDUINO_ARCH_ESP32)
  SPIClass hspi(HSPI);
  #if defined(USE_HSPI)
    #define USE_SPI hspi
    // begin() called in setup() — see below
  #else
    #define USE_SPI SPI
  #endif
  ADS1256 A(16, 17, ADS1256::PIN_UNUSED, 15, 2.500, &USE_SPI);  // ESP32 WROOM
  #ifndef LED_BUILTIN
    #define LED_BUILTIN 2   // most ESP32 boards have the onboard LED on GPIO2
  #endif

#else  // AVR fallback
  #define SPI_MOSI MOSI
  #define SPI_MISO MISO
  #define SPI_SCK  SCK
  #define USE_SPI  SPI
  ADS1256 A(2, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI);  // Uno / Nano
#endif

// default settings
#define SAMPLING_SPEED 100  // in Hz
#define TIMEOUT_DURATION 1000 // in milliseconds

unsigned long lastSampleTime = 0;
unsigned long interval = 1000000 / SAMPLING_SPEED; // 1_000_000us / 100Hz = 10ms

unsigned long lastHeartbeat = 0;

SystemState currentState = SystemState::STOP;

uint8_t PGASettings[7] = {
  PGA_1,
  PGA_2,
  PGA_4,
  PGA_8,
  PGA_16,
  PGA_32,
  PGA_64
};

uint8_t DataRateSettings[16] = {
  DRATE_2SPS,
  DRATE_5SPS,
  DRATE_10SPS,
  DRATE_15SPS,
  DRATE_25SPS,
  DRATE_30SPS,
  DRATE_50SPS,
  DRATE_60SPS,
  DRATE_100SPS,
  DRATE_500SPS,
  DRATE_1000SPS,
  DRATE_2000SPS,
  DRATE_3750SPS,
  DRATE_7500SPS,
  DRATE_15000SPS,
  DRATE_30000SPS
};


void initADC(SettingsPacket *s);
void getSettings();
void validateSettings(SettingsPacket *s);
void sendPacket();


void setup() {
  Serial.begin(250000);  //The value does not matter if you use an MCU with native USB

#if defined(ARDUINO_ARCH_RP2040)  //If RP2040 is used, we need to pass the SPI pins
  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
#endif

#if defined(USE_HSPI)      //If ESP32 is used, we need to start SPI with a non-strapping MISO pin
  hspi.begin(14, 25, 13);  //SCK, MISO (safe), MOSI
#endif

  getSettings();
}

void loop() {
  // Check for Heartbeat
  if (Serial.available() > 0) {
    while(Serial.available()) Serial.read(); // Clear buffer
    lastHeartbeat = millis();

    if (currentState == SystemState::STOP) {
      lastSampleTime = micros();
      currentState = SystemState::STREAMING;
    }
  }

  // Check for Timeout
  if (millis() - lastHeartbeat > TIMEOUT_DURATION && currentState != SystemState::STOP) {
    currentState = SystemState::STOP;
    A.stopConversion();
  }

  // Sampling and Streaming
  if (currentState == SystemState::STREAMING) {
    unsigned long currentTime = micros();
    // Use >= to handle potential slight jitter
    if (currentTime - lastSampleTime >= interval) {
      sendPacket();
    }
  } else {
    // We are in STOP. Maybe blink an LED to show "Waiting for PC" ?
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2); 
  }
}

void initADC(SettingsPacket *s) {
  A.InitializeADC();  //See the documentation for every details

  // Set PGA
  A.setPGA(PGASettings[s->ADCGain]);
  //Set DRATE
  A.setDRATE(DataRateSettings[s->ADCDataRate]);
}

void getSettings() {
  bool settingsReceived = false;
  SettingsPacket incomingSettings;

  while (!settingsReceived) {
    if (Serial.available() >= sizeof(SettingsPacket)) {
      if (Serial.peek() == 0xCC) {
        Serial.readBytes((uint8_t*)&incomingSettings, sizeof(SettingsPacket));

        if (incomingSettings.header1 == 0xCC && incomingSettings.header2 == 0xDD) {
          // Validate and apply settings
          validateSettings(&incomingSettings);
          // Respond with the same structure for verification
          Serial.write((uint8_t*)&incomingSettings, sizeof(incomingSettings));
          Serial.flush();
          
          settingsReceived = true;
        }
      } else {
        Serial.read(); // Discard garbage
      }
    }
  }
}

void validateSettings(SettingsPacket *s){
  if (s->samplingSpeed > 0) {
    interval = 1000000UL / s->samplingSpeed; 
  }

  // Update ADC Gain Index (Validate range 0-6)
  if (s->ADCGain > 6) {
    s->ADCGain = 6; // Default to index 6 (PGA_64)
  }

  // Update Data Rate Index (Validate range 0-15)
  if (s->ADCDataRate > 15) {
    s->ADCDataRate = 11; // Default to index 14 (2000SPS)
  }

  initADC(s);
}

void sendPacket() {
  lastSampleTime += interval; // Schedule next sample time based on the previous one to maintain consistent intervals, even if there is some processing delay

  ADC_Packet frame;

  frame.header1 = 0xAA;
  frame.header2 = 0xBB;
  
  frame.ch0 = A.cycleDifferential();
  frame.ch1 = A.cycleDifferential();
  frame.ch2 = A.cycleDifferential();
  A.cycleDifferential(); // we don't need the last channel but we need to call it to update the MUX for the next cycle

  // Calculate Checksum
  uint8_t* ptr = (uint8_t*)&frame;
  uint8_t chk = 0;

  // XOR everything except the last byte (the checksum itself)
  for(size_t i=0; i < sizeof(ADC_Packet) - 1; i++) {
      chk ^= ptr[i];
  }
  frame.checksum = chk;

  Serial.write((uint8_t*)&frame, sizeof(frame));  // Send the entire frame as binary data
  Serial.flush(); // Ensure all data is sent before proceeding
}