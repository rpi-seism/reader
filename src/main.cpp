#include <Arduino.h>

#include <ADS1256.h>

#include <struct.h>
#include <enums.h>

#if defined(ARDUINO_ARCH_RP2040)
#if defined(USE_SPI1)
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 10
#define USE_SPI SPI1
ADS1256 A(18, 20, 21, 19, 2.500, &USE_SPI); // RP2040 Zero - SPI1
#else
#define SPI_MOSI 3
#define SPI_MISO 4
#define SPI_SCK 2
#define USE_SPI SPI
ADS1256 A(7, ADS1256::PIN_UNUSED, 6, 5, 2.500, &USE_SPI); // RP2040 Waveshare Mini - SPI0
#endif

#elif defined(ARDUINO_ARCH_STM32)
#if defined(USE_SPI2)
#define USE_SPI spi2
SPIClass spi2(PB15, PB14, PB13);
ADS1256 A(PB10, PB11, ADS1256::PIN_UNUSED, PB12, 2.500, &USE_SPI); // STM32 SPI2
#else
#define USE_SPI SPI
ADS1256 A(PA2, ADS1256::PIN_UNUSED, ADS1256::PIN_UNUSED, PA4, 2.500, &USE_SPI); // STM32 SPI1
#endif

#elif defined(TEENSYDUINO)
#if defined(USE_SPI2)
#define USE_SPI SPI2
#elif defined(USE_SPI1)
#define USE_SPI SPI1
#else
#define USE_SPI SPI
#endif
ADS1256 A(7, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI); // Teensy 4.0 / 4.1

#elif defined(ARDUINO_ARCH_ESP32)
SPIClass hspi(HSPI);
#if defined(USE_HSPI)
#define USE_SPI hspi
// begin() called in setup() — see below
#else
#define USE_SPI SPI
#endif
ADS1256 A(16, 17, ADS1256::PIN_UNUSED, 15, 2.500, &USE_SPI); // ESP32 WROOM
#ifndef LED_BUILTIN
#define LED_BUILTIN 2 // most ESP32 boards have the onboard LED on GPIO2
#endif

#else // AVR fallback
#define SPI_MOSI MOSI
#define SPI_MISO MISO
#define SPI_SCK SCK
#define USE_SPI SPI
ADS1256 A(2, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI); // Uno / Nano
#endif

// default settings
#define SAMPLING_SPEED 100 // in Hz
#define SAMPLES_PER_PACKET 5
#define TIMEOUT_DURATION 1000 // in milliseconds
#define WRITE_TIMEOUT 500     // ms to detect a "stuck" serial port
// #define DEBUG_MODE

unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 1000000 / SAMPLING_SPEED;

int32_t buf_ehz[SAMPLES_PER_PACKET];
int32_t buf_ehn[SAMPLES_PER_PACKET];
int32_t buf_ehe[SAMPLES_PER_PACKET];
uint8_t sample_index = 0;

SystemState currentState = SystemState::STOP;
static unsigned long bufferFullStartTime;
static bool isTimerActive = false;

void initADC();
void sendPacket();
uint8_t get_checksum(int32_t *array, uint32_t size);

void setup()
{
  Serial.begin(115200); // The value does not matter if you use an MCU with native USB

#if defined(ARDUINO_ARCH_RP2040) // If RP2040 is used, we need to pass the SPI pins
  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
#endif

#if defined(USE_HSPI)     // If ESP32 is used, we need to start SPI with a non-strapping MISO pin
  hspi.begin(14, 25, 13); // SCK, MISO (safe), MOSI
#endif

#ifndef DEBUG_MODE
  initADC();
#endif
  // Respond with the same structure for verification

  currentState = SystemState::STREAMING;
}

void loop()
{
  if (Serial.availableForWrite() < 32)
  {
    if (!isTimerActive)
    {
      bufferFullStartTime = millis();
      isTimerActive = true;
    }

    if (millis() - bufferFullStartTime > WRITE_TIMEOUT)
    {
      if (currentState != SystemState::STOP)
      {
        currentState = SystemState::STOP;
        A.stopConversion();
      }
    }
  }
  else
  {
    isTimerActive = false; // Reset the flag
  }

  // Sampling and Streaming
  if (currentState == SystemState::STREAMING)
  {
    unsigned long currentTime = micros();
    if (currentTime - lastSampleTime >= sampleInterval)
    {
      lastSampleTime += sampleInterval;

      int32_t val_z, val_n, val_e;
#ifndef DEBUG_MODE
      val_z = A.cycleDifferential();
      val_n = A.cycleDifferential();
      val_e = A.cycleDifferential();
      A.cycleDifferential();
#else
      val_z = random(-10, 10);
      val_n = random(-10, 10);
      val_e = random(-10, 10);
#endif

      buf_ehz[sample_index] = val_z;
      buf_ehn[sample_index] = val_n;
      buf_ehe[sample_index] = val_e;
      sample_index++;

      if (sample_index >= SAMPLES_PER_PACKET)
      {
        sample_index = 0;
        sendPacket();
      }
    }
  }
  else
  {
    // We are in STOP. Maybe blink an LED to show "Waiting for PC" ?
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);
  }
}

void initADC()
{
  A.InitializeADC();
  A.setPGA(PGA_64);
  A.setDRATE(DRATE_2000SPS);
}

uint8_t get_checksum(int32_t *array, uint32_t size)
{
  uint8_t checksum = 0;
  uint8_t *bytes;

  for (uint8_t i = 0; i < size; i++)
  {
    bytes = (uint8_t *)&array[i];
    for (uint8_t j = 0; j < sizeof(int32_t); j++)
    {
      checksum ^= bytes[j];
    }
  }

  return checksum;
}

void sendPacket()
{
  ADC_Packet frame;
  frame.header1 = 0xFC;
  frame.header2 = 0x1B;

  for (uint8_t i = 0; i < SAMPLES_PER_PACKET; i++)
  {
    frame.channel_n[i] = buf_ehn[i];
    frame.channel_e[i] = buf_ehe[i];
    frame.channel_z[i] = buf_ehz[i];
  }

  frame.checksum[0] = get_checksum(buf_ehn, SAMPLES_PER_PACKET);
  frame.checksum[1] = get_checksum(buf_ehe, SAMPLES_PER_PACKET);
  frame.checksum[2] = get_checksum(buf_ehz, SAMPLES_PER_PACKET);

  frame.padding = 0;

  Serial.write((uint8_t *)&frame, sizeof(frame));
}