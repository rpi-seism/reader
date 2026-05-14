#include <Arduino.h>
#include <EEPROM.h>

#include <ADS1256.h>

#include <struct.h>

#if defined(ARDUINO_ARCH_RP2040)
#if defined(USE_SPI1)
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 10
#define USE_SPI SPI1
ADS1256 A(18, 20, 21, 19, 2.500, &USE_SPI);
#else
#define SPI_MOSI 3
#define SPI_MISO 4
#define SPI_SCK 2
#define USE_SPI SPI
ADS1256 A(7, ADS1256::PIN_UNUSED, 6, 5, 2.500, &USE_SPI);
#endif

#elif defined(ARDUINO_ARCH_STM32)
#if defined(USE_SPI2)
#define USE_SPI spi2
SPIClass spi2(PB15, PB14, PB13);
ADS1256 A(PB10, PB11, ADS1256::PIN_UNUSED, PB12, 2.500, &USE_SPI);
#else
#define USE_SPI SPI
ADS1256 A(PA2, ADS1256::PIN_UNUSED, ADS1256::PIN_UNUSED, PA4, 2.500, &USE_SPI);
#endif

#elif defined(TEENSYDUINO)
#if defined(USE_SPI2)
#define USE_SPI SPI2
#elif defined(USE_SPI1)
#define USE_SPI SPI1
#else
#define USE_SPI SPI
#endif
ADS1256 A(7, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI);

#elif defined(ARDUINO_ARCH_ESP32)
SPIClass hspi(HSPI);
#if defined(USE_HSPI)
#define USE_SPI hspi
#else
#define USE_SPI SPI
#endif
ADS1256 A(16, 17, ADS1256::PIN_UNUSED, 15, 2.500, &USE_SPI);
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

#else
#define SPI_MOSI MOSI
#define SPI_MISO MISO
#define SPI_SCK SCK
#define USE_SPI SPI
ADS1256 A(2, ADS1256::PIN_UNUSED, 8, 10, 2.500, &USE_SPI);
#endif

unsigned long lastSampleTime = 0;
unsigned long interval = 1000000 / DEFAULT_SAMPLE_RATE;

SampleBuffer buf;

uint8_t pgaSettings[7] = {
    PGA_1, PGA_2, PGA_4, PGA_8,
    PGA_16, PGA_32, PGA_64};

uint8_t dataRateSettings[16] = {
    DRATE_2SPS, DRATE_5SPS, DRATE_10SPS, DRATE_15SPS,
    DRATE_25SPS, DRATE_30SPS, DRATE_50SPS, DRATE_60SPS,
    DRATE_100SPS, DRATE_500SPS, DRATE_1000SPS, DRATE_2000SPS,
    DRATE_3750SPS, DRATE_7500SPS, DRATE_15000SPS, DRATE_30000SPS};

static uint32_t build_device_config()
{
  return (CONFIG_PACKET_INTERVAL_100MS << 30) |
         (CONFIG_SAMPLE_RATE_100HZ << 27) |
         (CONFIG_GNSS_NOT_AVAILABLE << 26) |
         (CONFIG_CHANNEL_INT32 << 24) |
         (CONFIG_CHANNEL_INT32 << 22) |
         (CONFIG_CHANNEL_INT32 << 20) |
         (CONFIG_CHANNEL_DISABLED << 18) |
         (CONFIG_CHANNEL_DISABLED << 16) |
         (CONFIG_CHANNEL_DISABLED << 14) |
         (CONFIG_CHANNEL_DISABLED << 12) |
         (CONFIG_CHANNEL_DISABLED << 10) |
         (1 << 4) |
         (1 << 0);
}

static void send_anyshake_packet()
{
  uint8_t packet[PACKET_SIZE];

  packet[0] = ANYSHAKE_HEADER_1;
  packet[1] = ANYSHAKE_HEADER_2;

  int64_t timestamp = (int64_t)millis();
  memcpy(&packet[2], &timestamp, TIMESTAMP_SIZE);

  uint32_t device_config = build_device_config();
  memcpy(&packet[10], &device_config, DEVICE_CONFIG_SIZE);

  uint32_t var_uint = 0;
  float var_float = 0.0f;
  switch ((millis() / 1000) % 10)
  {
  case 0:
    var_uint = DEVICE_ID;
    memcpy(&packet[14], &var_uint, VARIABLE_DATA_SIZE);
    break;
  default:
    var_uint = 0;
    memcpy(&packet[14], &var_uint, VARIABLE_DATA_SIZE);
    break;
  }

  uint16_t offset = HEADER_SIZE + TIMESTAMP_SIZE + DEVICE_CONFIG_SIZE + VARIABLE_DATA_SIZE;
  memcpy(&packet[offset], buf.ch0, CHUNK_LENGTH * sizeof(int32_t));
  memcpy(&packet[offset + CHUNK_LENGTH * sizeof(int32_t)], buf.ch1, CHUNK_LENGTH * sizeof(int32_t));
  memcpy(&packet[offset + 2 * CHUNK_LENGTH * sizeof(int32_t)], buf.ch2, CHUNK_LENGTH * sizeof(int32_t));

  uint16_t chk_start = HEADER_SIZE;
  uint16_t chk_end = PACKET_SIZE - TRAILER_SIZE - CHECKSUM_SIZE;
  uint8_t checksum = 0;
  for (uint16_t i = chk_start; i < chk_end; i++)
  {
    checksum ^= packet[i];
  }
  packet[PACKET_SIZE - TRAILER_SIZE - CHECKSUM_SIZE] = checksum;

  packet[PACKET_SIZE - TRAILER_SIZE] = ANYSHAKE_TRAILER_1;
  packet[PACKET_SIZE - 1] = ANYSHAKE_TRAILER_2;

  Serial.write(packet, PACKET_SIZE);
}

void setup()
{
  Serial.begin(115200);

#if defined(ARDUINO_ARCH_RP2040)
  SPI.setSCK(SPI_SCK);
  SPI.setTX(SPI_MOSI);
  SPI.setRX(SPI_MISO);
#endif

#if defined(USE_HSPI)
  hspi.begin(14, 25, 13);
#endif

  A.InitializeADC();
  A.setPGA(pgaSettings[DEFAULT_ADC_GAIN_INDEX]);
  A.setDRATE(dataRateSettings[DEFAULT_ADC_DRATE_INDEX]);

  buf.count = 0;
}

void loop()
{
  unsigned long now = micros();
  if (now - lastSampleTime >= interval)
  {
    lastSampleTime += interval;

    int32_t val0 = A.cycleDifferential();
    int32_t val1 = A.cycleDifferential();
    int32_t val2 = A.cycleDifferential();
    A.cycleDifferential();

    buf.ch0[buf.count] = val0;
    buf.ch1[buf.count] = val1;
    buf.ch2[buf.count] = val2;
    buf.count++;

    if (buf.count >= CHUNK_LENGTH)
    {
      send_anyshake_packet();
      buf.count = 0;
    }
  }
}
