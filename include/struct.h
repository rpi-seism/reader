#include <stdint.h>

#define ANYSHAKE_HEADER_1     0x01
#define ANYSHAKE_HEADER_2     0xFE
#define ANYSHAKE_TRAILER_1    0xEF
#define ANYSHAKE_TRAILER_2    0x10

#define DEFAULT_SAMPLE_RATE       100
#define DEFAULT_ADC_GAIN_INDEX    0
#define DEFAULT_ADC_DRATE_INDEX   8
#define DEVICE_ID                 0x00000001

#define PACKET_INTERVAL_MS    100
#define CHUNK_LENGTH          ((PACKET_INTERVAL_MS * DEFAULT_SAMPLE_RATE) / 1000)

#define CONFIG_PACKET_INTERVAL_100MS  0x00
#define CONFIG_SAMPLE_RATE_100HZ      0x03
#define CONFIG_GNSS_NOT_AVAILABLE     0x00
#define CONFIG_CHANNEL_INT32          0x03
#define CONFIG_CHANNEL_DISABLED       0x00

#define HEADER_SIZE      2
#define TIMESTAMP_SIZE   8
#define DEVICE_CONFIG_SIZE 4
#define VARIABLE_DATA_SIZE 4
#define CHECKSUM_SIZE    1
#define TRAILER_SIZE     2

#define CHANNEL_DATA_SIZE  (CHUNK_LENGTH * 3 * sizeof(int32_t))

#define PACKET_SIZE        (HEADER_SIZE + TIMESTAMP_SIZE + DEVICE_CONFIG_SIZE + VARIABLE_DATA_SIZE + CHANNEL_DATA_SIZE + CHECKSUM_SIZE + TRAILER_SIZE)

// Set via build_flags, e.g. -D UNIX_EPOCH_MS=1740000000000ULL
// Defaults to 0: timestamps count ms from boot. Set to real epoch for proper time.
#ifndef UNIX_EPOCH_MS
#define UNIX_EPOCH_MS 0ULL
#endif

struct __attribute__((__packed__)) SampleBuffer {
    int32_t ch0[CHUNK_LENGTH];
    int32_t ch1[CHUNK_LENGTH];
    int32_t ch2[CHUNK_LENGTH];
    uint8_t count;
};
