#include <stdint.h>

struct __attribute__((__packed__)) ADC_Packet {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    int32_t ch0;
    int32_t ch1;
    int32_t ch2;
    uint32_t unix_sec;   ///< 0 = GPS not locked, use with caution on daemon side
    uint32_t unix_usec;  ///< Microseconds within the second; only valid if unix_sec is valid.
    uint32_t crc;
};


struct __attribute__((__packed__)) SettingsPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0xDD;
    uint16_t samplingSpeed;
    uint8_t ADCGain;
    uint8_t ADCDataRate;
};