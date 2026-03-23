#include <stdint.h>

struct __attribute__((__packed__)) ADC_Packet {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    int32_t ch0;
    int32_t ch1;
    int32_t ch2;
    uint32_t crc;
};


struct __attribute__((__packed__)) SettingsPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0xDD;
    uint16_t samplingSpeed;
    uint8_t ADCGain;
    uint8_t ADCDataRate;
};