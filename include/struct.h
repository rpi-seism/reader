#include <stdint.h>

#define CHECKSUM_LEN 3
#define DATA_LEN 5

struct __attribute__((__packed__)) ADC_Packet {
    uint8_t header1 = 0xFC;
    uint8_t header2 = 0x1B;
    int32_t channel_n[DATA_LEN];
    int32_t channel_e[DATA_LEN];
    int32_t channel_z[DATA_LEN];
    uint8_t checksum[CHECKSUM_LEN];
    uint8_t padding;
};


struct __attribute__((__packed__)) SettingsPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0xDD;
    uint16_t samplingSpeed;
    uint8_t ADCGain;
    uint8_t ADCDataRate;
};
