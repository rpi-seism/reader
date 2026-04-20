#include <stdint.h>

#define MAX_CHANNELS 4  // with ADS1256 we can have max 3 differential channels


struct __attribute__((__packed__)) ADC_Packet {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xBB;
    int32_t channelData[MAX_CHANNELS];
    uint32_t crc;
};


struct __attribute__((__packed__)) SettingsPacket {
    uint8_t header1 = 0xCC;
    uint8_t header2 = 0xDD;
    uint16_t samplingSpeed;
    uint8_t ADCGain;
    uint8_t ADCDataRate;
    uint8_t numChannels;
};