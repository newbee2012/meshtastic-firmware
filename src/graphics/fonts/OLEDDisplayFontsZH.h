#ifndef OLEDDISPLAYFONTSZH_h
#define OLEDDISPLAYFONTSZH_h

#ifdef ARDUINO
#include <Arduino.h>
#elif __MBED__
#define PROGMEM
#endif

#ifdef OLED_MESSAGE_ZH

#define GB_START_HIGH 0xA1
#define GB_START_LOW 0xA1
#define GB_POSITIONS_PER_ZONE 94
#define GB_END_HIGH 0xE7
#define GB_END_LOW 0xFE

struct CodeMap {
    uint16_t unicode;
    uint16_t gbCode;
};

extern const uint8_t Simsun_Plain_10_ZH[] PROGMEM;
extern const CodeMap UTF8ToGB_Table[5941] PROGMEM;
#endif
#endif
