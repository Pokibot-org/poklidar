#ifndef SERIAL_H
#define SERIAL_H

#include "stm32g4xx_hal.h"

void print32(uint32_t v);
void print16(uint16_t v);
void print8(uint8_t v);
void print(char s[]);

#endif
