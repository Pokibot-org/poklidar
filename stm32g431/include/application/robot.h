#ifndef ROBOT_H
#define ROBOT_H

#include "stm32g4xx_hal.h"

struct Position {
   int16_t x;
   int16_t y;
   int16_t o;
};
typedef struct Position Position;

#endif
