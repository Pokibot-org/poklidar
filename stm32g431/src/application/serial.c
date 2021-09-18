#include "application/serial.h"

UART_HandleTypeDef huart2;


void print32(uint32_t v) {
   uint8_t i, j;
   char s[8];
   for (i = 0; i < 8; i++) {
      j = v&0x0F;
      v = v>>4;
      if (j < 10) {
         s[7-i] = j+'0';
      } else {
         s[7-i] = j-10+'A';
      }
   }
   HAL_UART_Transmit(&huart2, (uint8_t*)s, 8, 100);
}
void print16(uint16_t v) {
   uint8_t i, j;
   char s[4];
   for (i = 0; i < 4; i++) {
      j = v&0x0F;
      v = v>>4;
      if (j < 10) {
         s[3-i] = j+'0';
      } else {
         s[3-i] = j-10+'A';
      }
   }
   HAL_UART_Transmit(&huart2, (uint8_t*)s, 4, 100);
}
void print8(uint8_t v) {
   uint8_t i, j;
   char s[2];
   for (i = 0; i < 2; i++) {
      j = v&0x0F;
      v = v>>4;
      if (j < 10) {
         s[1-i] = j+'0';
      } else {
         s[1-i] = j-10+'A';
      }
   }
   HAL_UART_Transmit(&huart2, (uint8_t*)s, 2, 100);
}
void print(char s[]) {
   uint8_t size;
   for(size = 0; s[size]; size++);
   HAL_UART_Transmit(&huart2, (uint8_t*)s, size, 100);
}
