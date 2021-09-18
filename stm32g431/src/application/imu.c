#include "application/imu.h"
#include "application/robot.h"
#include "application/lidar.h"

#include "stm32g4xx_ll_cordic.h"

#include "math.h"

void resetIMU(IMU* imu, Position* p) {
   imu->estimatedPosition.x = p->x;
   imu->estimatedPosition.y = p->y;
   imu->estimatedPosition.o = p->o;
   imu->speedX = 0;
   imu->speedY = 0;
   imu->timestamp_xy =  HAL_GetTick();
}


Position getEstimatedPosition(IMU* imu, int16_t toffset) {
   Position p = { .x = 0, .y = 0, .o = 0};
   int16_t dt = HAL_GetTick()-imu->timestamp_o-toffset;
   // Should be divided by 500
   p.o = (imu->yaw - ((imu->gyrz)*dt/700))/16;
   // p.o = imu->yaw/16;
   while (p.o >=360) {
      p.o-=360;
   }
   while (p.o < 0) {
      p.o+=360;
   }
   return p;
}
Position getEstimatedPosition2(IMU* imu, int16_t toffset) {
   int32_t calcul;
   Position p = { .x = 0, .y = 0, .o = 0};
   int16_t dt = HAL_GetTick()-imu->timestamp_o;
   int16_t dt_offset = dt-toffset;
   // Should be divided by 500
   p.o = -(imu->yaw - ((imu->gyrz)*dt_offset/700))/16;
   // p.o = imu->yaw/16;
   while (p.o >=180) {
      p.o-=360;
   }
   while (p.o < -180) {
      p.o+=360;
   }
   // p.o = p.o*8192/45;
   // To prevent overflows:
   p.o = p.o*128/45;
   p.o = p.o*64;

   // p.x = (int32_t)imu->estimatedPosition.x + (int32_t)imu->speedX*dt/1000;
   // p.y = (int32_t)imu->estimatedPosition.y + (int32_t)imu->speedY*dt/1000;

   p.x = (int32_t)imu->estimatedPosition.x;
   p.y = (int32_t)imu->estimatedPosition.y;
   return p;
}

void setupBNO055(I2C_HandleTypeDef * hi2c) {
   uint8_t data;

   // Full reset
   data = 0x60;
   HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x3F, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   HAL_Delay(1000);
   // Select external clock
   data = 0x80;
   HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x3F, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // data = 0x0C;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

   // // Goto page 1
   // data = 0x01;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // // Interrupt mask
   // data = 0x01;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x0F, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // // Interrupt enable
   // data = 0x01;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x10, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   //
   // data = 0x10;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x11, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // data = 0xFC;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   //
   // // Goto page 0
   // data = 0x00;
   // HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

   // Select NDOF mode
   data = 0x0C;
   HAL_I2C_Mem_Write(hi2c, BNO055_I2C_ADDR, 0x3D, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

   // HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, 0x04, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // print8(data);
   // print(" ");
   // HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR, 0x05, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
   // print8(data);
   // print("\r\n");
   // SW: 3.11
}

void getBNO055Data(I2C_HandleTypeDef * hi2c, IMU* imu) {
   HAL_I2C_Mem_Read_IT(hi2c, BNO055_I2C_ADDR, 0x14, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&(imu->rawData), 26);
}

void insertIMUData(IMU* imu, uint8_t* buffer) {
   // buffer[8] = 0xAF;
   // buffer[9] = 0xFA;
   ((int16_t*)buffer)[4]  = 0xFAAF;
   ((int16_t*)buffer)[7]  = imu->rawData.GYR_Z;
   ((int16_t*)buffer)[10] = imu->rawData.EUL_X;
   ((int16_t*)buffer)[13] = imu->rawData.LIA_X;
   ((int16_t*)buffer)[16] = imu->rawData.LIA_Y;
}

void updateIMUOrientation(IMU* imu) {
   imu->timestamp_o = HAL_GetTick();
   imu->yaw = imu->rawData.EUL_X;
   imu->gyrz = imu->rawData.GYR_Z;
}

void updateIMUPosition(IMU* imu, Position* p) {
   uint32_t newTimestamp = HAL_GetTick();
   // Delta T in milli second
   int32_t deltaT = newTimestamp-imu->timestamp_xy;
   int32_t speedX, speedY;
   // Delta D in micro meter
   // Speed in micro meter per milli second
   // Which is equivalent to mm/s
   speedX = ((p->x-imu->estimatedPosition.x)*1000)/deltaT;
   speedY = ((p->y-imu->estimatedPosition.y)*1000)/deltaT;
   if (speedX < 2000 && speedY < 2000) {
      // Update speed only if the values make sense
      imu->speedX = speedX;
      imu->speedY = speedY;
   }
   // Now that speed as been computed,
   // we can store timestamp and new position
   imu->timestamp_xy = newTimestamp;
   imu->estimatedPosition.x = p->x;
   imu->estimatedPosition.y = p->y;
}

uint8_t isPositionTooFar(IMU* imu, Position* p) {
   uint32_t timestamp = HAL_GetTick();
   CParam cpin, cpout;
   cpin.x_angle  = (imu->estimatedPosition.x-p->x);
   cpin.y_module = (imu->estimatedPosition.x-p->x);
   // Assuming the CORDIC is already configured in FUNCTION_PHASE
   LL_CORDIC_WriteData(CORDIC, *(uint32_t*)&cpin);
   *((uint32_t*)&cpout) = LL_CORDIC_ReadData(CORDIC);
   // Margin is calculated assuming the robot can move maximum at 1m/s
   // So, the robot moves maximum at 1mm/ms
   // A factor 2 is added to take extra margin
   uint16_t errorMargin = (timestamp-imu->timestamp_xy)/5 + 0x20;
   // print("Margin: ");
   // print16(errorMargin);
   // print("\r\n");
   // Position is too far if distance between estimated position
   // and guessed position is greater than the margin error
   return cpout.y_module > errorMargin;
}

// Callibration status
// HAL_I2C_Mem_Read(&hi2c1, BNO055_I2C_ADDR, 0x35, I2C_MEMADD_SIZE_8BIT, i2cBuffer, 1, 100);
