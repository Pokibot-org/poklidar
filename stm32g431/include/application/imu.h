#ifndef IMU_H
#define IMU_H

#include "stm32g4xx_hal.h"

#include "application/robot.h"

#define BNO055_I2C_ADDR (0x28<<1)

struct BNO055_DATA {
   int16_t GYR_X;
   int16_t GYR_Y;
   int16_t GYR_Z;
   int16_t EUL_X; // Yaw
   int16_t EUL_Y; // Roll
   int16_t EUL_Z; // Pitch
   int16_t QUA_W;
   int16_t QUA_X;
   int16_t QUA_Y;
   int16_t QUA_Z;
   int16_t LIA_X;
   int16_t LIA_Y;
   int16_t LIA_Z;
};
typedef struct BNO055_DATA BNO055_DATA;

// GYR_Z, EUL_X, LIA_X, LIA_Y

struct IMU {
   Position estimatedPosition;
   int16_t speedX;
   int16_t speedY;
   uint32_t timestamp_o;
   uint32_t timestamp_xy;
   BNO055_DATA rawData;
   int16_t yaw;
   int16_t gyrz;
   int16_t liax;
   int16_t liay;
};
typedef struct IMU IMU;

void initIMU(IMU* imu);

Position getEstimatedPosition(IMU* imu, int16_t toffset);
Position getEstimatedPosition2(IMU* imu, int16_t toffset);
void setupBNO055(I2C_HandleTypeDef * hi2c);
void getBNO055Data(I2C_HandleTypeDef * hi2c, IMU* imu);
void insertIMUData(IMU* imu, uint8_t* buffer);
void updateIMUOrientation(IMU* imu);
void updateIMUPosition(IMU* imu, Position* p);

#endif
