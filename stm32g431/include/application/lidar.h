#ifndef LIDAR_H
#define LIDAR_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_cordic.h"

#include "application/robot.h"
#include "application/imu.h"
#include "application/serial.h"

//=========================================================//
// Generic Lidar
//=========================================================//

// Specific parameters
#define LIDAR_POINTS_PER_TURN 360
#define LIDAR_ROTATE_CLOCKWISE 1
#define LIDAR_PACKET_SIZE 6
#define LIDAR_FREQUENCY 5

#define POINT_ANGLE (65536/LIDAR_POINTS_PER_TURN)

#define MAX_POINTS_OF_INTEREST 10
#define NBR_OF_TARGETS 1

struct CParam {
   int16_t x_angle;
   int16_t y_module;
};
typedef struct CParam CParam;


struct Target {
   int16_t x;
   int16_t y;
   uint16_t size;
};
typedef struct Target Target;

void getTargets(Target targets[]);

struct PointMetadata {
   int16_t x;
   int16_t y;
   int16_t o;
};
typedef struct PointMetadata PointMetadata;

struct LidarPoint {
   uint16_t intensity;
   uint16_t distance;
   Position* position;
};
typedef struct LidarPoint LidarPoint;

struct PointOfInterest {
   uint16_t angle0;
   uint16_t angle1;
   uint16_t distance;
   uint16_t intensity;
};
typedef struct PointOfInterest PointOfInterest;

struct Lidar {
   uint8_t transAngle;
   LidarPoint points[LIDAR_POINTS_PER_TURN];
   Position positions[LIDAR_POINTS_PER_TURN];
   // PointOfInterest pointsOfInterset[MAX_POINTS_OF_INTEREST];
   uint8_t firstPOI;
   uint8_t newPOI;
   Target targets[NBR_OF_TARGETS];
   IMU* imu;
   uint8_t target_i;
   int16_t target_start_pi;
};
typedef struct Lidar Lidar;

struct Solution {
   CParam position;
   int16_t targetDifference;
};
typedef struct Solution Solution;



void initLidar(Lidar* lidar, IMU* imu);
void updateLidar(Lidar* lidar, uint16_t angle0, uint16_t angle1);
void computeTargetAngles(Position* estp, Target* t, CParam* cp0, CParam* cp1);
Solution findTarget(Lidar* lidar, uint16_t angle0, uint16_t angle1, Position* estp, CParam* target);
Solution getTargetRelativePosition(PointOfInterest* poi, Position* estp, CParam* target);

//=========================================================//
// Hardware specific
//=========================================================//

struct HLSLFCD2Point {
   uint16_t intensity;
   uint16_t distance;
   uint16_t reserved;
};
typedef struct HLSLFCD2Point HLSLFCD2Point;

struct HLSLFCD2Packet {
   uint8_t sync;
   uint8_t degree;
   uint16_t rpm;
   HLSLFCD2Point points[6];
   uint8_t checksum0;
   uint8_t checksum1;
};
typedef struct HLSLFCD2Packet HLSLFCD2Packet;

uint8_t getHLSLFCD2Packet(Lidar* lidar, uint8_t* packet);
int16_t point2radian(uint16_t index);

#endif
