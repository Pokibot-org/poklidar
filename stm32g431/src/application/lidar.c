#include "application/lidar.h"

//=========================================================//
// Init Target positions
// TODO: Global improve
//=========================================================//
void getTargets(Target targets[]) {
   // targets[0] = (Target) { .x = 0, .y = 67, .size = 90};

   // targets[0] = (Target) { .x = -900-10, .y = +550, .size = 90};
   // targets[1] = (Target) { .x = -900-10, .y = -550, .size = 90};
   // targets[2] = (Target) { .x = +900, .y = 0, .size = 90};

   targets[0] = (Target) { .x = +900-16, .y = 0, .size = 90};
}

//=========================================================//
// Init Lidar struct
//=========================================================//
void initLidar(Lidar* lidar, IMU* imu) {
   uint16_t i;
   for (i = 0; i < LIDAR_POINTS_PER_TURN; i++) {
      lidar->points[i].intensity = 0;
      lidar->points[i].distance = 0;
      lidar->points[i].position = 0;
   }
   lidar->firstPOI = 0;
   lidar->newPOI = 0;
   getTargets(lidar->targets);
   lidar->imu = imu;
   lidar->target_i = 0;
   lidar->target_start_pi = -1;

   Position initPosition = {.x=0, .y=0, .o=0};
   resetIMU(lidar->imu, &initPosition);
}

//=========================================================//
// Update Lidar struct between 2 angles
//=========================================================//
void updateLidar(Lidar* lidar, uint16_t angle0, uint16_t angle1) {
   uint16_t i;
   int16_t angle_lidar;
   Position* estp;
   Target* t;
   // CParam cp0 = {.x_angle=0, .y_module=0};
   // CParam cp1 = {.x_angle=0, .y_module=0};
   CParam cp0, cp1;
   Solution solution;
   Position posForIMU;
   cp0.x_angle = 0;
   cp0.y_module = 0;
   cp1.x_angle = 0;
   cp1.y_module = 0;

   // Transition angle
   lidar->transAngle = angle1;
   // Fill position fields for new lidar points
   // TODO: Update time offset if using new lidar
   // 2ms works fine for a 5Hz-360-points-lidar
   lidar->positions[angle0] = getEstimatedPosition2(lidar->imu, 2);
   estp = &(lidar->positions[angle0]);
   for (i = angle0; i != angle1; i = (i+1)%LIDAR_POINTS_PER_TURN) {
      lidar->points[angle0].position = estp;
   }

   // Target we are looking for
   t = &(lidar->targets[lidar->target_i]);
   computeTargetAngles(estp, t, &cp0, &cp1);

   // Sweep the new points to check if we start/end to see a target
   for (i = angle0; i != angle1; i = (i+1)%LIDAR_POINTS_PER_TURN) {
      // Absolute angle of the point
      angle_lidar = point2radian(i)+estp->o;
      if (lidar->target_start_pi == -1) {
         if (abs(angle_lidar-cp0.x_angle) < cp1.x_angle) {
            // Starting to see the target
            lidar->target_start_pi = i;
         }
      } else {
         if (abs(angle_lidar-cp0.x_angle) > cp1.x_angle) {
            // Target is no longer seen
            // Call findTarget() to precisely find the target
            // solution = findTarget(lidar, lidar->target_start_pi, i, estp, &cp0);
            // posForIMU.x = t->x-solution.position.x_angle;
            // posForIMU.y = t->y-solution.position.y_module;
            // if (solution.targetDifference < 500) {
            // print16(solution.targetDifference);
            // print(" ");
            // print8(lidar->target_i);
            // print("\r\nGuess: ");
            // print16(posForIMU.x);
            // print(" ");
            // print16(posForIMU.y);
            // print("\r\nEstimated: ");
            // print16(estp->x);
            // print(" ");
            // print16(estp->y);
            // print(" ");
            // print16(estp->o);
            // print(" ");
            // print16(lidar->imu->speedX);
            // print(" ");
            // print16(lidar->imu->speedY);
            // print("\r\n");
            // if (!isPositionTooFar(lidar->imu, &posForIMU)) {
            //    updateIMUPosition(lidar->imu, &posForIMU);
            // } else {
            //    // print("==========FUCK==========\r\n");
            // }

            lidar->target_start_pi = -1;
            // Looking for the new target
            lidar->target_i = (lidar->target_i+1)%NBR_OF_TARGETS;
            t = &(lidar->targets[lidar->target_i]);
            computeTargetAngles(estp, t, &cp0, &cp1);
         }
      }
   }
}

//=========================================================//
// estp: Estimated position
// t: Target we are looking for
// cp0: Return polar coordinates of the target relative to estimated position,
// but with absolute angle
// cp1: Return angular size of the target, as seen from the estimated position
//=========================================================//
void computeTargetAngles(Position* estp, Target* t, CParam* cp0, CParam* cp1) {
   CParam cpin;
   // Compute target position angle
   cpin.x_angle  = t->x-estp->x;
   cpin.y_module = t->y-estp->y;
   LL_CORDIC_WriteData(CORDIC, *(uint32_t*)&cpin);
   *((uint32_t*)cp0) = LL_CORDIC_ReadData(CORDIC);

   // Compute target size angle
   cpin.x_angle = cp0->y_module;
   // Size should be divided by 2
   // But we multiply it by 4 to take a margin, so by 2
   cpin.y_module = 2*t->size;
   LL_CORDIC_WriteData(CORDIC, *(uint32_t*)&cpin);
   *((uint32_t*)cp1) = (int32_t)LL_CORDIC_ReadData(CORDIC);
}

//=========================================================//
// Precisely search and find a target localized between 2 angles of the lidar.
// TODO: Do not use estp as global position, but specific position for each point
//=========================================================//
Solution findTarget(Lidar* lidar, uint16_t angle0, uint16_t angle1, Position* estp, CParam* target) {
   uint16_t i, i_next;
   int8_t poinbr;
   uint32_t totalDistance;
   uint32_t totalIntensity;
   uint16_t pointCount;
   int16_t bestDiff;
   int8_t bestPi;
   int16_t delta_d;
   PointOfInterest pointsOfInterset[10];
   Solution solution;


   // print16(angle0);
   // print(" ");
   // print16(angle1);
   // print("\r\n");
   //
   // for (i = angle0; i != angle1; i = (i+1)%LIDAR_POINTS_PER_TURN) {
   //    print16(lidar->points[i].distance);
   //    print(" ");
   // }
   // print("\r\n");

   i = angle0;
   poinbr = -1;
   while (i != angle1) {
      i_next = (i+1)%LIDAR_POINTS_PER_TURN;
      delta_d = lidar->points[i].distance-lidar->points[i_next].distance;

      // print16(abs(delta_d));
      // print(" ");

      if (poinbr != -1) {
         totalDistance+=lidar->points[i].distance;
         totalIntensity+=lidar->points[i].intensity;
         pointCount++;
      }

      if (abs(delta_d) > 150) {
         if (poinbr != -1) {
            pointsOfInterset[poinbr].angle1 = i;
            pointsOfInterset[poinbr].distance = totalDistance/pointCount;
            pointsOfInterset[poinbr].intensity = totalIntensity/pointCount;
         }
         poinbr++;
         pointsOfInterset[poinbr].angle0 = i;
         totalDistance = 0;
         totalIntensity = 0;
         pointCount = 0;
      }

      i = i_next;
   }
   // print("\r\n");
   if (poinbr < 0) {
      poinbr = 0;
   }

   bestDiff = 0x7FFF;
   bestPi = 0;
   for (i = 0; i < poinbr; i++) {
      delta_d = abs(target->y_module-pointsOfInterset[i].distance);
      if (bestDiff > delta_d) {
         bestDiff = delta_d;
         bestPi = i;
      }
      // print16(pointsOfInterset[i].distance);
      // print(" ");
      // print16(pointsOfInterset[i].intensity);
      // print("\r\n");
   }
   // print8(bestPi);
   // print(" ");
   // print8(poinbr);
   // print(" (best)\r\n");
   // print("POI: ");
   // print16(target->y_module);
   // print(" ");
   // print16(pointsOfInterset[bestPi].distance);
   // print(" ");
   // print16(pointsOfInterset[bestPi].intensity);
   // print("\r\n");

   solution = getTargetRelativePosition(&(pointsOfInterset[bestPi]), estp, target);
   solution.targetDifference = bestDiff;
   return solution;
}

Solution getTargetRelativePosition(PointOfInterest* poi, Position* estp, CParam* target) {
   uint16_t tangle;
   int16_t tangleRad;
   CParam cpin;
   Solution solution;

   if (poi->angle0 < poi->angle1) {
      tangle = (poi->angle1 + poi->angle0)/2;
   } else {
      tangle = (poi->angle1+LIDAR_POINTS_PER_TURN - poi->angle0)/2;
      if (tangle >= LIDAR_POINTS_PER_TURN) {
         tangle-=LIDAR_POINTS_PER_TURN;
      }
   }
   tangleRad = point2radian(tangle)+estp->o;


   LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_COSINE);
   cpin.x_angle  = tangleRad;
   cpin.y_module = poi->distance;
   // print16(cpin.x_angle);
   // print(" ");
   // print16(cpin.y_module);
   // print(" ");
   LL_CORDIC_WriteData(CORDIC, *(uint32_t*)&cpin);
   *((uint32_t*)&(solution.position)) = LL_CORDIC_ReadData(CORDIC);
   LL_CORDIC_SetFunction(CORDIC, LL_CORDIC_FUNCTION_PHASE);
   return solution;
}


//============================================================================//
//============================================================================//
//                       HARDWARE SPECIFIC FUNCTIONS
//============================================================================//
//============================================================================//


//=========================================================//
// Gets a raw packet received from HLSLFCD2
// Puts data into generic lidar struct
// Return 0 if CRC is ok, else 1
//=========================================================//
uint8_t getHLSLFCD2Packet(Lidar* lidar, uint8_t* rawPacket) {
   HLSLFCD2Packet* packet;
   uint8_t i, checksum;
   uint16_t degree;

   checksum = 1;
   for (i = 0; i < 40; i++) {
      checksum+=rawPacket[i];
   }

   packet = (HLSLFCD2Packet*)rawPacket;
   degree = (packet->degree-160)*6;
   for (i = 0; i < 6; i++) {
      lidar->points[degree+i].intensity = packet->points[i].intensity;
      lidar->points[degree+i].distance  = packet->points[i].distance;
   }

   updateLidar(lidar, degree, (degree+6)%LIDAR_POINTS_PER_TURN);

   if (checksum == packet->checksum0 && checksum == packet->checksum1) {
      return 0;
   } else {
      return 1;
   }
}

//=========================================================//
// Converts point to index to radian between -32768 and +32767
//=========================================================//
int16_t point2radian(uint16_t index) {
   // return index*65536/LIDAR_POINTS_PER_TURN;
   int16_t r = index;
   r = r*128/45;
   r = r*64;
   return -r;
}
