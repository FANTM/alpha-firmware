#ifndef __fantmpacket_h__
#define __fantmpacket_h__


#include "app_util_platform.h"

#define PACKET_SIZE 20
typedef struct Packet_t {
int16_t accelX;
int16_t accelY;
int16_t accelZ;
int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;
int16_t magX;
int16_t magY;
int16_t magZ;
int16_t temp;
} Packet_t;

void telemetrySend(Packet_t *data);

#endif
