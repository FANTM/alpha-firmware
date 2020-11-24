#ifndef __fantm_imu_h__
#define __fantm_imu_h__

#include "app_fifo.h"
#include "packet.h"

/**
 * This file is the API for the entire IMU folder, the rest should stay internal! 
 */

ret_code_t initIMU(Packet_t *packet);

void printAGMT(void);

ret_code_t updateAGMT(void);

void reduceAGMT();

typedef struct Data_t {
    app_fifo_t accelX;
    app_fifo_t accelY;
    app_fifo_t accelZ;
    app_fifo_t gyroX;
    app_fifo_t gyroY;
    app_fifo_t gyroZ;
    app_fifo_t temp;
    app_fifo_t magX;
    app_fifo_t magY;
    app_fifo_t magZ;
} Data_t;

#endif