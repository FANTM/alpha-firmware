#ifndef __icm20948_h__
#define __icm20948_h__

/**
 * Initialize the ICM20948 9 axis IMU 
 */
ret_code_t initIcm20948(void);

void printAGMT(void);

ret_code_t updateAGMT(void);

#endif