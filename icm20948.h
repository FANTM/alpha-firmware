#ifndef __icm20948_h__
#define __icm20948_h__

/**
 * Initialize the ICM20948 9 axis IMU 
 */
ret_code_t initIcm20948(void);

ret_code_t getAccelerationX(void);

ret_code_t getAccelerationY(void);

ret_code_t getAccelerationZ(void);

#endif