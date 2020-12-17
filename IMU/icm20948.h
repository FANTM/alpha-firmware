#ifndef __icm20948_h__
#define __icm20948_h__

#include "app_fifo.h"
#include "nrf_spi_mngr.h"
#include "imu.h"

#define ICM_SPI_TRANSACTION(_begin_callback, _end_callback, _p_transfers, _number_of_transfers)   \
{                                                                               \
    .begin_callback         = (nrf_spi_mngr_callback_begin_t) _begin_callback,  \
    .end_callback           = (nrf_spi_mngr_callback_end_t)   _end_callback,    \
    .p_user_data            = NULL,                                             \
    .p_transfers            = (nrf_spi_mngr_transfer_t const *) _p_transfers,   \
    .number_of_transfers    = (uint8_t) _number_of_transfers,                   \
    .p_required_spi_cfg     = NULL,                                             \
}

typedef enum ICMBank_t {
    BANK_0,
    BANK_1,
    BANK_2,
    BANK_3
} ICMBank_t;

typedef enum ICMRegBank0_t {
    WHO_AM_I = 0x00,
    USER_CTRL = 0x03,
    PWR_MGMT_1 = 0x06,
    INT_ENABLE = 0x10,
    ACCEL_X_H = 0x2D,
    ACCEL_X_L,
    ACCEL_Y_H,
    ACCEL_Y_L,
    ACCEL_Z_H,
    ACCEL_Z_L,
    GYRO_X_H,
    GYRO_X_L,
    GYRO_Y_H,
    GYRO_Y_L,
    GYRO_Z_H,
    GYRO_Z_L,
    TEMP_H,
    TEMP_L,
    EXT_SLV_SENS_DATA_00,
    EXT_SLV_SENS_DATA_01,
    EXT_SLV_SENS_DATA_02,
    EXT_SLV_SENS_DATA_03,
    FIFO_EN_1 = 0x66,
    FIFO_EN_2,
    FIFO_RST,
    FIFO_MODE,
    FIFO_COUNTH = 0x70,
    FIFO_COUNTL,
    FIFO_R_W,
    BANK0_BANK_SEL = 0x7F
} ICMRegBank0_t;

typedef enum ICMRegBank1_t {
    XA_OFFSET_H = 0x14,
    XA_OFFSET_L = 0x15,
    YA_OFFSET_H = 0x17,
    YA_OFFSET_L = 0x18,
    ZA_OFFSET_H = 0x1A,
    ZA_OFFSET_L = 0x1B,
    BANK1_BANK_SEL = 0x7F
} ICMRegBank1_t;

typedef enum ICMRegBank2_t {
    GYRO_SMPLRT_DIV = 0x00,
    GYRO_CONFIG_1,
    GYRO_CONFIG_2,
    XG_OFFSET_H,
    XG_OFFSET_L,
    YG_OFFSET_H,
    YG_OFFSET_L,
    ZG_OFFSET_H,
    ZG_OFFSET_L,
    ODR_ALIGN_EN,
    ACCEL_SMPLRT_DIV_1 = 0x10,
    ACCEL_SMPLRT_DIV_2,
    ACCEL_INTEL_CTRL,
    ACCEL_WOM_THR,
    ACCEL_CONFIG_1,
    ACCEL_CONFIG_2,
    TEMP_CONFIG = 0x53,
    BANK2_BANK_SEL = 0x7F
} ICMRegBank2_t;

typedef enum ICMRegBank3_t {
    I2C_MST_CTRL   = 0x01,
    I2C_SLV0_ADDR  = 0x03,
    I2C_SLV0_REG,
    I2C_SLV0_CTRL,
    I2C_SLV0_DO,
    I2C_SLV4_ADDR = 0x13,
    I2C_SLV4_REG,
    I2C_SLV4_CTRL,
    I2C_SLV4_DO,
    BANK3_BANK_SEL = 0x7F
} ICMRegBank3_t;

typedef union ICMReg_t {
    ICMRegBank0_t reg0;
    ICMRegBank1_t reg1;
    ICMRegBank2_t reg2;
    ICMRegBank3_t reg3;
} ICMReg_t;

ret_code_t writeICM(ICMReg_t *reg,  uint8_t *data);
ret_code_t readICM(ICMReg_t *reg,  nrf_spi_mngr_transaction_t *transaction, Data_t *outBuffer,  void (*endHandler)(ret_code_t, void *));
ret_code_t synchReadICM(ICMReg_t *reg, size_t size, uint8_t *data);
ret_code_t changeBank(ICMBank_t bank);

typedef struct HandlerParameters_t {
    uint8_t reg1;
    uint8_t reg2;
    Data_t *out;
    uint8_t *cmdBuff;
    uint8_t *recvBuff;
} HandlerParameters_t;


/**
 * Initialize the ICM20948 9 axis IMU 
 */
ret_code_t initIcm20948(void);

ret_code_t wakeUpICM(void);

ret_code_t sleepICM(void);

ret_code_t configICM(void);

#endif