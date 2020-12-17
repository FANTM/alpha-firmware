#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"

#include "icm20948.h"
#include "ak09916.h"
#include "imu_handlers.h"
#include "util.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

typedef enum AKReg_t {
    DEV_ID = 0x01,
    STATUS_1 = 0x10,
    MAGX_L,
    MAGX_H,
    MAGY_L,
    MAGY_H,
    MAGZ_L,
    MAGZ_H,
    STATUS_2 = 0x18,
    CNTL_2 = 0x31,
    CNTL_3
} AKReg_t;

typedef enum {
  AK09916_MAG_DATARATE_SHUTDOWN = 0x0, ///< Stops measurement updates
  AK09916_MAG_DATARATE_SINGLE =
      0x1, ///< Takes a single measurement then switches to
           ///< AK09916_MAG_DATARATE_SHUTDOWN
  AK09916_MAG_DATARATE_10_HZ = 0x2,  ///< updates at 10Hz
  AK09916_MAG_DATARATE_20_HZ = 0x4,  ///< updates at 20Hz
  AK09916_MAG_DATARATE_50_HZ = 0x6,  ///< updates at 50Hz
  AK09916_MAG_DATARATE_100_HZ = 0x8, ///< updates at 100Hz
} AK09916_Data_Rate_t;

static const uint8_t AK09916_ADDR = 0x0C;

static ret_code_t writeAK0(AKReg_t reg, uint8_t data);
ret_code_t synchReadAK0Data(int16_t *dest);

static ret_code_t setAK0DataRate(AK09916_Data_Rate_t dataRate) {
    /* Taken from Adafruit repo: https://github.com/adafruit/Adafruit_ICM20X/blob/master/Adafruit_ICM20948.cpp
   *
   * Following the datasheet, the sensor will be set to
   * AK09916_MAG_DATARATE_SHUTDOWN followed by a 100ms delay, followed by
   * setting the new data rate.
   *
   * See page 9 of https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
   */

    ret_code_t errCode;
    if ((errCode = writeAK0(CNTL_2, AK09916_MAG_DATARATE_SHUTDOWN)) != NRF_SUCCESS) 
        return errCode;

    nrf_delay_ms(100);

    return writeAK0(CNTL_2, dataRate);
}

static ret_code_t setAK0Reg(AKReg_t reg) {
    changeBank(BANK_3);
    ICMReg_t slvReg = {.reg3 = I2C_SLV4_REG};
    uint8_t bitmask[] = {
        BIT_MASK(reg, 0), BIT_MASK(reg, 1), BIT_MASK(reg, 2), BIT_MASK(reg, 3), 
        BIT_MASK(reg, 4), BIT_MASK(reg, 5), BIT_MASK(reg, 6), BIT_MASK(reg, 7)
    };
    return writeICM(&slvReg, bitmask);
}

static ret_code_t setAK0RW(bool read) {
    changeBank(BANK_3);
    uint8_t bitmask[] = {
        BIT_MASK(AK09916_ADDR, 0),
        BIT_MASK(AK09916_ADDR, 1),
        BIT_MASK(AK09916_ADDR, 2),
        BIT_MASK(AK09916_ADDR, 3),
        BIT_MASK(AK09916_ADDR, 4),
        BIT_MASK(AK09916_ADDR, 5),
        BIT_MASK(AK09916_ADDR, 6),
        2
    };

    if (read) {
        bitmask[7] = 1;
    } else {
        bitmask[7] = 0;
    }

    ICMReg_t reg = {.reg3 = I2C_SLV4_ADDR};
    return writeICM(&reg, bitmask);
}

static ret_code_t setAK0DataOut(uint8_t data) {
    changeBank(BANK_3);
    ICMReg_t reg = {.reg3 = I2C_SLV4_DO};
    uint8_t bitmask[] = {
        BIT_MASK(data, 0),
        BIT_MASK(data, 1),
        BIT_MASK(data, 2),
        BIT_MASK(data, 3),
        BIT_MASK(data, 4),
        BIT_MASK(data, 5),
        BIT_MASK(data, 6),
        BIT_MASK(data, 7)
    };
    return writeICM(&reg, bitmask);
}

// Kind of weird but you write the internal reg via SLV4
static ret_code_t writeAK0(AKReg_t reg, uint8_t data) {
    setAK0DataOut(data);
    setAK0Reg(reg);
    setAK0RW(false);  // Write mode
    changeBank(BANK_3);
    uint8_t ctrlBitmask[] = { 
        0, 0, 0, 0, 0, 0, 0, 1  // 0x80
    };
    ICMReg_t wrreg = {.reg3 = I2C_SLV4_CTRL};
    return writeICM(&wrreg, ctrlBitmask);
}

static ret_code_t configI2CMaster(void) {
    changeBank(BANK_3);
    ret_code_t errCode;
    ICMReg_t reg = {.reg3 = I2C_MST_CTRL};
    uint8_t bitmask[] = { // 0x17
        1, 1, 1, 0, //0x07
        1, 0, 0, 0  //0x10
    };
    if ((errCode = writeICM(&reg, bitmask)) != NRF_SUCCESS)
        return errCode;
    
    changeBank(BANK_0);
    uint8_t ctrlBitmask[] = {2, 2, 2, 2, 2, 1, 2, 2};
    ICMReg_t config = {.reg0 = USER_CTRL};
    
    return writeICM(&config, ctrlBitmask);
}

ret_code_t calibrateAK09916(int32_t *magBias, int32_t *magScale) {
    ret_code_t errCode = NRF_SUCCESS;
    uint16_t sampleCount = 1500;  // We set a 100Hz ODR, so data is avail. every 10ms
    //int32_t magBias[3] = {0,0,0};
    //int32_t magScale[3] = {0,0,0};
    int16_t magMax[3] = {-32753, -32767, -32767},
            magMin[3] = {32753, 0x7FFF, 0x7FFF},
            magTemp[3] = {0,0,0};
    changeBank(BANK_0);
    for (int i = 0; i < sampleCount; i++) {
        synchReadAK0Data(magTemp);
        for (int j = 0; j < 3; j++) {
            if (magTemp[j] > magMax[j]) {
                magMax[j] = magTemp[j];
            }
            if (magTemp[j] < magMin[j]) {
                magMin[j] = magTemp[j];
            }
        }
        nrf_delay_ms(12); 
    }

    // Hard iron!
    magBias[0] = (magMax[0] + magMin[0]) / 2;
    magBias[1] = (magMax[1] + magMin[1]) / 2;
    magBias[2] = (magMax[2] + magMin[2]) / 2;
    NRF_LOG_INFO("HARD IRONS: %d, %d, %d", magBias[0], magBias[1], magBias[2]);

    magScale[0] = (magMax[0] - magMin[0]) / 2;
    magScale[1] = (magMax[1] - magMin[1]) / 2;
    magScale[2] = (magMax[2] - magMin[2]) / 2;

    float avg = magScale[0] + magScale[1] + magScale[2];
    avg /= 3.0f;

    magScale[0] = avg / ((float) magScale[0]);
    magScale[0] = avg / ((float) magScale[0]);
    magScale[0] = avg / ((float) magScale[0]);
    NRF_LOG_INFO("SOFT IRONS: %d, %d, %d", magScale[0], magScale[1], magScale[2]);

    NRF_LOG_INFO("Mag cal done.");
    return errCode;
}

ret_code_t synchReadAK0Data(int16_t *dest) {
    uint8_t rawResult[8];
    uint8_t statusByte[1];
    ICMReg_t reg = {.reg0=EXT_SLV_SENS_DATA_00};
    ret_code_t errCode = synchReadICM(&reg, 1, statusByte);
    if (statusByte[0] & 0x01) {
        reg.reg0 = EXT_SLV_SENS_DATA_01;
        errCode = synchReadICM(&reg, 8, rawResult);
        uint8_t status2 = rawResult[7];
        if (!(status2 & 0x08)) {
            dest[0] = (int16_t) ((int16_t) rawResult[1] << 8) | rawResult[0];
            dest[1] = (int16_t) ((int16_t) rawResult[3] << 8) | rawResult[2];
            dest[2] = (int16_t) ((int16_t) rawResult[5] << 8) | rawResult[4];
        }
    }

    NRF_LOG_INFO("READINGS: %d, %d, %d", dest[0], dest[1], dest[2]);

    return errCode;
}

/* Not only initializes the sensor, but also kicks off continuous readings */
ret_code_t initAK09916() {
    ret_code_t errCode = NRF_SUCCESS;

    APP_ERROR_CHECK(configI2CMaster());
    changeBank(BANK_3);
    if ((errCode = setAK0DataRate(AK09916_MAG_DATARATE_100_HZ)) != NRF_SUCCESS)
        return errCode;

    ICMReg_t reg = {.reg3 = I2C_SLV0_ADDR};
    uint8_t addrBitmask[] = {
        BIT_MASK(AK09916_ADDR, 0),
        BIT_MASK(AK09916_ADDR, 1),
        BIT_MASK(AK09916_ADDR, 2),
        BIT_MASK(AK09916_ADDR, 3),
        BIT_MASK(AK09916_ADDR, 4),
        BIT_MASK(AK09916_ADDR, 5),
        BIT_MASK(AK09916_ADDR, 6),
        1  // First we want to read
    };
    if ((errCode = writeICM(&reg, addrBitmask)) != NRF_SUCCESS)
        return errCode;

    reg.reg3 = I2C_SLV0_REG;
    uint8_t regBitmask[] = {
        BIT_MASK(STATUS_1, 0),
        BIT_MASK(STATUS_1, 1),
        BIT_MASK(STATUS_1, 2),
        BIT_MASK(STATUS_1, 3),
        BIT_MASK(STATUS_1, 4),
        BIT_MASK(STATUS_1, 5),
        BIT_MASK(STATUS_1, 6),
        BIT_MASK(STATUS_1, 7),
    };
    if ((errCode = writeICM(&reg, regBitmask)) != NRF_SUCCESS)
        return errCode;

    reg.reg3 = I2C_SLV0_CTRL;
    uint8_t ctrlBitmask[] = { 
        1, 0, 0, 1, 0, 0, 0, 1  // 0x89 <- enable + read 9 bytes at a time
    };
    if ((errCode = writeICM(&reg, ctrlBitmask)) != NRF_SUCCESS)
        return errCode;

    int32_t bias[3], scale[3];
    return NRF_SUCCESS;
    //return calibrateAK09916(bias, scale);
}
