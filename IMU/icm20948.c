
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#include "icm20948.h"
#include "imu_handlers.h"
#include "data.h"
#include "util.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define ICM20948_QUEUE_LENGTH 64
#define ICM20948_SPI_INSTANCE_ID 0
#define ICM_WHO_AM_I 0xEA

NRF_SPI_MNGR_DEF(spiManager, ICM20948_QUEUE_LENGTH, ICM20948_SPI_INSTANCE_ID);

static uint8_t cmdBuff[] = { // Write commands use 2 bytes, read uses just 1
    0xff, 0xff};

static uint8_t recvBuff[] = { // Sized based on largest read (22 bytes)
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff};

static nrf_spi_mngr_transfer_t readTransfers[] = {
    NRF_SPI_MNGR_TRANSFER(cmdBuff, 1, recvBuff, 22) // 1 dummy + 6 accel + 6 gyro + 2 temp + 1 magStatus + 6 mag = 22
};

static ret_code_t initSpi0Master(void)
{
    nrf_drv_spi_config_t const mMaster0Config =
        {
            .sck_pin = SER_APP_SPIM0_SCK_PIN,
            .mosi_pin = SER_APP_SPIM0_MOSI_PIN,
            .miso_pin = SER_APP_SPIM0_MISO_PIN,
            .ss_pin = ARDUINO_4_PIN,
            .irq_priority = APP_IRQ_PRIORITY_LOWEST,
            .orc = 0xFF,
            .frequency = NRF_DRV_SPI_FREQ_2M,
            .mode = NRF_DRV_SPI_MODE_0,
            .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST};

    return nrf_spi_mngr_init(&spiManager, &mMaster0Config);
}

/**
 * ICM has 4 register banks.
 */
ret_code_t changeBank(ICMBank_t bank)
{
    ICMReg_t reg = {.reg0 = BANK0_BANK_SEL}; // Arbitrary RegBank because the register is the same in every bank
    uint8_t bitmap[] = {2, 2, 2, 2, 2, 2, 2, 2};
    switch (bank)
    {
    case BANK_0:
        bitmap[4] = 0;
        bitmap[5] = 0;
        break;
    case BANK_1:
        bitmap[4] = 1;
        bitmap[5] = 0;
        break;
    case BANK_2:
        bitmap[4] = 0;
        bitmap[5] = 1;
        break;
    case BANK_3:
        bitmap[4] = 1;
        bitmap[5] = 1;
        break;
    }

    return writeICM(&reg, bitmap);
}

/**
 * Take the ICM out of sleep mode, enabling reading outputs
 */
ret_code_t wakeUpICM(void)
{
    changeBank(BANK_0);
    ICMReg_t pwrMgmt = {.reg0 = PWR_MGMT_1};
    uint8_t bitmask[] = {2, 2, 2, 2, 2, 2, 0, 2};
    return writeICM(&pwrMgmt, bitmask);
}

/**
 * Software reset, not used currently
 */
ret_code_t resetICM(void)
{
    ICMReg_t pwrMgmt = {.reg0 = PWR_MGMT_1};
    uint8_t bitmask[] = {1, 2, 2, 2, 2, 2, 2, 2};
    ret_code_t errCode;
    errCode = writeICM(&pwrMgmt, bitmask);
    while (!nrf_spi_mngr_is_idle(&spiManager))
        ; // Spin until all transactions are complete

    nrf_delay_ms(20);
    uint8_t pwrMgmtData[1];
    synchReadICM(&pwrMgmt, 1, pwrMgmtData);
    while (BIT_MASK(pwrMgmtData[0], 7))
        nrf_delay_ms(10);
    nrf_delay_ms(50);

    return errCode;
}

/**
 * Put the ICM into low power sleep mode, all outputs beside WHOAMI will be 0
 */
ret_code_t sleepICM(void)
{
    ICMReg_t pwrMgmt = {.reg0 = PWR_MGMT_1};
    uint8_t bitmask[] = {2, 2, 2, 2, 2, 2, 1, 2};
    return writeICM(&pwrMgmt, bitmask);
}

/**
 * Setup the USER_CTRL register
 * USER_CTRL - ADDR: 0x03 - BANK: 0x00
 * BIT | NAME       | FUNCTION (if 1 is written)
 * 7   | DMP_EN     | Enable digital motion processing features
 * 6   | FIFO_EN    | Enable writing data into the FIFO
 * 5   | I2C_MST_EN | Enable I2C master mode
 * 4   | I2C_IF_DIS | Reset I2C slave module
 * 3   | DMP_RST    | Reset DMP module
 * 2   | SRAM_RST   | Reset SRAM module
 * 1   | I2C_MST_RST| Reset I2C master mode
 * 0   | ---------- | Reserved
 */
static ret_code_t configUserCtrl(void)
{
    changeBank(BANK_0);
    ICMReg_t config = {.reg0 = USER_CTRL};
    uint8_t bitmask[] = {2, 0, 0, 1, 1, 0, 0, 0};
    return writeICM(&config, bitmask);
}

static ret_code_t configGyro(void)
{
    changeBank(BANK_2);
    ret_code_t errCode;
    ICMReg_t config = {.reg2 = GYRO_CONFIG_1};
    uint8_t bitmask[] = {1, 0, 0, 1, 1, 0, 2, 2}; // 0x19 (top 2 bits are reserved!)
    errCode = writeICM(&config, bitmask);

    if (errCode != NRF_SUCCESS)
        return errCode;

    config.reg2 = GYRO_SMPLRT_DIV;
    uint8_t smplBitmask[] = {0, 0, 1, 0, 0, 0, 0, 0}; // 0x04 - 220Hz sample rate
    return writeICM(&config, smplBitmask);
}

static ret_code_t configAccel(void)
{
    changeBank(BANK_2);
    ret_code_t errCode;
    ICMReg_t config = {.reg2 = ACCEL_CONFIG_1};
    uint8_t accelConfigBitmask[] = {1, 0, 0, 1, 1, 2, 2, 2};
    errCode = writeICM(&config, accelConfigBitmask);

    if (errCode != NRF_SUCCESS)
        return errCode;

    config.reg2 = ACCEL_SMPLRT_DIV_2;
    uint8_t accelSmplBitmask[] = {0, 0, 1, 0, 0, 0, 0, 0};
    return writeICM(&config, accelSmplBitmask);
}

/** 
 * Call all of the needed configuration register writes.
 */
ret_code_t configICM(void)
{
    APP_ERROR_CHECK(configUserCtrl());

    APP_ERROR_CHECK(configGyro());

    APP_ERROR_CHECK(configAccel());

    return NRF_SUCCESS;
}

/**
 * Read a register over SPI on the ICM. Specifically schedules the transaction for as soon as the manager is free,
 * and then it puts the data in a common buffer which is parsed by the end handler
 */
ret_code_t readICM(ICMReg_t *reg, nrf_spi_mngr_transaction_t *transaction, Data_t *outBuffer, void (*endHandler)(ret_code_t, void *))
{

    HandlerParameters_t *params = (HandlerParameters_t *)malloc(sizeof(HandlerParameters_t));
    params->reg1 = (uint8_t)(0x80 | *((uint8_t *)reg));
    params->reg2 = (uint8_t)(0x80 | (*((uint8_t *)reg) + 1));
    params->out = outBuffer;
    params->cmdBuff = cmdBuff;
    params->recvBuff = recvBuff;

    transaction->begin_callback = startReadHandler;
    transaction->end_callback = endHandler;
    transaction->p_transfers = readTransfers;
    transaction->number_of_transfers = 1;
    transaction->p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, transaction);
}

/**
 *  For those who like instant gratification ;)
 */
ret_code_t synchReadICM(ICMReg_t *reg, size_t size, uint8_t *data)
{
    ret_code_t errCode;
    uint8_t readData[size + 1];
    static uint8_t readReg[1];

    readReg[0] = (uint8_t)(0x80 | *((uint8_t *)reg));

    static nrf_spi_mngr_transfer_t transfer[] = {
        NRF_SPI_MNGR_TRANSFER(readReg, 1, NULL, 0)};
    transfer[0].p_rx_data = readData;
    transfer[0].rx_length = size + 1;

    // Have to add error handling
    errCode = nrf_spi_mngr_perform(&spiManager, NULL, transfer, 1, NULL);
    for (int i = 0; i < size; i++)
    {
        data[i] = readData[i + 1];
    }
    return errCode;
}

/**
 * Follows read-modify-write convention, data should be a bitmask of size 8.
 */
ret_code_t writeICM(ICMReg_t *reg, uint8_t *data)
{
    static uint8_t readData[2];
    static uint8_t readReg[1];
    readReg[0] = (uint8_t)(0x80 | *((uint8_t *)reg));
    readData[0] = 0xff;
    readData[1] = 0xff;
    static nrf_spi_mngr_transfer_t readTransfer[] = {
        NRF_SPI_MNGR_TRANSFER(readReg, 1, readData, 2)};

    nrf_spi_mngr_perform(&spiManager, NULL, readTransfer, 1, NULL);

    /* TODO, if we have the value, we can confirm that the new data is different and possibly avoid additional writes */

    HandlerParameters_t *params = (HandlerParameters_t *)malloc(sizeof(HandlerParameters_t));

    params->reg1 = 0x7F & *((uint8_t *)reg); // Turn register into the write version!
    params->reg2 = readData[1];
    params->cmdBuff = cmdBuff;
    params->recvBuff = recvBuff;

    for (int i = 0; i < 8; i++)
    {
        switch (data[i])
        {
        case 0:
            //Clear bit
            params->reg2 &= ~(1 << i);
            break;
        case 1:
            //Set bit
            params->reg2 |= 1 << i;
            break;
        default:
            break;
        }
    }

    static nrf_spi_mngr_transfer_t writeTransfer[] = {
        NRF_SPI_MNGR_TRANSFER(cmdBuff, 2, NULL, 0)};

    static nrf_spi_mngr_transaction_t writeTransaction = {
        .begin_callback = startWriteHandler,
        .end_callback = NULL,
        .p_user_data = NULL,
        .p_transfers = writeTransfer,
        .number_of_transfers = 1,
        .p_required_spi_cfg = NULL};
    writeTransaction.p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, &writeTransaction);
}

static void resetBitmap(uint8_t *bitmask)
{
    for (int i = 0; i < 8; i++)
    {
        bitmask[i] = 2;
    }
}

/**
 * Adapted from drcpattison's awesome example code:
 *  https://github.com/drcpattison/ICM-20948/blob/master/src/ICM20948.cpp#242
 * 
 * It's ugly, it's long, but the device is a pain in the butt to work with so that's how 
 * it is.
 */
ret_code_t calibrateIcm20948(float *outGyroBias, float *outAccelBias)
{
    ret_code_t errCode = NRF_SUCCESS;
    size_t dataSize = 12;
    uint8_t data[dataSize];
    int32_t accelBias[3] = {0, 0, 0}, gyroBias[3] = {0, 0, 0};
    ICMReg_t reg = {.reg0 = PWR_MGMT_1};
    uint8_t bitmask[8] = {2, 2, 2, 2, 2, 2, 2, 1};
    // writeICM(&reg, bitmask);
    // nrf_delay_ms(200);

    bitmask[7] = 2;
    bitmask[0] = 1;
    writeICM(&reg, bitmask);
    nrf_delay_ms(200);

    wakeUpICM();
    // Disable INTs
    reg.reg0 = INT_ENABLE;
    bitmask[0] = 0;
    bitmask[1] = 0;
    bitmask[2] = 0;
    bitmask[3] = 0;
    bitmask[7] = 0;
    writeICM(&reg, bitmask);

    // Disable FIFO
    reg.reg0 = FIFO_EN_1;
    bitmask[7] = 2;
    writeICM(&reg, bitmask);

    reg.reg0 = FIFO_EN_2;
    bitmask[4] = 0;
    writeICM(&reg, bitmask);

    // // Turn on internal clock
    // reg.reg0 = PWR_MGMT_1;
    // resetBitmap(bitmask);
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);

    // Reset DMP and diable I2C Master
    reg.reg0 = USER_CTRL;
    resetBitmap(bitmask);
    bitmask[5] = 0;
    bitmask[6] = 0;
    writeICM(&reg, bitmask);

    resetBitmap(bitmask);
    bitmask[3] = 1;
    writeICM(&reg, bitmask);

    // // Reset FIFO
    // reg.reg0 = FIFO_RST;
    // resetBitmap(bitmask);
    // bitmask[4] = 1;
    // bitmask[3] = 1;
    // bitmask[2] = 1;
    // bitmask[1] = 1;
    // bitmask[0] = 1;
    // writeICM(&reg, bitmask);
    // nrf_delay_ms(10);

    // bitmask[4] = 0;
    // bitmask[3] = 0;
    // bitmask[2] = 0;
    // bitmask[1] = 0;
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);
    // nrf_delay_ms(15);

    // // Set FIFO to snapshot
    // reg.reg0 = FIFO_MODE;
    // bitmask[4] = 1;
    // bitmask[3] = 1;
    // bitmask[2] = 1;
    // bitmask[1] = 1;
    // bitmask[0] = 1;
    // writeICM(&reg, bitmask);

    // changeBank(BANK_2);

    // reg.reg2 = GYRO_CONFIG_1;
    // resetBitmap(bitmask);
    // bitmask[0] = 1;
    // bitmask[1] = 0;
    // bitmask[2] = 0;
    // writeICM(&reg, bitmask);

    // reg.reg2 = GYRO_SMPLRT_DIV;
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);

    // reg.reg2 = GYRO_CONFIG_1;
    // resetBitmap(bitmask);
    // bitmask[0] = 0;
    // bitmask[1] = 0;
    // bitmask[2] = 0;
    // writeICM(&reg, bitmask);

    // reg.reg2 = ACCEL_CONFIG_1;
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);

    uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384; // = 16384 LSB/g

    changeBank(BANK_0);

    // reg.reg0 = USER_CTRL;
    // resetBitmap(bitmask);
    // bitmask[6] = 1;
    // writeICM(&reg, bitmask);

    // reg.reg0 = FIFO_EN_2;
    // resetBitmap(bitmask);
    // bitmask[4] = 1;
    // bitmask[3] = 1;
    // bitmask[2] = 1;
    // bitmask[1] = 1;
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);
    // nrf_delay_ms(40);

    // reg.reg0 = FIFO_EN_2;
    // resetBitmap(bitmask);
    // bitmask[4] = 0;
    // bitmask[3] = 0;
    // bitmask[2] = 0;
    // bitmask[1] = 0;
    // bitmask[0] = 0;
    // writeICM(&reg, bitmask);

    // reg.reg0 = FIFO_COUNTH;
    // uint8_t fifoCount[2];
    // synchReadICM(&reg, 2, fifoCount);
    // uint16_t count = ((uint16_t)(0x1F & fifoCount[0]) << 8) | fifoCount[1];
    // uint16_t packetCount = count / 12;
    uint16_t packetCount = 100;  // arbitrary for now.
    NRF_LOG_INFO("PACKET COUNT: %d", packetCount);
    int16_t accelMax[3] = {-32753, -32753, -32753},
            accelMin[3] = {32753, 32753, 32753},
            gyroMax[3]  = {-32753, -32753, -32753},
            gyroMin[3]  = {32753, 32753, 32753};
    for (int i = 0; i < packetCount; i++)
    {
        int16_t accelTemp[3] = {0, 0, 0}, gyroTemp[3] = {0, 0, 0};

        //reg.reg0 = FIFO_R_W;
        reg.reg0 = ACCEL_X_H;
        synchReadICM(&reg, dataSize, data);

        accelTemp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
        accelTemp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accelTemp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyroTemp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyroTemp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyroTemp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        for (int j = 0; j < 3; j++) {
            if (accelTemp[j] > accelMax[j]) {
                accelMax[j] = accelTemp[j];
            }
            if (gyroTemp[j] > gyroMax[j]) {
                gyroMax[j] = gyroTemp[j];
            }
            if (accelTemp[j] < accelMin[j]) {
                accelMin[j] = accelTemp[j];
            }
            if (gyroTemp[j] < gyroMin[j]) {
                gyroMin[j] = gyroTemp[j];
            }
        }

        // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases.
        accelBias[0] += (int32_t)accelTemp[0];
        accelBias[1] += (int32_t)accelTemp[1];
        accelBias[2] += (int32_t)accelTemp[2];

        // TODO, this is superflous at the moment, and being fazed out in favor of a different approach.
        gyroBias[0] += (int32_t)gyroTemp[0];
        gyroBias[1] += (int32_t)gyroTemp[1];
        gyroBias[2] += (int32_t)gyroTemp[2];
    }
    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accelBias[0] /= (int32_t)packetCount;
    accelBias[1] /= (int32_t)packetCount;
    accelBias[2] /= (int32_t)packetCount;
    // deprecated, slated for removal
    gyroBias[0] /= (int32_t)packetCount;
    gyroBias[1] /= (int32_t)packetCount;
    gyroBias[2] /= (int32_t)packetCount;

    // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    if (accelBias[2] > 0L)
    {
        accelBias[2] -= (int32_t)accelsensitivity;
    }
    else
    {
        accelBias[2] += (int32_t)accelsensitivity;
    }

    for (int i = 0; i < 3; i++) {
        gyroBias[i] = (gyroMax[i] + gyroMin[i]) / 2;
    }


    // Construct the gyro biases for push to the hardware gyro bias registers,
    // which are reset to zero upon device startup.
    // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
    // format.
    data[0] = (-gyroBias[0] / 4 >> 8) & 0xFF;
    // Biases are additive, so change sign on calculated average gyro biases
    data[1] = (-gyroBias[0] / 4) & 0xFF;
    data[2] = (-gyroBias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyroBias[1] / 4) & 0xFF;
    data[4] = (-gyroBias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyroBias[2] / 4) & 0xFF;


    changeBank(BANK_2);

    // Finally push biases into registers
     uint8_t dataBitmask[8] = {2, 2, 2, 2, 2, 2, 2, 2};
    for (int i = 0; i < 6; i++)
    {
        reg.reg2 = XG_OFFSET_H + i;
        for (int j = 0; j < 8; j++)
        {
            dataBitmask[j] = BIT_MASK(data[i], j);
        }
        writeICM(&reg, dataBitmask);
    }

    // Output scaled gyro biases for display in the main program
    outGyroBias[0] = (float)gyroBias[0] / (float)gyrosensitivity;
    outGyroBias[1] = (float)gyroBias[1] / (float)gyrosensitivity;
    outGyroBias[2] = (float)gyroBias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer
    // bias registers. These registers contain factory trim values which must be
    // added to the calculated accelerometer biases; on boot up these registers
    // will hold non-zero values. In addition, bit 0 of the lower byte must be
    // preserved since it is used for temperature compensation calculations.
    // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    changeBank(BANK_1);
    int32_t accelFactoryBias[3] = {0, 0, 0};

    reg.reg1 = XA_OFFSET_H;
    synchReadICM(&reg, 2, data);
    accelFactoryBias[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

    reg.reg1 = YA_OFFSET_H;
    synchReadICM(&reg, 2, data);
    accelFactoryBias[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

    reg.reg1 = ZA_OFFSET_H;
    synchReadICM(&reg, 2, data);
    accelFactoryBias[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
    // Define mask for temperature compensation bit 0 of lower byte of
    // accelerometer bias registers
    uint32_t mask = 1uL;
    // Define array to hold mask bit for each accelerometer bias axis
    uint8_t maskBit[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++)
    {
        // If temperature compensation bit is set, record that fact in mask_bit
        if ((accelFactoryBias[i] & mask))
        {
            maskBit[i] = 0x01;
        }
    }

    // Construct total accelerometer bias, including calculated average
    // accelerometer bias from above
    // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
    // (16 g full scale)
    accelFactoryBias[0] -= (accelBias[0] / 8);
    accelFactoryBias[1] -= (accelBias[1] / 8);
    accelFactoryBias[2] -= (accelBias[2] / 8);

    data[0] = (accelFactoryBias[0] >> 8) & 0xFF;
    data[1] = (accelFactoryBias[0]) & 0xFF;
    // preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[1] = data[1] | maskBit[0];
    data[2] = (accelFactoryBias[1] >> 8) & 0xFF;
    data[3] = (accelFactoryBias[1]) & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[3] = data[3] | maskBit[1];

    data[4] = (accelFactoryBias[2] >> 8) & 0xFF;
    data[5] = (accelFactoryBias[2]) & 0xFF;
    // Preserve temperature compensation bit when writing back to accelerometer
    // bias registers
    data[5] = data[5] | maskBit[2];

    // Push Accel compensation values into the registers
    reg.reg1 = XA_OFFSET_H;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[0], i);
    }
    writeICM(&reg, dataBitmask);

    reg.reg1 = XA_OFFSET_L;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[1], i);
    }
    writeICM(&reg, dataBitmask);

    reg.reg1 = YA_OFFSET_H;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[2], i);
    }
    writeICM(&reg, dataBitmask);

    reg.reg1 = YA_OFFSET_L;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[3], i);
    }
    writeICM(&reg, dataBitmask);

    reg.reg1 = ZA_OFFSET_H;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[4], i);
    }
    writeICM(&reg, dataBitmask);

    reg.reg1 = ZA_OFFSET_L;
    for (int i = 0; i < 8; i++)
    {
        dataBitmask[i] = BIT_MASK(data[5], i);
    }
    writeICM(&reg, dataBitmask);

    // Output scaled accelerometer biases for display in the main program
    outAccelBias[0] = (float)accelBias[0] / (float)accelsensitivity;
    outAccelBias[1] = (float)accelBias[1] / (float)accelsensitivity;
    outAccelBias[2] = (float)accelBias[2] / (float)accelsensitivity;

    return errCode;
}

ret_code_t initIcm20948(void)
{
    float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
    ICMReg_t whoami = {.reg0 = WHO_AM_I};
    APP_ERROR_CHECK(initSpi0Master());
    APP_ERROR_CHECK(wakeUpICM());
    APP_ERROR_CHECK(configICM());
    APP_ERROR_CHECK(calibrateIcm20948(gyroBias, accelBias));
    nrf_delay_ms(50);

    APP_ERROR_CHECK(changeBank(BANK_0));
    uint8_t id[1];
    synchReadICM(&whoami, 1, id);
    if (id[0] != ICM_WHO_AM_I)
    {
        NRF_LOG_INFO("Failed to identify ICM");
    }

    return NRF_SUCCESS;
}