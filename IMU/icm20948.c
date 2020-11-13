
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

#define ICM20948_QUEUE_LENGTH     64  
#define ICM20948_SPI_INSTANCE_ID  0
 
NRF_SPI_MNGR_DEF(spiManager, ICM20948_QUEUE_LENGTH, ICM20948_SPI_INSTANCE_ID);

static uint8_t cmdBuff[] = {  // Write commands use 2 bytes, read uses just 1
    0xff, 0xff
};

static uint8_t recvBuff[] = {  // Sized based on largest read (22 bytes)
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff,
    0xff, 0xff
};

static nrf_spi_mngr_transfer_t readTransfers[] =  {
        NRF_SPI_MNGR_TRANSFER(cmdBuff, 1, recvBuff, 22)  // 1 dummy + 6 accel + 6 gyro + 2 temp + 1 magStatus + 6 mag = 22
};

static ret_code_t initSpi0Master(void) {
    nrf_drv_spi_config_t const mMaster0Config =
    {
        .sck_pin        = SER_APP_SPIM0_SCK_PIN,
        .mosi_pin       = SER_APP_SPIM0_MOSI_PIN,
        .miso_pin       = SER_APP_SPIM0_MISO_PIN,
        .ss_pin         = ARDUINO_4_PIN,
        .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
        .orc            = 0xFF,
        .frequency      = NRF_DRV_SPI_FREQ_2M,
        .mode           = NRF_DRV_SPI_MODE_0,
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    return nrf_spi_mngr_init(&spiManager, &mMaster0Config);
}


/**
 * ICM has 4 register banks.
 */ 
ret_code_t changeBank(ICMBank_t bank) {
    ICMReg_t reg = {.reg0 = BANK0_BANK_SEL}; // Arbitrary RegBank because the register is the same in every bank
    uint8_t bitmap[] = {2,2,2,2,2,2,2,2};
    switch(bank) {
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

    ret_code_t errCode = writeICM(&reg, bitmap);
    //while(!nrf_spi_mngr_is_idle(&spiManager));  // Spin until all transactions are complete, we want to make sure the bank is switched.
    return errCode;
}

/**
 * Take the ICM out of sleep mode, enabling reading outputs
 */
ret_code_t wakeUpICM(void) {
    ICMReg_t pwrMgmt = {.reg0 = PWR_MGMT_1};
    uint8_t bitmask[] = {2,2,2,2,2,2,0,2};
    ret_code_t errCode;
    errCode = writeICM(&pwrMgmt, bitmask);
    return errCode;
}

/**
 * Take the ICM out of sleep mode, enabling reading outputs
 */
ret_code_t resetICM(void) {
    ICMReg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {1,2,2,2,2,2,2,2};
    ret_code_t errCode;
    errCode = writeICM(&pwrMgmt, bitmask);
    while(!nrf_spi_mngr_is_idle(&spiManager));  // Spin until all transactions are complete
    
    nrf_delay_ms(20);
    while (BIT_MASK(synchReadICM(&pwrMgmt), 7))
        nrf_delay_ms(10);
    nrf_delay_ms(50);

    return errCode;
}

/**
 * Put the ICM into low power sleep mode, all outputs beside WHOAMI will be 0
 */ 
ret_code_t sleepICM(void) {
    ICMReg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {2,2,2,2,2,2,1,2};
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
ret_code_t configUserCtrl(void) {
    ICMReg_t config;
    config.reg0 = USER_CTRL;
    uint8_t bitmask[] = {2, 0, 0, 1, 1, 1, 0, 0};
    return writeICM(&config, bitmask);
}

// TODO Configure FIFO
// static ret_code_t configFifo(void) {
//     Reg_t config;
//     config.reg0 = USER_CTRL;
//     uint8_t bitmask[] = {2, 2, 2, 2, 1, 2, 1, 1};
//     return writeICM(&config, BANK_0, bitmask);
// }

/** 
 * Call all of the needed configuration register writes.
 */
ret_code_t configICM(void) {
    ret_code_t errCode;

    if ((errCode = configUserCtrl()) != NRF_SUCCESS) 
        return errCode;
    
    return errCode;
}

/**
 * Read a register over SPI on the ICM. 
 * Takes in a reg Union, and the bank to do the bank management for the user,
 * as well as a transaction 
 */
ret_code_t readICM(ICMReg_t *reg, nrf_spi_mngr_transaction_t *transaction, Data_t *outBuffer, void (*endHandler)(ret_code_t, void *)) {

    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));
    params->reg1 = (uint8_t)(0x80 | *((uint8_t *)reg));
    params->reg2 = (uint8_t)(0x80 | (*((uint8_t *)reg) + 1));
    params->out  = outBuffer;
    params->cmdBuff = cmdBuff;
    params->recvBuff = recvBuff;

    transaction->begin_callback = startReadHandler;
    transaction->end_callback = endHandler;
    transaction->p_transfers = readTransfers;
    transaction->number_of_transfers = 1;
    transaction->p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, transaction);
}

uint8_t synchReadICM(ICMReg_t *reg) {

    static uint8_t readData[] = {0xff, 0xff};
    static uint8_t readReg[1]; 
    
    readReg[0] = (uint8_t)(0x80 | *((uint8_t *)reg));
    
    static nrf_spi_mngr_transfer_t transfer[] =  {
        NRF_SPI_MNGR_TRANSFER(readReg, 1, readData, 2)
    };
    
    // Have to add error handling
    nrf_spi_mngr_perform(&spiManager, NULL, transfer, 1, NULL);

    return readData[1];
}

/**
 * Follows read-modify-write convention, data should be a bitmask of size 8.
 */
ret_code_t writeICM(ICMReg_t *reg, uint8_t *data) {
    static uint8_t readData[2];
    static uint8_t readReg[1]; 
    readReg[0] = (uint8_t)(0x80 | *((uint8_t *)reg));
    readData[0] = 0xff;
    readData[1] = 0xff;
    static nrf_spi_mngr_transfer_t readTransfer[] =  {
        NRF_SPI_MNGR_TRANSFER(readReg, 1, readData, 2)
    };
    
    nrf_spi_mngr_perform(&spiManager, NULL, readTransfer, 1, NULL);

    /* TODO, if we have the value, we can confirm that the new data is different and possibly avoid additional writes */

    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));

    params->reg1 = 0x7F & *((uint8_t *)reg);   // Turn register into the write version!
    params->reg2 = readData[1];
    params->cmdBuff = cmdBuff;
    params->recvBuff = recvBuff;

    for (int i = 0; i < 8; i++) {
        switch(data[i]) {
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

    static nrf_spi_mngr_transfer_t writeTransfer[] =  {
        NRF_SPI_MNGR_TRANSFER(cmdBuff, 2, NULL, 0)
    };

    static nrf_spi_mngr_transaction_t writeTransaction = {
        .begin_callback      = startWriteHandler,
        .end_callback        = NULL,
        .p_user_data         = NULL,
        .p_transfers         = writeTransfer,
        .number_of_transfers = 1,
        .p_required_spi_cfg  = NULL
    };
    writeTransaction.p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, &writeTransaction);
}


ret_code_t initIcm20948(void) {
    ICMReg_t whoami = {.reg0 = WHO_AM_I};
    APP_ERROR_CHECK(initSpi0Master());
    APP_ERROR_CHECK(changeBank(BANK_0));
    //APP_ERROR_CHECK(resetICM());
    APP_ERROR_CHECK(wakeUpICM());
    APP_ERROR_CHECK(configICM());
    NRF_LOG_INFO("==WHOAMI: %d==", synchReadICM(&whoami));
    return NRF_SUCCESS;
}