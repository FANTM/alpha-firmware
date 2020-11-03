
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"

#include "icm20948.h"
#include "data.h"

#define ICM20948_QUEUE_LENGTH     15  // FIXME Need to replace these two values
#define ICM20948_SPI_INSTANCE_ID  0
#define WINDOW_SIZE 16

NRF_SPI_MNGR_DEF(spiManager, ICM20948_QUEUE_LENGTH, ICM20948_SPI_INSTANCE_ID);

typedef enum Bank_t {
    BANK_0,
    BANK_1,
    BANK_2,
    BANK_3
} Bank_t;

typedef enum RegBank0_t {
    WHO_AM_I = 0x00,
    USER_CTRL = 0x03,
    PWR_MGMT_1 = 0x06,
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
    BANK0_BANK_SEL = 0x7F
} RegBank0_t;

typedef enum RegBank1_t {
    BANK1_BANK_SEL = 0x7F
} RegBank1_t;

typedef enum RegBank2_t {
    BANK2_BANK_SEL = 0x7F
} RegBank2_t;

typedef enum RegBank3_t {
    BANK3_BANK_SEL = 0x7F
} RegBank3_t;

typedef union Reg_t {
    RegBank0_t reg0;
    RegBank1_t reg1;
    RegBank2_t reg2;
    RegBank3_t reg3;
} Reg_t;

typedef struct Data_t {
    app_fifo_t accel;
    app_fifo_t gyro;
    app_fifo_t mag;
    app_fifo_t temp;
} Data_t;

typedef struct HandlerParameters_t {
    uint8_t reg1;
    uint8_t reg2;
} HandlerParameters_t;

static ret_code_t writeICM(Reg_t *reg, Bank_t bank, uint8_t *data);
static ret_code_t readICM(Reg_t *reg, Bank_t bank,  nrf_spi_mngr_transaction_t *transaction) ;
static void accelHandler(ret_code_t resultCode, void *userData);

static uint8_t accelBuff[WINDOW_SIZE];
static uint8_t gyroBuff[WINDOW_SIZE];
static uint8_t magBuff[WINDOW_SIZE];
static uint8_t tempBuff[WINDOW_SIZE];

static app_fifo_t accelFifo;
static app_fifo_t gyroFifo;
static app_fifo_t magFifo;
static app_fifo_t tempFifo;

static Data_t dataStore;

static uint8_t cmdBuff[] = {
    0xff, 0xff
};

static uint8_t recvBuff[] = {
    0xff, 0xff, 0xff, 0xff
};

static nrf_spi_mngr_transfer_t readTransfers[] =  {
        NRF_SPI_MNGR_TRANSFER(cmdBuff, 1, recvBuff, 2),
        NRF_SPI_MNGR_TRANSFER(&(cmdBuff[1]), 1, &(recvBuff[2]), 2)
};

static Bank_t activeBank = BANK_0;  // Device boots into bank 0

static ret_code_t initDataStore(void) {
    ret_code_t errCode;

    dataStore.accel = accelFifo;
    dataStore.gyro  = gyroFifo;
    dataStore.mag   = magFifo;
    dataStore.temp  = tempFifo;

    if ((errCode = app_fifo_init(&(dataStore.accel), accelBuff, (uint16_t) sizeof(accelBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.gyro), gyroBuff, (uint16_t) sizeof(gyroBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.mag), magBuff, (uint16_t) sizeof(magBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.temp), tempBuff, (uint16_t) sizeof(tempBuff))) != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static ret_code_t initSpi0Master(void) {
    // SPI0 (with transaction manager) initialization.
    nrf_drv_spi_config_t const mMaster0Config =
    {
        .sck_pin        = SER_APP_SPIM0_SCK_PIN,
        .mosi_pin       = SER_APP_SPIM0_MOSI_PIN,
        .miso_pin       = SER_APP_SPIM0_MISO_PIN,
        .ss_pin         = ARDUINO_4_PIN,
        .irq_priority   = APP_IRQ_PRIORITY_LOWEST,
        .orc            = 0xFF,
        .frequency      = NRF_DRV_SPI_FREQ_4M,
        .mode           = NRF_DRV_SPI_MODE_0,
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    return nrf_spi_mngr_init(&spiManager, &mMaster0Config);
}

static void startReadHandler(void * userData) {
    HandlerParameters_t *params = (HandlerParameters_t *) userData;
    cmdBuff[0] = params->reg1;
    cmdBuff[1] = params->reg2;
}

static void genericEndReadHandler(ret_code_t resultCode, void * userData) {
    NRF_LOG_INFO("Result: %d\n\r", resultCode);
    NRF_LOG_INFO("Returned data: %d %d %d %d", recvBuff[0], recvBuff[1], recvBuff[2], recvBuff[3]);
    free(userData);
    return;
}

static void accelHandler(ret_code_t resultCode, void *userData) {
    int16_t raw = (recvBuff[1] << 8) | recvBuff[3];
    //double adjG = ((double) raw) * 0.00059855; //16384;
    //double accel = adjG * 9.80665;
    NRF_LOG_INFO("raw accel: %d\n\r", raw);
    free(userData);
    return;
}

static void tempHandler(ret_code_t resultCode, void *userData) {
    NRF_LOG_INFO("Result: %d", resultCode);
    NRF_LOG_INFO("Returned data: %d %d %d %d", recvBuff[0], recvBuff[1], recvBuff[2], recvBuff[3]);
    NRF_LOG_INFO("Combined value: %d\n\r", (uint16_t) ((recvBuff[1] << 8) | recvBuff[3]));
    // NRF_LOG_INFO("Temperature: %d")
    free(userData);
}

/**
 * ICM has 4 register banks, along with a 5th for its magnetometer.
 */ 
static ret_code_t changeBank(Bank_t currBank, Bank_t newBank) {
    Reg_t reg;
    reg.reg0 = BANK0_BANK_SEL;  // Arbitrary RegBank because the register is the same in every bank
    if (currBank == newBank)
        return NRF_SUCCESS;

    uint8_t bitmap[] = {2,2,2,2, 2,2,2,2};
    switch(newBank) {
        case BANK_0:
            bitmap[4] = 0;
            bitmap[5] = 0; 
        case BANK_1:
            bitmap[4] = 1;
            bitmap[5] = 0; 
        case BANK_2:
            bitmap[4] = 0;
            bitmap[5] = 1; 
        case BANK_3:
            bitmap[4] = 1;
            bitmap[5] = 1; 
    }
    ret_code_t errCode = writeICM(&reg, currBank, bitmap);
    activeBank = newBank;
    return errCode;
}

/**
 * Take the ICM out of sleep mode, enabling reading outputs
 */
static ret_code_t wakeUpICM(void) {
    Reg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {2,2,2,2,2,2,0,2};
    return writeICM(&pwrMgmt, BANK_0, bitmask);
}

/**
 * Put the ICM into low power sleep mode, all outputs beside WHOAMI will be 0
 */ 
static ret_code_t sleepICM(void) {
    Reg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {2,2,2,2,2,2,1,2};
    return writeICM(&pwrMgmt, BANK_0, bitmask);
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
static ret_code_t configUserCtrl(void) {
    Reg_t config;
    config.reg0 = USER_CTRL;
    uint8_t bitmask[] = {2, 2, 2, 2, 1, 0, 1, 1};
    return writeICM(&config, BANK_0, bitmask);
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
static ret_code_t configICM(void) {
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
static ret_code_t readICM(Reg_t *reg, Bank_t bank,  
                            nrf_spi_mngr_transaction_t *transaction) {
    if (bank != activeBank) {
        changeBank(activeBank, bank);
    }

    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));
    params->reg1 = (uint8_t)(0x80 | *((uint8_t *)reg));
    params->reg2 = (uint8_t)(0x80 | (*((uint8_t *)reg) + 1));

    transaction->p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, transaction);
}

/**
 * Follows read-modify-write convention, data should be a bitmask of size 8.
 */
static ret_code_t writeICM(Reg_t *reg, Bank_t bank, uint8_t *data) {
    if (bank != activeBank) {
        changeBank(activeBank, bank);
    }

    static uint8_t readData[2];
    static uint8_t readReg[1]; 
    readReg[0] = (uint8_t)(0x80 | *((uint8_t *)reg));

    static nrf_spi_mngr_transfer_t readTransfer[] =  {
        NRF_SPI_MNGR_TRANSFER(readReg, 1, readData, 2)
    };
    
    nrf_spi_mngr_perform(&spiManager, NULL, readTransfer, 1, NULL);
    NRF_LOG_INFO("Write Read data: %d %d", readData[0], readData[1]);
    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));
    
    params->reg1 = 0x7F & *((uint8_t *)reg);   // Turn register into the write version!
    params->reg2 = readData[1];
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
        .begin_callback      = startReadHandler,
        .end_callback        = genericEndReadHandler,
        .p_user_data         = NULL,
        .p_transfers         = writeTransfer,
        .number_of_transfers = 1,
        .p_required_spi_cfg  = NULL
    };

    writeTransaction.p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, &writeTransaction);
}

ret_code_t getAccelerationX(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_X_H;
    static nrf_spi_mngr_transaction_t transaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = accelHandler,
        .p_user_data         = NULL,
        .p_transfers         = readTransfers,
        .number_of_transfers = 2,
        .p_required_spi_cfg  = NULL
    };
    return readICM(&accelReg, BANK_0, &transaction);
}

ret_code_t getAccelerationY(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Y_H;
    static nrf_spi_mngr_transaction_t transaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = accelHandler,
        .p_user_data         = NULL,
        .p_transfers         = readTransfers,
        .number_of_transfers = 2,
        .p_required_spi_cfg  = NULL
    };
    return readICM(&accelReg, BANK_0, &transaction);
}

ret_code_t getAccelerationZ(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Z_H;
    static nrf_spi_mngr_transaction_t transaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = accelHandler,
        .p_user_data         = NULL,
        .p_transfers         = readTransfers,
        .number_of_transfers = 2,
        .p_required_spi_cfg  = NULL
    };
    return readICM(&accelReg, BANK_0, &transaction);
}

ret_code_t getTemp(void) {
    Reg_t tempReg;
    tempReg.reg0 = TEMP_H;
    static nrf_spi_mngr_transaction_t transaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = tempHandler,
        .p_user_data         = NULL,
        .p_transfers         = readTransfers,
        .number_of_transfers = 2,
        .p_required_spi_cfg  = NULL
    };
    return readICM(&tempReg, BANK_0, &transaction);
}

/**
 * Pulls in the most recent Acc/Gyro/Mag/Temp readings
 */ 
ret_code_t getAGMT(void) {
    getAccelerationX();
    getAccelerationY();
    getAccelerationZ();
    return NRF_SUCCESS;
}

ret_code_t initIcm20948(void) {
    Reg_t whoami;
    whoami.reg0 = WHO_AM_I;
    APP_ERROR_CHECK(initSpi0Master());
    APP_ERROR_CHECK(initDataStore());
    APP_ERROR_CHECK(wakeUpICM());
    APP_ERROR_CHECK(configICM());
    static nrf_spi_mngr_transaction_t transaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = genericEndReadHandler,
        .p_user_data         = NULL,
        .p_transfers         = readTransfers,
        .number_of_transfers = 1,
        .p_required_spi_cfg  = NULL
    };
    APP_ERROR_CHECK(readICM(&whoami, BANK_0, &transaction));
    attachDataChannel((uint32_t) getAGMT, false);
    return NRF_SUCCESS;
}