
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"

#include "icm20948.h"

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

static Bank_t activeBank = BANK_0;  // Device boots into bank 0

static ret_code_t writeICM(Reg_t *reg, Bank_t bank, uint8_t *data);
static ret_code_t readICM(Reg_t *reg, Bank_t bank, bool isWord, void (*handler)());

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
    NRF_LOG_INFO("Result: %d", resultCode);
    NRF_LOG_INFO("Returned data: %d %d %d %d", recvBuff[0], recvBuff[1], recvBuff[2], recvBuff[3]);
    NRF_LOG_INFO("Combined value: %d\n\r", (uint16_t) ((recvBuff[1] << 8) | recvBuff[3]));
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

static ret_code_t changeBank(Bank_t currBank, Bank_t newBank) {
    RegBank0_t reg = BANK0_BANK_SEL;  // Arbitrary RegBank because the register is the same in every bank
    if (currBank == newBank)
        return NRF_SUCCESS;
    return NRF_SUCCESS;
    //writeByte()
}

static ret_code_t wakeUpICM(void) {
    Reg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {2,2,2,2,2,2,0,2};
    writeICM(&pwrMgmt, BANK_0, bitmask);
    return readICM(&pwrMgmt, BANK_0, false, genericEndReadHandler);
}

static ret_code_t sleepICM(void) {
    Reg_t pwrMgmt;
    pwrMgmt.reg0 = PWR_MGMT_1;
    uint8_t bitmask[] = {2,2,2,2,2,2,1,2};
    return writeICM(&pwrMgmt, BANK_0, bitmask);
}

static ret_code_t configICM(void) {
    Reg_t config;
    config.reg0 = USER_CTRL;
    uint8_t bitmask[] = {2, 2, 2, 2, 1, 2, 1, 1};
    return writeICM(&config, BANK_0, bitmask);
}

static ret_code_t readICM(Reg_t *reg, Bank_t bank, bool isWord, void (*handler)()) {
    const int size = isWord ? 2 : 1;
    if (bank != activeBank) {
        changeBank(activeBank, bank);
    }

    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));
    params->reg1 = (uint8_t)(0x80 | *((uint8_t *)reg));
    if (isWord)
        params->reg2 = (uint8_t)(0x80 | (*((uint8_t *)reg) + 1));
    else
        params->reg2 = 0;
    
    static nrf_spi_mngr_transfer_t cmdTransfers[] =  {
        NRF_SPI_MNGR_TRANSFER(cmdBuff, 1, recvBuff, 2),
        NRF_SPI_MNGR_TRANSFER(&(cmdBuff[1]), 1, &(recvBuff[2]), 2)
    };

    static nrf_spi_mngr_transaction_t cmdTransaction = {
        .begin_callback      = startReadHandler,
        .end_callback        = NULL,
        .p_user_data         = NULL,
        .p_transfers         = cmdTransfers,
        .number_of_transfers = NULL,
        .p_required_spi_cfg  = NULL
    };

    cmdTransaction.number_of_transfers = size;
    cmdTransaction.end_callback = handler;
    cmdTransaction.p_user_data = params;

    return nrf_spi_mngr_schedule(&spiManager, &cmdTransaction);
}

/**
 * Follows read-modify-write convention, data should be a bitmask of size 8.
 */
static ret_code_t writeICM(Reg_t *reg, Bank_t bank, uint8_t *data) {
    if (bank != activeBank) {
        changeBank(activeBank, bank);
    }

   
    static uint8_t readData[2];
    static uint8_t cmd[1]; 
    cmd[0] = (uint8_t)(0x80 | *((uint8_t *)reg));

    static nrf_spi_mngr_transfer_t readTransfer[] =  {
        NRF_SPI_MNGR_TRANSFER(cmd, 1, readData, 2)
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
    return readICM(&accelReg, BANK_0, true, accelHandler);
}

ret_code_t getAccelerationY(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Y_H;
    return readICM(&accelReg, BANK_0, true, accelHandler);
}

ret_code_t getAccelerationZ(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Z_H;
    return readICM(&accelReg, BANK_0, true, accelHandler);
}

ret_code_t getTemp(void) {
    Reg_t tempReg;
    tempReg.reg0 = TEMP_H;
    return readICM(&tempReg, BANK_0, true, tempHandler);
}

ret_code_t initIcm20948(void) {
    Reg_t whoami;
    whoami.reg0 = WHO_AM_I;
    APP_ERROR_CHECK(initSpi0Master());
    APP_ERROR_CHECK(initDataStore());
    APP_ERROR_CHECK(wakeUpICM());
    APP_ERROR_CHECK(configICM());
    APP_ERROR_CHECK(readICM(&whoami, BANK_0, false, genericEndReadHandler));
    return NRF_SUCCESS;
}