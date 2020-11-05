
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "icm20948.h"
#include "data.h"

#define ICM20948_QUEUE_LENGTH     64  // FIXME Need to replace these two values
#define ICM20948_SPI_INSTANCE_ID  0
#define WINDOW_SIZE 16


#define ICM_SPI_TRANSACTION(_begin_callback, _end_callback, _p_transfers, _number_of_transfers)   \
{                                                                               \
    .begin_callback         = (nrf_spi_mngr_callback_begin_t) _begin_callback,  \
    .end_callback           = (nrf_spi_mngr_callback_end_t)   _end_callback,    \
    .p_user_data            = NULL,                                             \
    .p_transfers            = (nrf_spi_mngr_transfer_t const *) _p_transfers,   \
    .number_of_transfers    = (uint8_t) _number_of_transfers,                   \
    .p_required_spi_cfg     = NULL,                                             \
}


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
    app_fifo_t accelX;
    app_fifo_t accelY;
    app_fifo_t accelZ;
    app_fifo_t gyroX;
    app_fifo_t gyroY;
    app_fifo_t gyroZ;
    app_fifo_t mag;
    app_fifo_t temp;
} Data_t;

typedef struct HandlerParameters_t {
    uint8_t reg1;
    uint8_t reg2;
    app_fifo_t *out;
} HandlerParameters_t;

static ret_code_t writeICM(Reg_t *reg, Bank_t bank, uint8_t *data);
static ret_code_t readICM(Reg_t *reg, Bank_t bank,  nrf_spi_mngr_transaction_t *transaction, app_fifo_t *outBuffer);
static void dataHandler(ret_code_t resultCode, void *userData);

static uint8_t accelXBuff[WINDOW_SIZE];
static uint8_t accelYBuff[WINDOW_SIZE];
static uint8_t accelZBuff[WINDOW_SIZE];
static uint8_t gyroXBuff[WINDOW_SIZE];
static uint8_t gyroYBuff[WINDOW_SIZE];
static uint8_t gyroZBuff[WINDOW_SIZE];
static uint8_t magBuff[WINDOW_SIZE];
static uint8_t tempBuff[WINDOW_SIZE];

static app_fifo_t accelXFifo;
static app_fifo_t accelYFifo;
static app_fifo_t accelZFifo;

static app_fifo_t gyroXFifo;
static app_fifo_t gyroYFifo;
static app_fifo_t gyroZFifo;

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

// Reduce via the average, no weighting yet
static int16_t reduceFIFO(app_fifo_t *fifo) {
    ret_code_t errCode;
    uint8_t poppedDataH;
    uint8_t poppedDataL;
    int accumulator = 0;
    int counter = 0;
    CRITICAL_REGION_ENTER();
    while ((errCode = app_fifo_get(fifo, &poppedDataH)) != NRF_ERROR_NOT_FOUND) {
        if (app_fifo_get(fifo, &poppedDataL) != NRF_SUCCESS) {
            break;
        }

        int16_t dataWord = (poppedDataH << 8) | poppedDataL;
        accumulator += dataWord;
        counter++;
    }
    if (counter == 0) {
        CRITICAL_REGION_EXIT();
        return 0;
    } 
        
    int16_t ret = (int16_t) (accumulator / counter);
    CRITICAL_REGION_EXIT();
    return ret; 
}

static ret_code_t initDataStore(void) {
    ret_code_t errCode;

    dataStore.accelX = accelXFifo;
    dataStore.accelY = accelYFifo;
    dataStore.accelZ = accelZFifo;

    dataStore.gyroX  = gyroXFifo;
    dataStore.gyroY  = gyroYFifo;
    dataStore.gyroZ  = gyroZFifo;

    dataStore.mag   = magFifo;
    dataStore.temp  = tempFifo;

    if ((errCode = app_fifo_init(&(dataStore.accelX), accelXBuff, (uint16_t) sizeof(accelXBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.accelY), accelYBuff, (uint16_t) sizeof(accelYBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.accelZ), accelZBuff, (uint16_t) sizeof(accelZBuff))) != NRF_SUCCESS)
        return errCode;
    
    if ((errCode = app_fifo_init(&(dataStore.gyroX), gyroXBuff, (uint16_t) sizeof(gyroXBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.gyroY), gyroYBuff, (uint16_t) sizeof(gyroYBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.gyroZ), gyroZBuff, (uint16_t) sizeof(gyroZBuff))) != NRF_SUCCESS)
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

static void dataHandler(ret_code_t resultCode, void *userData) {
    app_fifo_t *outputFIFO = ((HandlerParameters_t *) userData)->out;
    if (outputFIFO == NULL) {
        if (userData != NULL) {
            free(userData);
        }
        return;
    }
    
    ret_code_t errCode = app_fifo_put(outputFIFO, recvBuff[1]);
    uint8_t poppedData;
    if (errCode == NRF_ERROR_NO_MEM) {
        app_fifo_get(outputFIFO, &poppedData);
        errCode = app_fifo_put(outputFIFO, recvBuff[1]);
    }

    errCode = app_fifo_put(outputFIFO, recvBuff[3]);
    if (errCode == NRF_ERROR_NO_MEM) {
        app_fifo_get(outputFIFO, &poppedData);
        errCode = app_fifo_put(outputFIFO, recvBuff[3]);
    }
    //double adjG = ((double) raw) * 0.00059855; //16384;
    //double accel = adjG * 9.80665;
    //NRF_LOG_INFO("raw accel: %d\n\r", raw);
    free(userData);
    return;
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
    ret_code_t errCode;
    errCode = writeICM(&pwrMgmt, BANK_0, bitmask);
    return errCode;
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
    uint8_t bitmask[] = {2, 2, 2, 1, 1, 0, 1, 1};
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
static ret_code_t readICM(Reg_t *reg, Bank_t bank, nrf_spi_mngr_transaction_t *transaction, app_fifo_t *outBuffer) {
    if (bank != activeBank) {
        changeBank(activeBank, bank);
    }

    HandlerParameters_t *params = (HandlerParameters_t *) malloc(sizeof(HandlerParameters_t));
    params->reg1 = (uint8_t)(0x80 | *((uint8_t *)reg));
    params->reg2 = (uint8_t)(0x80 | (*((uint8_t *)reg) + 1));
    params->out  = outBuffer;

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
    readData[0] = 0xff;
    readData[1] = 0xff;
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
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.accelX));
}

ret_code_t getAccelerationY(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Y_H;
    static nrf_spi_mngr_transaction_t transaction = 
       ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.accelY));
}

ret_code_t getAccelerationZ(void) {
    Reg_t accelReg;
    accelReg.reg0 = ACCEL_Z_H;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.accelZ));
}

ret_code_t getGyroX(void) {
    Reg_t accelReg;
    accelReg.reg0 = GYRO_X_H;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.gyroX));
}

ret_code_t getGyroY(void) {
    Reg_t accelReg;
    accelReg.reg0 = GYRO_Y_H;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.gyroY));
}

ret_code_t getGyroZ(void) {
    Reg_t accelReg;
    accelReg.reg0 = GYRO_Z_H;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&accelReg, BANK_0, &transaction, &(dataStore.gyroZ));
}

ret_code_t getTemp(void) {
    Reg_t tempReg;
    tempReg.reg0 = TEMP_H;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, dataHandler, readTransfers, 2);
    return readICM(&tempReg, BANK_0, &transaction, &(dataStore.temp));
}

ret_code_t getPwrMgmt(void) {
    Reg_t pwrMgmt;
    pwrMgmt.reg0 = USER_CTRL;
    static nrf_spi_mngr_transaction_t transaction = 
        ICM_SPI_TRANSACTION(startReadHandler, genericEndReadHandler, readTransfers, 2);
    return readICM(&pwrMgmt, BANK_0, &transaction, NULL);
}

static int watcher = 1;
/**
 * Pulls in the most recent Acc/Gyro/Mag/Temp readings
 */ 
ret_code_t updateAGMT(void) {
    if (watcher == 0) {     // Addresses Ezra's weird bug. Basically resets the device.
        wakeUpICM();
        configICM();
        nrf_delay_us(200);
    }
    getAccelerationX();
    getAccelerationY();
    getAccelerationZ();
    getGyroX();
    getGyroY();
    getGyroZ();
    return NRF_SUCCESS;
}

void printAGMT(void) {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    accelX = reduceFIFO(&(dataStore.accelX));
    accelY = reduceFIFO(&(dataStore.accelY));
    accelZ = reduceFIFO(&(dataStore.accelZ));
    gyroX = reduceFIFO(&(dataStore.gyroX));
    gyroY = reduceFIFO(&(dataStore.gyroY));
    gyroZ = reduceFIFO(&(dataStore.gyroZ));
    watcher = accelX + accelY + accelZ + gyroX + gyroY + gyroZ;  // Checks if we've crashed

    NRF_LOG_INFO("ACCELX: %d, Y: %d, Z: %d | GYROX: %d, Y: %d, Z: %d\n\r", accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
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
    APP_ERROR_CHECK(readICM(&whoami, BANK_0, &transaction, NULL));
    attachDataChannel((uint32_t) updateAGMT, false);
    return NRF_SUCCESS;
}