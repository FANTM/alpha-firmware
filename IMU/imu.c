#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#include "icm20948.h"
#include "ak09916.h"
#include "imu_handlers.h"
#include "data.h"
#include "packet.h" 

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define WINDOW_SIZE 16

static uint8_t accelXBuff[WINDOW_SIZE];
static uint8_t accelYBuff[WINDOW_SIZE];
static uint8_t accelZBuff[WINDOW_SIZE];

static uint8_t gyroXBuff[WINDOW_SIZE];
static uint8_t gyroYBuff[WINDOW_SIZE];
static uint8_t gyroZBuff[WINDOW_SIZE];

static uint8_t magXBuff[WINDOW_SIZE];
static uint8_t magYBuff[WINDOW_SIZE];
static uint8_t magZBuff[WINDOW_SIZE];

static uint8_t tempBuff[WINDOW_SIZE];

static app_fifo_t accelXFifo;
static app_fifo_t accelYFifo;
static app_fifo_t accelZFifo;

static app_fifo_t gyroXFifo;
static app_fifo_t gyroYFifo;
static app_fifo_t gyroZFifo;

static app_fifo_t magXFifo;
static app_fifo_t magYFifo;
static app_fifo_t magZFifo;

static app_fifo_t tempFifo;

static Data_t dataStore;

// Reduce via the average, no weighting yet
static int16_t reduceFIFO(app_fifo_t *fifo) {
    ret_code_t errCode;
    uint8_t poppedDataH;
    uint8_t poppedDataL;
    int accumulator = 0;
    int counter = 0;
    //CRITICAL_REGION_ENTER();
    while ((errCode = app_fifo_get(fifo, &poppedDataH)) != NRF_ERROR_NOT_FOUND) {
        if (app_fifo_get(fifo, &poppedDataL) != NRF_SUCCESS) {
            break;
        }

        int16_t dataWord = (poppedDataH << 8) | poppedDataL;
        accumulator += dataWord;
        counter++;
    }
    if (counter == 0) {
        //CRITICAL_REGION_EXIT();
        return 0;
    } 
        
    int16_t ret = (int16_t) (accumulator / counter);
    //CRITICAL_REGION_EXIT();
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

    dataStore.magX   = magXFifo;
    dataStore.magY   = magYFifo;
    dataStore.magZ   = magZFifo;

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
    
    if ((errCode = app_fifo_init(&(dataStore.magX), magXBuff, (uint16_t) sizeof(magXBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.magY), magYBuff, (uint16_t) sizeof(magYBuff))) != NRF_SUCCESS)
        return errCode;
    if ((errCode = app_fifo_init(&(dataStore.magZ), magZBuff, (uint16_t) sizeof(magZBuff))) != NRF_SUCCESS)
        return errCode;

    if ((errCode = app_fifo_init(&(dataStore.temp), tempBuff, (uint16_t) sizeof(tempBuff))) != NRF_SUCCESS)
        return errCode;

    return NRF_SUCCESS;
}

static ret_code_t readData(void) {
    ICMReg_t reg = {.reg0 = ACCEL_X_H};
    static nrf_spi_mngr_transaction_t transaction;
    return readICM(&reg, &transaction, &dataStore, allDataHandler);
}

ret_code_t getPwrMgmt(void) {
    ICMReg_t pwrMgmt = {.reg0 = PWR_MGMT_1};
    static nrf_spi_mngr_transaction_t transaction;
    return readICM(&pwrMgmt, &transaction, NULL, genericEndReadHandler);
}

/**
 * Pulls in the most recent Acc/Gyro/Mag/Temp readings
 */ 
ret_code_t updateAGMT(void) {
    return readData();
}

void printAGMT(void) {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ; 
    int16_t magX, magY, magZ;
    accelX = reduceFIFO(&(dataStore.accelX));
    accelY = reduceFIFO(&(dataStore.accelY));
    accelZ = reduceFIFO(&(dataStore.accelZ));
    gyroX = reduceFIFO(&(dataStore.gyroX));
    gyroY = reduceFIFO(&(dataStore.gyroY));
    gyroZ = reduceFIFO(&(dataStore.gyroZ));
    magX = reduceFIFO(&(dataStore.magX));
    magY = reduceFIFO(&(dataStore.magY));
    magZ = reduceFIFO(&(dataStore.magZ));
    NRF_LOG_INFO("START========================");
    NRF_LOG_INFO("ACCEL,%d,%d,%d", accelX, accelY, accelZ);
    NRF_LOG_INFO("GYRO,%d,%d,%d",  gyroX, gyroY, gyroZ);
    NRF_LOG_INFO("MAG,%d,%d,%d", magX, magY, magZ);
    NRF_LOG_INFO("END==========================\n");
}

void getAGMT(Packet_t *packet) {
    packet->accelX = reduceFIFO(&(dataStore.accelX));
    packet->accelY = reduceFIFO(&(dataStore.accelY));
    packet->accelZ = reduceFIFO(&(dataStore.accelZ));
    packet->gyroX = reduceFIFO(&(dataStore.gyroX));
    packet->gyroY = reduceFIFO(&(dataStore.gyroY));
    packet->gyroZ = reduceFIFO(&(dataStore.gyroZ));
    packet->magX = reduceFIFO(&(dataStore.magX));
    packet->magY = reduceFIFO(&(dataStore.magY));
    packet->magZ = reduceFIFO(&(dataStore.magZ));
}

ret_code_t initIMU() {
    APP_ERROR_CHECK(initDataStore());
    APP_ERROR_CHECK(initIcm20948());
    APP_ERROR_CHECK(initAK09916());
    changeBank(BANK_0);  // After configuration we don't have much reason to leave BANK_0
    attachDataChannel((uint32_t) updateAGMT, false);
    return NRF_SUCCESS;
}