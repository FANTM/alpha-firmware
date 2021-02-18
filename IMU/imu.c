#include "app_util_platform.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#include "icm20948.h"
#include "ak09916.h"
#include "imu_handlers.h"
#include "data.h"
#include "packet.h" 

#define NRF_LOG_MODULE_NAME FANTM_IMU
#if (FANTM_IMU_LOG_ENABLED)
#define NRF_LOG_LEVEL       6
#else
#define NRF_LOG_LEVEL       0
#endif
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


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
static Packet_t *imuPacket = NULL;

// Reduce via the average, no weighting yet
static int16_t reduceFIFO(app_fifo_t *fifo) {
    ret_code_t errCode;
    uint8_t poppedDataH;
    uint8_t poppedDataL;
    int accumulator = 0;
    int counter = 0;
    int16_t ret = 0;
    CRITICAL_REGION_ENTER();
    while ((errCode = app_fifo_get(fifo, &poppedDataH)) != NRF_ERROR_NOT_FOUND) {
        if (app_fifo_get(fifo, &poppedDataL) != NRF_SUCCESS) {
            break;
        }

        int16_t dataWord = ((int16_t) poppedDataH << 8) | poppedDataL;
        accumulator += dataWord;
        counter++;
    }
    if (counter != 0) {
        ret = (int16_t) (accumulator / counter);
    } 
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

    return errCode;
}

/**
 * Take the ICM out of sleep mode, enabling reading outputs
 */
ret_code_t wakeUpICM(void)
{
    changeBank(BANK_0);
    ICMReg_t pwrMgmt = PWR_MGMT_1;
    uint8_t bitmask[] = {2, 2, 2, 2, 2, 2, 0, 2};
    return writeICM(&pwrMgmt, bitmask);
}

/**
 * Put the ICM into low power sleep mode, all outputs beside WHOAMI will be 0
 */
ret_code_t sleepICM(void)
{
    ICMReg_t pwrMgmt = PWR_MGMT_1;
    uint8_t bitmask[] = {2, 2, 2, 2, 2, 2, 1, 2};
    return writeICM(&pwrMgmt, bitmask);
}

/**
 * Pulls in the most recent Acc/Gyro/Mag/Temp readings
 */ 
ret_code_t updateAGMT(void) {
    return readDataICM(&dataStore, allDataHandler);
}

void reduceAGMT() {
    imuPacket->accelX = reduceFIFO(&(dataStore.accelX));
    imuPacket->accelY = reduceFIFO(&(dataStore.accelY));
    imuPacket->accelZ = reduceFIFO(&(dataStore.accelZ));
    imuPacket->gyroX = reduceFIFO(&(dataStore.gyroX));
    imuPacket->gyroY = reduceFIFO(&(dataStore.gyroY));
    imuPacket->gyroZ = reduceFIFO(&(dataStore.gyroZ));
    imuPacket->magX = reduceFIFO(&(dataStore.magX));
    imuPacket->magY = reduceFIFO(&(dataStore.magY));
    imuPacket->magZ = reduceFIFO(&(dataStore.magZ));
    imuPacket->temp = reduceFIFO(&(dataStore.temp));
}

ret_code_t initIMU(Packet_t *packet) {
    APP_ERROR_CHECK(initDataStore());
    APP_ERROR_CHECK(initIcm20948());
    APP_ERROR_CHECK(initAK09916());
    changeBank(BANK_0);  // After configuration we don't have much reason to leave BANK_0
    imuPacket = packet;
    attachDataChannel((uint32_t) updateAGMT, false);
    return NRF_SUCCESS;
}