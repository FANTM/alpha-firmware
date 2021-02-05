#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"
#include "app_fifo.h"

#include "icm20948.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

void debugEndHandler(ret_code_t resultCode, void * userData) {
    NRF_LOG_INFO("Result: %d\n\r", resultCode);
    HandlerParameters_t *params = (HandlerParameters_t *) userData;
    NRF_LOG_INFO("Returned data: %d %d", params->recvBuff[0], params->recvBuff[1]);
    return;
}

// Helper for adding new data to the fifo, since it requires a lot of checks
static ret_code_t safeAppFifoPut(app_fifo_t *fifo, uint8_t data) {
    ret_code_t errCode = app_fifo_put(fifo, data);
    uint8_t poppedData;
    if (errCode == NRF_ERROR_NO_MEM) {
        app_fifo_get(fifo, &poppedData);    // Dummy pop to clear space
        errCode = app_fifo_put(fifo, data);
    }
    return errCode;
}

void allDataHandler(ret_code_t resultCode, void *userData) {
    HandlerParameters_t *params = (HandlerParameters_t *) userData;
    if (params == NULL) {
        return;
    }
    
    Data_t *dataStore = params->out;
    ret_code_t errCode;
    errCode = safeAppFifoPut(&dataStore->accelX, params->recvBuff[1]);  // ACCELX_H
    if (errCode != NRF_SUCCESS) {  // Lazy testing, if we just test a couple we will likely catch some kind of system error
        NRF_LOG_INFO("ERROR!");
    }
    safeAppFifoPut(&dataStore->accelX, params->recvBuff[2]);  // ACCELX_L
    safeAppFifoPut(&dataStore->accelY, params->recvBuff[3]);  // ACCELY_H
    safeAppFifoPut(&dataStore->accelY, params->recvBuff[4]);  // ACCELY_L
    safeAppFifoPut(&dataStore->accelZ, params->recvBuff[5]);  // ACCELZ_H
    errCode = safeAppFifoPut(&dataStore->accelZ, params->recvBuff[6]);  // ACCELZ_L
    if (errCode != NRF_SUCCESS) {  
        NRF_LOG_INFO("ERROR!");
    }
    safeAppFifoPut(&dataStore->gyroX, params->recvBuff[7]);   // GYROX_H
    safeAppFifoPut(&dataStore->gyroX, params->recvBuff[8]);   // GYROX_L
    safeAppFifoPut(&dataStore->gyroY, params->recvBuff[9]);   // GYROY_H
    safeAppFifoPut(&dataStore->gyroY, params->recvBuff[10]);  // GYROY_L
    safeAppFifoPut(&dataStore->gyroZ, params->recvBuff[11]);  // GYROZ_H
    safeAppFifoPut(&dataStore->gyroZ, params->recvBuff[12]);  // GYROZ_L

    safeAppFifoPut(&dataStore->temp, params->recvBuff[13]);   // TEMP_H
    safeAppFifoPut(&dataStore->temp, params->recvBuff[14]);   // TEMP_L

    // Reverse insertion order for Mag because it's big endian
    safeAppFifoPut(&dataStore->magX, params->recvBuff[17]);   // MAGX_H
    safeAppFifoPut(&dataStore->magX, params->recvBuff[16]);   // MAGX_L
    safeAppFifoPut(&dataStore->magY, params->recvBuff[19]);   // MAGY_H
    safeAppFifoPut(&dataStore->magY, params->recvBuff[18]);   // MAGY_L
    safeAppFifoPut(&dataStore->magZ, params->recvBuff[21]);   // MAGZ_H
    safeAppFifoPut(&dataStore->magZ, params->recvBuff[20]);   // MAGZ_L

}
