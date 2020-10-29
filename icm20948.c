
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"

#include "nrf_log.h"

#include "icm20948.h"

#define ICM20948_QUEUE_LENGTH     15  // FIXME Need to replace these two values
#define ICM20948_SPI_INSTANCE_ID  0

NRF_SPI_MNGR_DEF(spiManager, ICM20948_QUEUE_LENGTH, ICM20948_SPI_INSTANCE_ID);

typedef enum RegBank0_t {
    WHO_AM_I = 0x00,
    ACCEL_X_H = 0x29,
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
    TEMP_L
} RegBank0_t;

static NRF_SPI_MNGR_BUFFER_LOC_IND uint8_t commandBuff[] = {
    (uint8_t) (0x80 | WHO_AM_I)
};

static volatile NRF_SPI_MNGR_BUFFER_LOC_IND uint8_t recvData[] = {
    0xff, 0xff
};

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

static void spiStartTransactionHandler(void * p_user_data) {
    NRF_LOG_INFO("START!");
    //nrf_gpio_pin_clear(ARDUINO_4_PIN);
    return;
}

static void spiEndTransactionHandler(ret_code_t resultCode, void * p_user_data) {
    //nrf_gpio_pin_set(ARDUINO_4_PIN);
    NRF_LOG_INFO("WHO AM I? => %d %d", recvData[0], recvData[1]);
    NRF_LOG_INFO("RET CODE => %d", resultCode);
    return;
}

static ret_code_t readRegister(void) {
    static nrf_spi_mngr_transfer_t NRF_SPI_MNGR_BUFFER_LOC_IND cmdTransfers[] = {
        NRF_SPI_MNGR_TRANSFER(
            commandBuff, sizeof(commandBuff),
            recvData, sizeof(recvData)
        )
    };

    static nrf_spi_mngr_transaction_t const cmdTransaction = {
            .begin_callback      = spiStartTransactionHandler,
            .end_callback        = spiEndTransactionHandler,
            .p_user_data         = NULL,
            .p_transfers         = cmdTransfers,
            .number_of_transfers = sizeof(cmdTransfers) / sizeof(cmdTransfers[0]),
            .p_required_spi_cfg  = NULL
    };

    return nrf_spi_mngr_schedule(&spiManager, &cmdTransaction);
}

uint8_t checkData(void) {
    return recvData[0];
}

ret_code_t initIcm20948(void) {
    /* TODO Delete this block, just to trick the compiler warning */
    RegBank0_t reg = WHO_AM_I;
    if (reg == WHO_AM_I);
    /***/
    APP_ERROR_CHECK(initSpi0Master());
    APP_ERROR_CHECK(readRegister());
    return NRF_SUCCESS;
}