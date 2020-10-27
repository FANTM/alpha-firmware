
#include "app_util_platform.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_spi_mngr.h"
#include "boards.h"
#include "nordic_common.h"

#define ICM20948_QUEUE_LENGTH     15  // FIXME Need to replace these two values
#define ICM20948_SPI_INSTANCE_ID  0

NRF_SPI_MNGR_DEF(m_nrf_spi_mngr, ICM20948_QUEUE_LENGTH, ICM20948_SPI_INSTANCE_ID);

static typedef enum RegBank0_t {
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
    TEMP_L,
} RegBank0_t;

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
        .frequency      = NRF_DRV_SPI_FREQ_8M,
        .mode           = NRF_DRV_SPI_MODE_0,
        .bit_order      = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
    };

    return nrf_spi_mngr_init(&m_nrf_spi_mngr, &mMaster0Config);
}

static uint8_t readRegister(void) {
    return 0;
}

ret_code_t initIcm20948(void) {
    APP_ERROR_CHECK(initSpi0Master());
}