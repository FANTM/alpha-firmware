/** @file
 * @brief FANTM master driver file
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "app_error.h"
#include "nrf_spi_mngr.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "data.h"
#include "nrf_log_default_backends.h"

#include "imu.h"
#include "myoware.h"

#define NRF_LOG_MODULE_NAME fantm
#define NRF_LOG_LEVEL       6
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
NRF_LOG_MODULE_REGISTER();


#if defined( __GNUC__ ) && (__LINT__ == 0)
    // This is required if one wants to use floating-point values in 'printf'
    // (by default this feature is not linked together with newlib-nano).
    // Please note, however, that this adds about 13 kB code footprint...
    __ASM(".global _printf_float");
#endif

int main(void) {
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());
    initDataChannels();
    APP_ERROR_CHECK(initIMU());
    initMyoware();
    NRF_LOG_INFO("FANTM Alpha firmware initialized. \n\r");
    while (true)
    {
        if (printFlag) {
            printAGMT();
            printFlag = false;
        }
        if (readFlag) {
            updateAGMT();
            readFlag = false;
        }
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}

