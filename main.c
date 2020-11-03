/**
 * Copyright (c) 2017 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @file
 * @brief SPI Example Application main file.
 *
 * This file contains the source code for a sample application using nRF SPI manager and nRF GFX.
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

#include "icm20948.h"
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

APP_TIMER_DEF(appTimer);


static void lfclk_config(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}
 
// static void testHandler(void * p_context) {
//     //NRF_LOG_INFO("DATA: %d\n\r", checkData());
//     getAccelerationX();
//     getAccelerationY();
//     getAccelerationZ();
//     uint16_t myo = readMyoware();
//     NRF_LOG_INFO("MyoWare: %d\n\r", myo);
//     return;
// }

static void initTimer(void) {
    ret_code_t errCode;

    errCode = app_timer_init();
    APP_ERROR_CHECK(errCode);
    
 //   errCode = app_timer_create(&appTimer, 
//        APP_TIMER_MODE_REPEATED, 
 //       timerHandler);
    APP_ERROR_CHECK(errCode);

    errCode = app_timer_start(appTimer, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(errCode);

}



int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();
    NRF_LOG_INFO("Start Boot \n\r");
    APP_ERROR_CHECK(nrf_pwr_mgmt_init());

    //lfclk_config();
    // initTimer();
    initDataChannels();
    APP_ERROR_CHECK(initIcm20948());
    initMyoware();
    NRF_LOG_INFO("SPI transaction manager example started. \n\r");
    while (true)
    {
        nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
    }
}

