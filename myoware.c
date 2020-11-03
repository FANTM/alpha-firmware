
#include "boards.h"
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nordic_common.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"
#include "myoware.h"
#include "data.h"
#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"




#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;

static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static uint16_t              readData;



static void initSamplingEvent(void)
{
    attachDataChannel((uint32_t) nrf_drv_saadc_sample, false);
}


static void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;
        int accumulator = 0;
        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            accumulator += p_event->data.done.p_buffer[i];
        }
        readData = accumulator / SAMPLES_IN_BUFFER;
        NRF_LOG_INFO("READ DATA: %d\n\r", readData);
    }
}


static void initSaadc(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}


uint16_t readMyoware(void) {
    return readData;
}

ret_code_t initMyoware(void) {
    initSaadc();
    initSamplingEvent();
    return NRF_SUCCESS;
}
