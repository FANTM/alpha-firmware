
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

#define SAMPLES_IN_BUFFER 1  // Number of times _sample is called before we generate a "finished" callback

/* Stores the read samples */
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];

/**
 * Sends a callback to the data polling loop. In this case that callback just samples
 * the ADC, which then triggers its own callback when the sample is collected.
 */
static void initSamplingEvent(void) {
    attachDataChannel((uint32_t) nrf_drv_saadc_sample, false);
}

/**
 * Callback from sampling. When we are done sampling the ADC, the data is made available here.
 */ 
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
        accumulator /= SAMPLES_IN_BUFFER;
        NRF_LOG_INFO("READ DATA: %d\n\r", accumulator);
    }
}

/**
 * Initialize the appropriate HAL drivers, and pins for using the ADC
 * In this case we are using pin A0 -> 2
 */ 
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

/**
 * Initializes each part of the myoware setup, connecting hardware and handlers.
 */ 
ret_code_t initMyoware(void) {
    initSaadc();
    initSamplingEvent();
    return NRF_SUCCESS;
}
