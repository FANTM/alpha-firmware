#include "boards.h"
#include "nrf.h"
#include "nrfx_saadc.h"
#include "nordic_common.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "myoware.h"
#include "data.h"
#include "packet.h"

#define NRF_LOG_MODULE_NAME fantm

#include "nrf_log.h"

#define SAMPLES_IN_BUFFER 16  // Number of times _sample is called before we generate a "finished" callback
#define CHANNEL_COUNT 1

/* Stores the read samples */
static nrf_saadc_value_t m_buffer_pool[CHANNEL_COUNT];
static nrfx_saadc_channel_t channels[CHANNEL_COUNT] = {
    NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN2, 2)
};

static Packet_t *myoPacket = NULL;  // Data packet defined in the main driver. -> myo field needs to be populated

/**
 * Sends a callback to the data polling loop. In this case that callback just samples
 * the ADC, which then triggers its own callback when the sample is collected.
 */
static void initSamplingEvent(void) {
    attachDataChannel((uint32_t) nrfx_saadc_mode_trigger, false);
}

/**
 * Callback from sampling. When we are done sampling the ADC, the data is made available here.
 */ 
static void saadc_callback(nrfx_saadc_evt_t const * p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE)
    {
        ret_code_t err_code;
        int i;
        int accumulator = 0;
        for (i = 0; i < p_event->data.done.size; i++)
        {
            accumulator += p_event->data.done.p_buffer[i];
        }
        accumulator /= p_event->data.done.size;
        myoPacket->myo = (uint16_t) accumulator;

        nrfx_saadc_buffer_set(p_event->data.done.p_buffer, p_event->data.done.size);
        //nrfx_saadc_mode_trigger();
    }
}

/**
 * Initialize the appropriate HAL drivers, and pins for using the ADC
 * In this case we are using pin AIN2 -> PIN 4
 */ 
static void initSaadc(void)
{
    ret_code_t err_code;
    NRF_LOG_INFO("TASK 1");
    //nrfx_saadc_channel_t channel_config =
    //    NRFX_SAADC_DEFAULT_CHANNEL_SE((nrf_saadc_input_t)NRF_SAADC_INPUT_AIN2, 0);
    //NRF_LOG_INFO("TASK 2");

    //err_code = nrfx_saadc_init(NULL, saadc_callback);
    err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("TASK 3");

    err_code = nrfx_saadc_channels_config(channels, CHANNEL_COUNT);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("TASK 4");

    err_code = nrfx_saadc_simple_mode_set(
        (1 << 2), 
        NRFX_SAADC_CONFIG_RESOLUTION,
        NRFX_SAADC_CONFIG_OVERSAMPLE, 
        saadc_callback);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("TASK 5");

    err_code = nrfx_saadc_buffer_set(m_buffer_pool, CHANNEL_COUNT);
    NRF_LOG_INFO("ERR CODE: %d", err_code);

    ///err_code = nrfx_saadc_buffer_set(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    //NRF_LOG_INFO("ERR CODE: %d", err_code);
    //APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("TASK 6");

}

/**
 * Initializes each part of the myoware setup, connecting hardware and handlers.
 */ 
ret_code_t initMyoware(Packet_t *packet) {
    initSaadc();
    myoPacket = packet;
    //nrfx_saadc_mode_trigger();
    initSamplingEvent();
    return NRF_SUCCESS;
}
