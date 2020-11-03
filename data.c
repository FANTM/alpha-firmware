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

#define NUM_CHANNELS 4
#define PERIOD_MS    1000

static void (*dataCallbacks[NUM_CHANNELS])();
static int activeChannels = 0;
static const nrf_drv_timer_t harvestTimer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_ppi_channel_t     channels[NUM_CHANNELS];

static void timerHandler(nrf_timer_event_t event_type, void * p_context)
{
    int i;
    for (i = 0; i < activeChannels; i++) {
        dataCallbacks[i]();
    }
}

void initDataChannels() {
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&harvestTimer, &timer_cfg, timerHandler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 1ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&harvestTimer, PERIOD_MS);
    nrf_drv_timer_extended_compare(&harvestTimer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   true);
    nrf_drv_timer_enable(&harvestTimer);
}

ret_code_t attachDataChannel(uint32_t taskAddr, bool usePPI) {
    if (activeChannels == NUM_CHANNELS) {
        return NRF_ERROR_DATA_SIZE;
    }
    ret_code_t err_code = NRF_SUCCESS;

    CRITICAL_REGION_ENTER();
    if(usePPI) {
        uint32_t timerCompareEventAddr = nrf_drv_timer_compare_event_address_get(&harvestTimer, NRF_TIMER_CC_CHANNEL0);
        err_code = nrf_drv_ppi_channel_alloc(&channels[activeChannels]);
        err_code = nrf_drv_ppi_channel_assign(channels[activeChannels],
                                          timerCompareEventAddr,
                                          taskAddr);
        nrf_drv_ppi_channel_enable(channels[activeChannels]);
    } else {
        dataCallbacks[activeChannels] = (void (*)()) taskAddr;
    }
    activeChannels++;
    CRITICAL_REGION_EXIT();            
    return err_code;                                            
}