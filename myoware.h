#ifndef __myoware_h__
#define __myoware_h__

#include "packet.h"

/**
 * Initialize a constantly sampling ADC that will update an internal record of its value
 */
ret_code_t initMyoware(Packet_t *packet);

/**
 * Gets the most recent reading from the SAADC.
 */
uint16_t readMyoware(void);
#endif