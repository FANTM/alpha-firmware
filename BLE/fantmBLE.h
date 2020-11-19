#ifndef __fantm_ble_h__
#define __fantm_ble_h__

ret_code_t initBLE(void);

ret_code_t writeBLE(uint8_t *data, uint16_t *length);

#endif