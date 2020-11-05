#ifndef __fantm_data_h__
#define __fantm_data_h__

void initDataChannels(void);

ret_code_t attachDataChannel(uint32_t taskAddr, bool usePPI);

void harvestData();

volatile bool printFlag;
volatile bool readFlag;

#endif