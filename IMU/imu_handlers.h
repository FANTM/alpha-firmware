#ifndef __imu_handlers_h__
#define __imu_handlers_h__

void startReadHandler(void * userData);
void startWriteHandler(void *userData);

void genericEndReadHandler(ret_code_t resultCode, void * userData);

void dataHandler(ret_code_t resultCode, void *userData);
void allDataHandler(ret_code_t resultCode, void *userData);

#endif