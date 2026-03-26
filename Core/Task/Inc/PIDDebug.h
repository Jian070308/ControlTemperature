#ifndef CONTROLTEMPERATURE_PIDDEBUG_H
#define CONTROLTEMPERATURE_PIDDEBUG_H

#include <stdint.h>

void PID_Debug_Init(void);
void PID_Debug_RxCpltCallback(uint8_t byte);
void PID_Debug_Process(void);

#endif //CONTROLTEMPERATURE_PIDDEBUG_H
