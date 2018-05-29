#ifndef HELPER_H
#define HELPER_H

#include <stdint.h>
#include <iodef.h>

/****************************************************************************************
* Standalone timer, that does not use any static variables.
* Except for the static counter variable, this timer is identical to the OpenBLT timer.h
* implementation. With the standalone timer, this variable must be stored externally and
* must be given to the saTimerUpdate() function as argument.
****************************************************************************************/
void saTimerInit(void);
void saTimerUpdate(uint32_t *millisecond_counter);
void saTimerReset(void);
/***************************************************************************************/

/****************************************************************************************
* Helper functions that implement a actively polling loop until a specific event occurs.
****************************************************************************************/
void msleep(uint32_t ms);
void waitForSignal(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction state);
uint8_t waitForSignalTimeout(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction state, uint32_t timeout_ms);
/***************************************************************************************/

/****************************************************************************************
* Helper functions that use the LED to signal some states or visualize data.
****************************************************************************************/
void setLed(uint8_t on);
void blinkSOS(uint32_t loops);
void blinkSOSinf(void);
void blinkOK(uint32_t loops);
void blinkOKinf(void);
void visualizeData(uint8_t* data, uint32_t bytes, uint32_t loops);
void visualizeByte(uint8_t byte, uint32_t loops);
/***************************************************************************************/

#endif // HELPER_H
