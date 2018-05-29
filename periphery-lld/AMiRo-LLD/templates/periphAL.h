/*
AMiRo-LLD is a compilation of low-level hardware drivers for the Autonomous Mini Robot (AMiRo) platform.
Copyright (C) 2016..2018  Thomas Schöpping et al.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _AMIROLLD_PERIPHAL_H_
#define _AMIROLLD_PERIPHAL_H_

/*============================================================================*/
/* VERSION                                                                    */
/*============================================================================*/

/**
 * @brief   The periphery abstraction layer interface major version.
 * @note    Changes of the major version imply incompatibilities.
 */
#define PERIPHAL_VERSION_MAJOR    1

/**
 * @brief   The periphery abstraction layer interface minor version.
 * @note    A higher minor version implies new functionalty, but all old interfaces are still available.
 */
#define PERIPHAL_VERSION_MINOR    0

/*============================================================================*/
/* DEPENDENCIES                                                               */
/*============================================================================*/

#include <periphALtypes.h>

/*============================================================================*/
/* GENERAL                                                                    */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Delay execution by a specific number of microseconds.
 *
 * @param[in]   us    Time to sleep until execution continues in microseconds.
 */
void usleep(apalTime_t us);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* GPIO                                                                       */
/*============================================================================*/

typedef osGpio_t apalGpio_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Read the current value of a GPIO pin.
 *
 * @param[in]   gpio  GPIO to read.
 * @param[out]  val   Current value of the GPIO.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalGpioRead(apalGpio_t* gpio, apalGpioState_t* const val);

/**
 * @brief Set the value of a GPIO pin.
 *
 * @param[in] gpio  GPIO to write.
 * @param[in] val   Value to set for the GPIO.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalGpioWrite(apalGpio_t* gpio, const apalGpioState_t val);

/**
 * @brief Toggle the output of a GPIO.
 *
 * @param[in] gpio  GPIO to toggle.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalGpioToggle(apalGpio_t* gpio);

/**
 * @brief Get the current on/off state of a control GPIO.
 *
 * @param[in]   gpio  Control GPIO to read.
 * @param[out]  val   Current activation status of the control GPIO.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalControlGpioGet(const apalControlGpio_t* const cgpio, apalControlGpioState_t* const val);

/**
 * @brief Turn a control GPIO 'on' or 'off' respectively.
 *
 * @param[in] gpio  Control GPIO to set.
 * @param[in] val   Activation value to set for the control GPIO.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalControlGpioSet(const apalControlGpio_t* const cgpio, const apalControlGpioState_t val);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* PWM                                                                        */
/*============================================================================*/

/**
 * @brief PWM driver type.
 */
typedef osPWMDriver_t apalPWMDriver_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set the PWM with given parameters.
 *
 * @param[in] pwm       PWM driver to set.
 * @param[in] channel   Channel of the PWM driver to set.
 * @param[in] width     Width to set the channel to.
 *
 * @return  The status indicates whether the function call was successful.
 */
apalExitStatus_t apalPWMSet(apalPWMDriver_t* pwm, const apalPWMchannel_t channel, const apalPWMwidth_t width);

/**
 * @brief   Retrieve the current frequency of the PWM.
 *
 * @param[in]  pwm        PWM driver to read.
 * @param[out] frequency  The currently set frequency.
 *
 * @return  The status indicates whether the function call was successful.
 */
apalExitStatus_t apalPWMGetFrequency(apalPWMDriver_t* pwm, apalPWMfrequency_t* const frequency);

/**
 * @brief   Retrieve the current period of the PWM.
 *
 * @param[in]   pwm     PWM driver to read.
 * @param[out]  period  The currently set period.
 *
 * @return  The status indicates whether the function call was successful.
 */
apalExitStatus_t apalPWMGetPeriod(apalPWMDriver_t* pwm, apalPWMperiod_t* const period);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* QEI                                                                        */
/*============================================================================*/

/**
 * @brief QEI driver type.
 */
typedef osQEIDriver_t apaQEIDriver_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Gets the direction of the last transition.
 *
 * @param[in]   qei         The QEI driver to use.
 * @param[out]  direction   The direction of the last transition.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalQEIGetDirection(apalQEIDriver_t* qei, apalQEIDirection_t* const direction);

/**
 * @brief Gets the current position of the ecnoder.
 *
 * @param[in]   qei       The QEI driver to use.
 * @param[out]  position  The current position of the encoder.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalQEIGetPosition(apalQEIDriver_t* qei, apalQEICount_t* const position);

/**
 * @brief Gets the value range of the encoder.
 *
 * @param[in]   qei     The QEI driver to use.
 * @param[out]  range   The value range of the encoder.
 *
 * @return The status indicates whether the function call was successful.
 */
apalExitStatus_t apalQEIGetRange(apalQEIDriver_t* qei, apalQEICount_t* const range);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* I2C                                                                        */
/*============================================================================*/

/**
 * @brief I2C driver type.
 */
typedef osI2CDriver_t apalI2CDriver_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Transmit data and receive a response.
 *
 * @param[in]   i2cd      The I2C driver to use.
 * @param[in]   addr      Address to write to.
 * @param[in]   txbuf     Buffer containing data to send.
 * @param[in]   txbytes   Number of bytes to send.
 * @param[out]  rxbuf     Buffer to store a response to.
 * @param[in]   rxbytes   Number of bytes to receive.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return The status indicates whether the function call was succesful or a timeout occurred.
 */
apalExitStatus_t apalI2CMasterTransmit(apalI2CDriver_t* i2cd, const apalI2Caddr_t addr, const uint8_t* const txbuf, const size_t txbytes, uint8_t* const rxbuf, const size_t rxbytes, const apalTime_t timeout);

/**
 * @brief Read data from a specific address.
 *
 * @param[in]   i2cd      The I2C driver to use.
 * @param[in]   addr      Address to read.
 * @param[out]  rxbuf     Buffer to store the response to.
 * @param[in]   rxbytes   Number of bytes to receive.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return The status indicates whether the function call was succesful or a timeout occurred.
 */
apalExitStatus_t apalI2CMasterReceive(apalI2CDriver_t* i2cd, const apalI2Caddr_t addr, uint8_t* const rxbuf, const size_t rxbytes, const apalTime_t timeout);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* SPI                                                                        */
/*============================================================================*/

/**
 * @brief SPI driver type.
 */
typedef osSPIDriver_t apalSPIDriver_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Transmit and receive data from SPI
 *
 * @param[in]   spid      The SPI driver to use.
 * @param[in]   txData    Buffer containing data to send.
 * @param[out]  rxData    Buffer to store.
 * @param[in]   length    Number of bytes to send.
 *
 * @return The status indicates whether the function call was succesful.
 */
apalExitStatus_t apalSPIExchange(apalSPIDriver_t* spid, const uint8_t* const txData , uint8_t* const rxData, const size_t length);

/**
 * @brief Receive data from SPI
 *
 * @param[in]   spid      The SPI driver to use.
 * @param[out]  data      Buffer to store.
 * @param[in]   length    Number of bytes to send.
 *
 * @return The status indicates whether the function call was succesful.
 */
apalExitStatus_t apalSPIReceive(apalSPIDriver_t* spid, uint8_t* const data, const size_t length);

/**
 * @brief Transmit data to SPI
 *
 * @param[in]   spid      The SPI driver to use.
 * @param[in]   data      Buffer containing data to send.
 * @param[in]   length    Number of bytes to send.
 *
 * @return The status indicates whether the function call was succesful.
 */
apalExitStatus_t apalSPITransmit(apalSPIDriver_t* spid, const uint8_t* const data, const size_t length);

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/* DEBUG                                                                      */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Assert function to check a given condition.
 * @note  Using a macro is preferable.
 *
 * @param[in] c   The condition to check.
 */
void apalDbgAssert(bool c);

#ifdef __cplusplus
}
#endif

#endif /* _AMIROOS_PERIPHAL_H_ */
