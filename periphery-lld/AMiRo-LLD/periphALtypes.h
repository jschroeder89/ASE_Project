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

#ifndef _AMIROLLD_PERIPHALTYPES_H_
#define _AMIROLLD_PERIPHALTYPES_H_

/*============================================================================*/
/* DEPENDENCIES                                                               */
/*============================================================================*/

#include <stdint.h>

/*============================================================================*/
/* GENERAL                                                                    */
/*============================================================================*/

/**
 * @brief Time measurement type (in microseconds).
 */
typedef uint32_t    apalTime_t;

/**
 * @brief Status values used as return value for all (or most) function calls.
 * @note  The status can be used as mask of flags.
 */
typedef enum {
  APAL_STATUS_OK                = 0x00u, /**< success, no error occurred                          */
  APAL_STATUS_SUCCESS           = 0x00u, /**< success, no error occurred                          */
  APAL_STATUS_ERROR             = 0x01u, /**< failed, some unspecified error occured              */
  APAL_STATUS_FAILURE           = 0x01u, /**< failed, some unspecified error occured              */
  APAL_STATUS_TIMEOUT           = 0x02u, /**< failed, timeout occurred                            */
  APAL_STATUS_INVALIDARGUMENTS  = 0x04u, /**< failed, invalid arguments                           */
  APAL_STATUS_UNAVAILABLE       = 0x08u, /**< failed, function unavailable                        */
  APAL_STATUS_WARNING           = 0x10u, /**< success, but the result is probably not as expected */
} apalExitStatus_t;

/*============================================================================*/
/* GPIO                                                                       */
/*============================================================================*/

/**
 * @brief   Forward declaration.
 * @details Struct must be defined in 'periphAL.h'.
 */
typedef struct apalGpio_t apalGpio_t;

/**
 * @brief Status values to read/write a GPIO port.
 */
typedef enum {
  APAL_GPIO_LOW   = 0,  /**< logical low state  */
  APAL_GPIO_HIGH  = 1,  /**< logical high state */
} apalGpioState_t;

/**
 * @brief Status values to turn a control GPIO 'on' and 'off'.
 */
typedef enum {
  APAL_GPIO_OFF = 0,  /**< logical 'turned off' state */
  APAL_GPIO_ON  = 1,  /**< logical 'turned on' state  */
} apalControlGpioState_t;

/**
 * @brief   Polarity state of the control GPIO.
 */
typedef enum {
  APAL_GPIO_ACTIVE_LOW  = 0x00,   /**< A logically low state is defined as 'on'.  */
  APAL_GPIO_ACTIVE_HIGH = 0x01,   /**< A locically high state is defined as 'on'. */
} apalGpioActive_t;

/**
 * @brief   Informative signal edge for input control GPIOs.
 */
typedef enum {
  APAL_GPIO_EDGE_NONE     = 0x00,   /**< No edge indicates an interrupt.                      */
  APAL_GPIO_EDGE_RISING   = 0x01,   /**< Rising edges indicate an interrupt.                  */
  APAL_GPIO_EDGE_FALLING  = 0x02,   /**< Falling edges indicate an interrupt.                 */
  APAL_GPIO_EDGE_BOTH     = 0x03,   /**< Both rising and falling edges indicate an interrupt. */
} apalGpioEdge_t;

/**
 * @brief   Signal direction for the control GPIO.
 */
typedef enum {
  APAL_GPIO_DIRECTION_UNDEFINED     = 0x00,   /**< Signal direction is undefined.     */
  APAL_GPIO_DIRECTION_INPUT         = 0x01,   /**< Signal direction is input only.    */
  APAL_GPIO_DIRECTION_OUTPUT        = 0x02,   /**< Signal direction is output only    */
  APAL_GPIO_DIRECTION_BIDIRECTIONAL = 0x03,   /**< Signal direction is bidirectional. */
} apalGpioDirection_t;

/**
 * @brief Control GPIO meta information
 */
typedef struct {
  apalGpioActive_t active       : 1;  /**< Active state of the GPIO */
  apalGpioEdge_t edge           : 2;  /**< Edge configuration for according signals */
  apalGpioDirection_t direction : 2;  /**< Direction configuration for according signals */
} apalGpioMeta_t;

/**
 * @brief Control GPIO type.
 */
typedef struct {
  apalGpio_t* gpio;     /**< The GPIO to use.                 */
  apalGpioMeta_t meta;  /**< Meta information about the GPIO. */
} apalControlGpio_t;

/*============================================================================*/
/* PWM                                                                        */
/*============================================================================*/

/**
 * @brief PWM channel type.
 */
typedef uint8_t   apalPWMchannel_t;

/**
 * @brief PWM width type.
 */
typedef uint16_t  apalPWMwidth_t;

/**
 * @brief PWM frequency type.
 */
typedef uint32_t  apalPWMfrequency_t;

/**
 * @brief PWM period time.
 */
typedef uint32_t  apalPWMperiod_t;

/**
 * @brief PWM width limits and special values.
 */
typedef enum {
  APAL_PWM_WIDTH_OFF  = 0x0000u,  /**< PWM off            */
  APAL_PWM_WIDTH_MIN  = 0x0000u,  /**< PWM minimum width  */
  APAL_PWM_WIDTH_MAX  = 0xFFFFu,  /**< PWM maximum width  */
} apalPWMWidthLimit_t;

/*============================================================================*/
/* QEI                                                                        */
/*============================================================================*/

/**
 * @brief QEI counter type.
 */
typedef uint32_t  apalQEICount_t;

/**
 * @brief Direction of the QEI.
 */
typedef enum {
  APAL_QEI_DIRECTION_UP   = 0,  /**< QEI counted up   */
  APAL_QEI_DIRECTION_DOWN = 1,  /**< QEI counted down */
} apalQEIDirection_t;

/*============================================================================*/
/* I2C                                                                        */
/*============================================================================*/

/**
 * @brief I2C address type.
 */
typedef uint16_t apalI2Caddr_t;

/*============================================================================*/
/* SPI                                                                        */
/*============================================================================*/


#endif /* _AMIROLLD_PERIPHALTYPES_H_ */
