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

#ifndef _AMIROLLD_BQ24103A_H_
#define _AMIROLLD_BQ24103A_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_BQ24103A) || defined(__DOXYGEN__)

/**
 * @brief Active state of enabled gpio.
 */
#define BQ24103A_LLD_ENABLED_GPIO_ACTIVE_STATE        APAL_GPIO_ACTIVE_LOW

/**
 * @brief Active state of charge status gpio.
 */
#define BQ24103A_LLD_CHARGE_STATUS_GPIO_ACTIVE_STATE  APAL_GPIO_ACTIVE_HIGH

/**
 * @brief The BQ24103A driver struct.
 */
typedef struct {
  apalControlGpio_t gpio_enabled;       /**< @brief GPIO to enable/disable the bq24103a */
  apalControlGpio_t gpio_charge_status; /**< @brief GPIO showing the status of the bq24103a */
} BQ24103ADriver;

/**
 * @brief Enable enum.
 */
typedef enum {
  BQ24103A_LLD_ENABLED  = 0x01,         /**< @brief 'charging on' state */
  BQ24103A_LLD_DISABLED = 0x00,         /**< @brief 'charging off' state */
} bq24103a_lld_enable_t;

/**
 * @brief Charge state enum.
 */
typedef enum {
  BQ24103A_LLD_CHARGING     = 0x01,     /**< @brief status indicating that the battery is being charged */
  BQ24103A_LLD_NOT_CHARGING = 0x00,     /**< @brife status indicating that the battery is not being charged */
} bq24103a_lld_charge_state_t;


#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t bq24103a_lld_get_enabled(const BQ24103ADriver* const bq24103a, bq24103a_lld_enable_t* const enable);
  apalExitStatus_t bq24103a_lld_set_enabled(const BQ24103ADriver* const bq24103a, const bq24103a_lld_enable_t enable);
  apalExitStatus_t bq24103a_lld_get_charge_status(const BQ24103ADriver* const bq24103a, bq24103a_lld_charge_state_t* const charge);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_BQ24103A) */

#endif /* _AMIROLLD_BQ24103A_H_ */
