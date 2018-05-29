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

#include <alld_bq24103a.h>

#if defined(AMIROLLD_CFG_USE_BQ24103A) || defined(__DOXYGEN__)

/**
 * @brief Read the value of the POWER_EN Gpio pin.
 * @param[in]   tpsd        bq24103a drivers
 * @param[out]  pinstate    current value of the gpio pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
bq24103a_lld_get_enabled(const BQ24103ADriver* const bq24103a, bq24103a_lld_enable_t* const enable)
{
  apalDbgAssert(bq24103a != NULL);
  apalDbgAssert(enable != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&bq24103a->gpio_enabled, &gpio_state);
  *enable = gpio_state == APAL_GPIO_ON ? BQ24103A_LLD_ENABLED : BQ24103A_LLD_DISABLED;
  return status;
}

/**
 * @brief Set the value of the POWER_EN Gpio pin.
 * @param[in]   tpsd        bq24103a drivers
 * @param[in]   pinstate    new value of the gpio pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
bq24103a_lld_set_enabled(const BQ24103ADriver* const bq24103a, const bq24103a_lld_enable_t enable)
{
  apalDbgAssert(bq24103a != NULL);

  return apalControlGpioSet(&bq24103a->gpio_enabled, enable == BQ24103A_LLD_ENABLED ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

/**
 * @brief Read the value of the STAT1 charge status Gpio pin.
 * @param[in]   tpsd        bq24103a drivers
 * @param[out]  pinstate    current value of the gpio pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
bq24103a_lld_get_charge_status(const BQ24103ADriver* const bq24103a, bq24103a_lld_charge_state_t* const charge)
{
  apalDbgAssert(bq24103a != NULL);
  apalDbgAssert(charge != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&bq24103a->gpio_charge_status, &gpio_state);
  *charge = gpio_state == APAL_GPIO_ON ? BQ24103A_LLD_CHARGING : BQ24103A_LLD_NOT_CHARGING;
  return status;
}

#endif /* defined(AMIROLLD_CFG_USE_BQ24103A) */
