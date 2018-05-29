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

#include <alld_tps62113.h>

#if defined(AMIROLLD_CFG_USE_TPS62113) || defined(__DOXYGEN__)

/**
 * @brief Read the value of the POWER_EN Gpio pin.
 * @param[in]   tpsd        tps62113 drivers
 * @param[out]  pinstate    current value of the gpio pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
tps62113_lld_get_power_en(const TPS62113Driver* const tps62113, tps62113_lld_power_en_t* const power)
{
  apalDbgAssert(tps62113 != NULL);
  apalDbgAssert(power != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&tps62113->gpio_power, &gpio_state);
  *power = (gpio_state == APAL_GPIO_ON) ? TPS62113_LLD_POWER_ENABLED : TPS62113_LLD_POWER_DISABLED;
  return status;
}

/**
 * @brief Set the value of the POWER_EN Gpio pin.
 * @param[in]   tpsd        tps62113 drivers
 * @param[in]   pinstate    new value of the gpio pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
tps62113_lld_set_power_en(const TPS62113Driver* const tps62113, const tps62113_lld_power_en_t power)
{
  apalDbgAssert(tps62113 != NULL);

  return apalControlGpioSet(&tps62113->gpio_power, (power == TPS62113_LLD_POWER_ENABLED) ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

#endif /* defined(AMIROLLD_CFG_USE_TPS62113) */
