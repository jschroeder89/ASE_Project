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

#include <alld_tps2051bdbv.h>

#if defined(AMIROLLD_CFG_USE_TPS2051BDBV) || defined(__DOXYGEN__)

/**
 * @brief Set the laser_en GPIO pin.
 * @param[in]   tpsd        tps2051 driver
 * @param[in]   enable      new state of the pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
tps2051b_lld_set_enable(const TPS2051BDriver* const tps2051b, const tps2051b_lld_enable_t enable)
{
  apalDbgAssert(tps2051b != NULL);

  return apalControlGpioSet(&tps2051b->enable, enable == TPS2051B_LLD_ENABLE ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

/**
 * @brief Read the laser_en GPIO pin.
 * @param[in]   tpsd        tps2051 driver
 * @param[out]  enable      state of the enable pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
tps2051b_lld_read_enable(const TPS2051BDriver* const tps2051b, tps2051b_lld_enable_t* const enable)
{
  apalDbgAssert(tps2051b != NULL);
  apalDbgAssert(enable != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&tps2051b->enable, &gpio_state);
  *enable = gpio_state == APAL_GPIO_ON ? TPS2051B_LLD_ENABLE : TPS2051B_LLD_DISABLE;
  return status;
}

/**
 * @brief Read the laser_oc GPIO pin.
 * @param[in]   tpsd        tps2051 driver
 * @param[out]  oc          state of the overcurrent pin
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
tps2051b_lld_read_overcurrent(const TPS2051BDriver* const tps2051b, tps2051b_lld_overcurrent_t* const oc)
{
  apalDbgAssert(tps2051b != NULL);
  apalDbgAssert(oc != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&tps2051b->overcurrent, &gpio_state);
  *oc = gpio_state == APAL_GPIO_ON ? TPS2051B_LLD_OVERCURRENT : TPS2051B_LLD_NO_OVERCURRENT;
  return status;
}

#endif /* defined(AMIROLLD_CFG_USE_TPS2051BDBV) */
