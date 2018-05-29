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

#include <alld_a3906.h>

#if defined(AMIROLLD_CFG_USE_A3906) || defined(__DOXYGEN__)

/**
 * @brief Sets the power state of the A3906.
 * @param[in]   a3906d      The A3906 driver.
 * @param[in]   power_state The new power state to set.
 *
 * @return The return status indicates whether the function call was successfull.
 */
inline apalExitStatus_t
a3906_lld_set_power(const A3906Driver* const a3906, const a3906_lld_power_t power_state)
{
  apalDbgAssert(a3906 != NULL);

  // set the output of the pin depending on the activation property
  return apalControlGpioSet(&a3906->power_pin, (power_state == A3906_LLD_POWER_ON) ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

/**
 * @brief Gets the current power state of the A3906.
 *
 * @param[in]   a3906d        The A3906 driver,
 * @param[out]  power_state   The currently set power state.
 *
 * @return The return status indicates whether the function call was successfull.
 */
inline apalExitStatus_t
a3906_lld_get_power(const A3906Driver* const a3906, a3906_lld_power_t* const power_state)
{
  apalDbgAssert(a3906 != NULL);
  apalDbgAssert(power_state != NULL);

  // get current activation state of power gpio
  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&a3906->power_pin, &gpio_state);
  *power_state = gpio_state == APAL_GPIO_ON ? A3906_LLD_POWER_ON : A3906_LLD_POWER_OFF;
  return status;
}

/**
 * @brief Set the PWM width for one channel.
 *
 * @param[in]   pwmd      The PWM driver to use.
 * @param[in]   channel   The channel of the given PWM driver to set.
 * @param[in]   width     The new width to set the PWM to.
 *
 * @return The return status indicates whether the function call was successfull.
 */
inline apalExitStatus_t
a3906_lld_set_pwm(apalPWMDriver_t* pwm, const apalPWMchannel_t channel, const apalPWMwidth_t width)
{
  apalDbgAssert(pwm != NULL);

  return apalPWMSet(pwm, channel, width);
}

#endif /* defined(AMIROLLD_CFG_USE_A3906) */
