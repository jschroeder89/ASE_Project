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

#include <alld_led.h>

#if defined(AMIROLLD_CFG_USE_LED) || defined(__DOXYGEN__)

/**
 * @brief Set the LED to a specific state.
 * @param[in]   ledp        The LED driver object.
 * @param[in]   led_state   The state to set the LED to.
 * @return                  An indicator whether the call was successful.
 */
inline apalExitStatus_t
led_lld_set(const LEDDriver* const led, const led_lld_state_t state)
{
  apalDbgAssert(led != NULL);

  return apalControlGpioSet(&(led->gpio), (state == LED_LLD_STATE_ON) ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

/**
 * @brief Get the current status of the LED.
 * @param[in]   ledp        The LED driver object.
 * @param[out]  led_state   The state object to fill.
 * @return                  An indicator whether the call was successful.
 */
inline apalExitStatus_t
led_lld_get(const LEDDriver* const led, led_lld_state_t* const state)
{
  apalDbgAssert(led != NULL);
  apalDbgAssert(state != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&led->gpio, &gpio_state);
  *state = gpio_state == APAL_GPIO_ON ? LED_LLD_STATE_ON : LED_LLD_STATE_OFF;
  return status;
}

/**
 * @brief Toggle the LED.
 * @param[in]   ledp        The LED driver object.
 * @return                  An indicator whether the call was successful.
 */
inline apalExitStatus_t
led_lld_toggle(const LEDDriver* const led)
{
  apalDbgAssert(led != NULL);

  return apalGpioToggle(led->gpio.gpio);
}

#endif /* defined(AMIROLLD_CFG_USE_LED) */
