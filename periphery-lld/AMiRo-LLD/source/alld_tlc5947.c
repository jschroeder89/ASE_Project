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

#include <alld_tlc5947.h>

#if defined(AMIROLLD_CFG_USE_TLC5947) || defined(__DOXYGEN__)

/**
 * @brief   Set the blank state of the TLC5947.
 * @param[in] tlc5947   The TLC5947 driver object.
 * @param[in] blank     The state to set the TLC5947 driver to.
 * @return              An indicator whether the call was successful.
 */
inline apalExitStatus_t
tlc5947_lld_setBlank(const TLC5947Driver* const tlc5947, const tlc5947_lld_blank_t blank)
{
  apalDbgAssert(tlc5947 != NULL);

  // set the output value of the GPIO pin depending on the activation property
  return apalControlGpioSet(&tlc5947->blank_gpio, blank == TLC5947_LLD_BLANK_ENABLE ? APAL_GPIO_ON : APAL_GPIO_OFF);
}

/**
 * @brief   Get the current status of the TLC5947s blank pin.
 * @param[in]  tlc5947  The TLC5947 driver object.
 * @param[out] blank    The state object to fill.
 * @return              An indicator whether the call was successful.
 */
inline apalExitStatus_t
tlc5947_lld_getBlank(const TLC5947Driver* const tlc5947, tlc5947_lld_blank_t* const blank)
{
  apalDbgAssert(tlc5947 != NULL);
  apalDbgAssert(blank != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&tlc5947->blank_gpio, &gpio_state);
  *blank = gpio_state == APAL_GPIO_ON ? TLC5947_LLD_BLANK_ENABLE : TLC5947_LLD_BLANK_DISABLE;
  return status;
}

/**
 * @brief   Writes a pulse on XLAT signal.
 * @details A pulse makes the TLC to move the grayscale register data to the internal latch and visualize it.
 * @param[in] tlc5947   The TLC5947 driver object.
 * @return              An indicator whether the call was successful.
 */
apalExitStatus_t tlc5947_lld_update(const TLC5947Driver* const tlc5947)
{
  apalDbgAssert(tlc5947 != NULL);

  apalExitStatus_t status = apalControlGpioSet(&tlc5947->xlat_gpio, APAL_GPIO_ON);
  // The XLAT signal has to be active for at least 30 ns.
  // It is assumed that that these function calls satisfy this requirement even without explicit delay.
  if (apalControlGpioSet(&tlc5947->xlat_gpio, APAL_GPIO_OFF) == APAL_STATUS_OK && status == APAL_STATUS_OK) {
    return APAL_STATUS_OK;
  } else {
    return APAL_STATUS_ERROR;
  }
}

/**
 * @brief Write buffer via SPI to the TLC5947.
 * @return                  An indicator whether the call was successful.
 */
inline apalExitStatus_t
tlc5947_lld_write(const TLC5947Driver* const tlc5947, const tlc5947_lld_buffer_t* const buffer)
{
  apalDbgAssert(tlc5947 != NULL);
  apalDbgAssert(buffer != NULL);

  // send buffer via SPI
  return apalSPITransmit(tlc5947->spi_driver, buffer->data, TLC5947_LLD_BUFFER_SIZE);
}

/**
 * @brief   Set a specific channel of a given buffer.
 * @param[in] buffer    The buffer to modify.
 * @param[in] channel   The channel to set.
 * @param[in] value     The new value to set.
 *                      Must be a 12bit value.
 * @return              An indicator whether the call was successful.
 */
inline void
tlc5947_lld_setBuffer(tlc5947_lld_buffer_t* const buffer, const uint8_t channel, const uint16_t value)
{
  apalDbgAssert(buffer != NULL);
  apalDbgAssert(channel < TLC5947_LLD_NUM_CHANNELS);
  apalDbgAssert(value <= (1 << TLC5947_LLD_PWM_RESOLUTION_BITS) - 1);

  // reverse the channel number, since the data is shifted within the TLC5947 and thus the order is inverted
  const uint8_t idx = (uint16_t)(TLC5947_LLD_NUM_CHANNELS-1 - channel) * TLC5947_LLD_PWM_RESOLUTION_BITS / 8;
  // channel is odd / inverse channel is even
  if (channel % 2) {
    buffer->data[idx] = (value >> 4) & 0xFFu;
    buffer->data[idx+1] = ((value << 4) & 0xF0u) | (buffer->data[idx+1] & 0x0Fu);
  }
  // channel is even / inverse channel is odd
  else {
    buffer->data[idx] = (buffer->data[idx] & 0xF0u) | ((value >> 8) & 0x0Fu);
    buffer->data[idx+1] = value & 0xFFu;
  }
  return;
}

/**
 * @brief   Read a specific state from a given buffer.
 * @param[in] buffer    The buffer to read from.
 * @param[in] channel   The channel to read
 * @return              An indicator whether the call was successful.
 */
inline uint16_t
tlc5947_lld_getBuffer(const tlc5947_lld_buffer_t* const buffer, const uint8_t channel)
{
  apalDbgAssert(buffer != NULL);
  apalDbgAssert(channel < TLC5947_LLD_NUM_CHANNELS);

  // reverse the channel number, since the data is shifted within the TLC5947 and thus the order is inverted
  const uint8_t idx = (uint16_t)(TLC5947_LLD_NUM_CHANNELS-1- channel) * TLC5947_LLD_PWM_RESOLUTION_BITS / 8;
  // channel is odd / inverse channel is even
  if (channel % 2) {
    return (((uint16_t)(buffer->data[idx])) << 4) | (uint16_t)((buffer->data[idx+1] & 0xF0u) >> 4);
  }
  // channel is even / inverse channel is odd
  else {
    return (((uint16_t)(buffer->data[idx] & 0x0Fu)) << 8) | (uint16_t)(buffer->data[idx+1]);
  }
}

#endif /* defined(AMIROLLD_CFG_USE_TLC5947) */
