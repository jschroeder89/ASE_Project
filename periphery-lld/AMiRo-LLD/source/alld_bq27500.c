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

#include <alld_bq27500.h>

#if defined(AMIROLLD_CFG_USE_BQ27500) || defined(__DOXYGEN__)

#include <string.h>

/**
 * @brief Read the battery low Gpio pin.
 * @param[in]   bqd       The bq27500 driver
 * @param[out]  batlow    The battery low state
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_read_batlow(const BQ27500Driver* const bq27500, bq27500_lld_batlow_t* const batlow)
{
  apalDbgAssert(bq27500 != NULL);
  apalDbgAssert(batlow != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&bq27500->gpio_batlow, &gpio_state);
  *batlow = (gpio_state == APAL_GPIO_ON) ? BQ27500_LLD_BATTERY_LOW : BQ27500_LLD_BATTERY_NOT_LOW;
  return status;
}

/**
 * @brief Read the battery good Gpio pin.
 * @param[in]   bqd       The bq27500 driver
 * @param[out]  batgood   The battery good state
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_read_batgood(const BQ27500Driver* const bq27500, bq27500_lld_batgood_t* const batgood)
{
  apalDbgAssert(bq27500 != NULL);
  apalDbgAssert(batgood != NULL);

  apalControlGpioState_t gpio_state;
  apalExitStatus_t status = apalControlGpioGet(&bq27500->gpio_batgood, &gpio_state);
  *batgood = (gpio_state == APAL_GPIO_ON) ? BQ27500_LLD_BATTERY_GOOD : BQ27500_LLD_BATTERY_NOT_GOOD;
  return status;
}

/**
 * @brief Execute one of the standard commands.
 * @param[in]   i2cd      The I2C Driver
 * @param[in]   cmd       The command to be executed
 * @param[out]  dst       The return value of the command
 * @param[in]   timeout   Timeout of the I2C bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_std_command(const BQ27500Driver* const bq27500, const bq27500_lld_std_command_t cmd, uint16_t* const dst, const apalTime_t timeout)
{
  apalDbgAssert(bq27500 != NULL);
  apalDbgAssert(dst != NULL);

  uint8_t buffer[2];
  apalExitStatus_t status = apalI2CMasterTransmit(bq27500->i2cd, BQ27500_LLD_I2C_ADDR, (uint8_t*)&cmd, 1, buffer, 2, timeout);
  *dst = (uint16_t)((buffer[1] << 8) | buffer[0]);
  return status;
}

/**
 * @brief Execute one of the control sub commands.
 * @param[in]   i2cd      The I2C Driver
 * @param[in]   cmd       The sub command to be executed
 * @param[in]   timeout   Timeout of the I2C bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_sub_command_call(const BQ27500Driver* const bq27500, const bq27500_lld_control_subcmd_t cmd, const apalTime_t timeout)
{
  apalDbgAssert(bq27500 != NULL);

  uint8_t buffer[3] = {BQ27500_LLD_STD_CMD_Control, (uint8_t)(cmd & 0x00FFu), (uint8_t)((cmd & 0xFF00u) >> 8)};
  return apalI2CMasterTransmit(bq27500->i2cd, BQ27500_LLD_I2C_ADDR, buffer, 3, NULL, 0, timeout);
}

/**
 * @brief Read the return value of a previously executed control sub command.
 * @param[in]   i2cd      The I2C Driver
 * @param[out]  data      The data read from the sub command
 * @param[in]   timeout   Timeout of the I2C bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_sub_command_read(const BQ27500Driver* const bq27500, uint16_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(bq27500 != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[3] = {0, 0, 0};
  apalExitStatus_t status = apalI2CMasterTransmit(bq27500->i2cd, BQ27500_LLD_I2C_ADDR, buffer, 1, &buffer[1], 2, timeout);
  *data = (uint16_t)((buffer[2] << 8) | buffer[1]);
  return status;
}

/**
 * @brief Execute on of the extended commands.
 * @param[in]   i2cd      The I2C Driver
 * @param[in]   cmd       The ext command to be executed
 * @param[in]   rw        Use the command in read or write mode
 * @param[in]   buffer    A buffer for input and output
 * @param[in]   length    length of the buffer
 * @param[in]   offset    Offset for reading or writing
 * @param[in]   timeout   Timeout of the I2C bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_ext_command(const BQ27500Driver* const bq27500, const bq27500_lld_ext_command_t cmd, const bq27500_lld_ext_cmd_access_t rw, uint8_t* const buffer, const uint8_t length, const uint8_t offset, const apalTime_t timeout)
{
  apalDbgAssert(bq27500 != NULL);
  apalDbgAssert(buffer != NULL);

  uint8_t in_buffer[1+length];
  in_buffer[0] = cmd + offset;
  if (rw == BQ27500_LLD_EXT_CMD_WRITE) {
    memcpy(in_buffer+1, buffer, length);
  }
  uint8_t txbytes = 1 + ((rw == BQ27500_LLD_EXT_CMD_WRITE) ? length : 0);
  uint8_t rxbytes = ((rw == BQ27500_LLD_EXT_CMD_READ) ? length : 0);
  return apalI2CMasterTransmit(bq27500->i2cd, BQ27500_LLD_I2C_ADDR, in_buffer, txbytes, ((rw == BQ27500_LLD_EXT_CMD_READ) ? buffer : NULL), rxbytes, timeout);
}

/**
 * @brief Send data via the CTNL command.
 * @param[in]   i2cd      The I2C Driver
 * @param[in]   data      Data to be sent
 * @param[in]   timeout   Timeout of the I2C bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_send_ctnl_data(const BQ27500Driver* const bq27500, const uint16_t data, const apalTime_t timeout)
{
  apalDbgAssert(bq27500 != NULL);

  uint8_t buffer[3] = {BQ27500_LLD_STD_CMD_Control, (uint8_t)(data & 0x00FFu), (uint8_t)((data & 0xFF00u) >> 8)};
  return apalI2CMasterTransmit(bq27500->i2cd, BQ27500_LLD_I2C_ADDR, buffer, 3, NULL, 0, timeout);
}

/**
 * @bried Computes the checksum of blockdata.
 * @param[in]   blockdata   Data to compute the checksum of (32 byte buffer)
 * @param[out]  sum         The computed checksum
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
bq27500_lld_compute_blockdata_checksum(const uint8_t* const blockdata, uint8_t* const sum)
{
  apalDbgAssert(blockdata != NULL);
  apalDbgAssert(sum != NULL);

  *sum = 0;
  for (uint8_t dataIdx = 0; dataIdx < 32; dataIdx++) {
    *sum += blockdata[dataIdx];
  }
  *sum = 0xFF - *sum;
  return APAL_STATUS_SUCCESS;
}

#endif /* defined(AMIROLLD_CFG_USE_BQ27500) */
