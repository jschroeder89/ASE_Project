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

#include <alld_mpr121.h>

#if defined(AMIROLLD_CFG_USE_MPR121) || defined(__DOXYGEN__)

#include <string.h>

/**
 * @brief Read one or more of the internal registers of the mpr121.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   regaddr Address of the register
 * @param[in]   offset  Offset to be added to the register address
 * @param[in]   size    Size of the data to be read
 * @param[out]  data    The data that has been Read
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_read_register(const MPR121Driver* const mprd, const mpr121_lld_register_t regaddr, const uint8_t offset, const uint8_t size, uint8_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(mprd != NULL && mprd->i2cd != NULL);
  apalDbgAssert(data != NULL);

  uint8_t addr = regaddr + offset;
  return apalI2CMasterTransmit(mprd->i2cd, MPR121_LLD_I2C_ADDR_FIXED, &addr, 1, data, size, timeout);
}

/**
 * @brief Write to one or more of the internal registers of the mpr121.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   regaddr Address of the register
 * @param[in]   offset  Offset to be added to the register address
 * @param[in]   size    Size of the data to be read
 * @param[in]   data    The data that will be written
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_write_register(const MPR121Driver* const mprd, const mpr121_lld_register_t regaddr, const uint8_t offset, const uint8_t size, const uint8_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(mprd != NULL && mprd->i2cd != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[size+1];
  buffer[0] = regaddr + offset;
  memcpy(buffer+1, data, size);
  return apalI2CMasterTransmit(mprd->i2cd, MPR121_LLD_I2C_ADDR_FIXED, buffer, size+1, NULL, 0, timeout);
}

/**
 * @brief Perform a soft reset of the mpr121.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_soft_reset(const MPR121Driver* const mprd, const  apalTime_t timeout)
{
  apalDbgAssert(mprd != NULL && mprd->i2cd != NULL);

  uint8_t buffer[2] = {MPR121_LLD_REGISTER_SOFT_RESET, MPR121_LLD_I2C_SOFTRESET};
  return apalI2CMasterTransmit(mprd->i2cd, MPR121_LLD_I2C_ADDR_FIXED, buffer, 2, NULL, 0, timeout);
}

/**
 * @brief Read the filtered data of the mpr121 sensors.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   index   The index of the sensor from which reading should be started
 * @param[in]   num     Number of consecutive sensors to read
 * @param[in]   data    The data that has been read
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_read_filtered_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint16_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(data != NULL);

  uint8_t buffer[2*num];
  apalExitStatus_t status = mpr121_lld_read_register(mprd, MPR121_LLD_REGISTER_FILTERED_DATA, index*2, num*2, buffer, timeout);
  for (uint8_t dataIdx = 0; dataIdx < num; dataIdx++) {
    data[dataIdx] = (((uint16_t)buffer[2*dataIdx+1]) << 8) | buffer[dataIdx*2];
  }
  return status;
}

/**
 * @brief Read the baseline data of the mpr121 sensors.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   index   The index of the sensor from which reading should be started
 * @param[in]   num     Number of consecutive sensors to read
 * @param[in]   data    The data that has been read
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_read_baseline_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint8_t* const data, const apalTime_t timeout)
{
  return mpr121_lld_read_register(mprd, MPR121_LLD_REGISTER_BASELINE, index, num, data, timeout);
}

/**
 * @brief Read the electrode data of the mpr121 sensors.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   index   The index of the sensor from which reading should be started
 * @param[in]   num     Number of consecutive sensors to read
 * @param[in]   data    The data that has been read
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_read_electrode_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint8_t* const data, const apalTime_t timeout)
{
  return mpr121_lld_read_register(mprd, MPR121_LLD_REGISTER_ELE_CURRENT, index, num, data, timeout);
}

/**
 * @brief Write a configuration to the mpr121.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   cfg     The configuration to be written
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_write_config(const MPR121Driver* const mprd, const mpr121_lld_config_t cfg, const apalTime_t timeout)
{
  const apalExitStatus_t status = mpr121_lld_write_register(mprd, MPR121_LLD_REGISTER_AUTOCFG_CTRL_0, 0, 5, cfg.values, timeout);
  if (status != APAL_STATUS_OK) {
    return status;
  } else {
    return mpr121_lld_write_register(mprd, MPR121_LLD_REGISTER_CONFIG_1, 0, 3, &(cfg.values[5]), timeout);
  }
}

/**
 * @brief Read a configuration from the mpr121.
 * @param[in]   mprd    MPR121 Driver
 * @param[in]   cfg     The configuration to be written
 * @param[in]   timeout Timeout for the I2C Bus
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
mpr121_lld_read_config(const MPR121Driver* const mprd, mpr121_lld_config_t* const cfg, const apalTime_t timeout)
{
  const apalExitStatus_t status = mpr121_lld_read_register(mprd, MPR121_LLD_REGISTER_AUTOCFG_CTRL_0, 0, 5, cfg->values, timeout);
  if (status != APAL_STATUS_OK) {
    return status;
  } else {
    return mpr121_lld_read_register(mprd, MPR121_LLD_REGISTER_CONFIG_1, 0, 3, &(cfg->values[5]), timeout);
  }
}

#endif /* defined(AMIROLLD_CFG_USE_MPR121) */
