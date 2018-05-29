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

#include <alld_pca9544a.h>

#if defined(AMIROLLD_CFG_USE_PCA9544A) || defined(__DOXYGEN__)

/**
 * @brief Read the control register of the PCA9544A.
 *
 * @param[in]   pca9544a  The PCA9544A driver object.
 * @param[out]  data      The data read from the PCA9544A.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
pca9544a_lld_read(const PCA9544ADriver* const pca9544a, uint8_t* const  data, const apalTime_t timeout)
{
  apalDbgAssert(pca9544a != NULL);
  apalDbgAssert(data != NULL);

  return apalI2CMasterReceive(pca9544a->i2cd, (PCA9544A_LLD_I2C_ADDR_FIXED | pca9544a->addr), data, 1, timeout);
}

/**
 * @brief Set the control register of the PCA9544A.
 *
 * @param[in] pca9544a  The PCA9544A driver object.
 * @param[in] data      The value to write to the PCA9544A.
 * @param[in] timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
pca9544a_lld_write(const PCA9544ADriver* const pca9544a, const uint8_t data, const apalTime_t timeout)
{
  apalDbgAssert(pca9544a != NULL);

  return apalI2CMasterTransmit(pca9544a->i2cd, (PCA9544A_LLD_I2C_ADDR_FIXED | pca9544a->addr), &data, 1, NULL, 0, timeout);
}

/**
 * @brief Read the interrupt status.
 *
 * @param[in]   pca9544a  The PCA9544A driver object.
 * @param[out]  status    The status of the four interrupts.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
pca9544a_lld_getintstatus(const PCA9544ADriver* const pca9544a, pca9544a_lld_intstatus_t* const status, const apalTime_t timeout)
{
  apalDbgAssert(pca9544a != NULL);
  apalDbgAssert(status != NULL);

  uint8_t buffer;
  apalExitStatus_t stat = apalI2CMasterReceive(pca9544a->i2cd, (PCA9544A_LLD_I2C_ADDR_FIXED | pca9544a->addr), &buffer, 1, timeout);
  *status = buffer >> 4;
  return stat;
}

/**
 * @brief Read which channel is currently set.
 *
 * @param[in]   pca9544a  The PCA9544A driver object.
 * @param[out]  channel   The identifier of the set channel.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
pca9544a_lld_getcurrentchannel(const PCA9544ADriver* const pca9544a, pca9544a_lld_chid_t* const channel, const apalTime_t timeout)
{
  apalDbgAssert(pca9544a != NULL);
  apalDbgAssert(channel != NULL);

  uint8_t buffer;
  apalExitStatus_t stat = apalI2CMasterReceive(pca9544a->i2cd, (PCA9544A_LLD_I2C_ADDR_FIXED | pca9544a->addr), &buffer, 1, timeout);
  if (buffer & PCA9544A_LLD_CTRLREG_EN) {
    *channel = buffer & PCA9544A_LLD_CTRLREG_CH_MASK;
  } else {
    *channel = PCA9544A_LLD_CH_NONE;
  }
  return stat;
}

/**
 * @brief Set the channel for multiplexing.
 *
 * @param[in]   pca9544a  The PCA9544A driver object.
 * @param[in]   channel   The channel to set for multiplexing.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
pca9544a_lld_setchannel(const PCA9544ADriver* const pca9544a, const pca9544a_lld_chid_t channel, const apalTime_t timeout)
{
  apalDbgAssert(pca9544a != NULL);

  uint8_t buffer = (channel == PCA9544A_LLD_CH_NONE) ? 0x00u : (PCA9544A_LLD_CTRLREG_EN | channel);
  return apalI2CMasterTransmit(pca9544a->i2cd, (PCA9544A_LLD_I2C_ADDR_FIXED | pca9544a->addr), &buffer, 1, NULL, 0, timeout);
}

#endif /* defined(AMIROLLD_CFG_USE_PCA9544A) */
