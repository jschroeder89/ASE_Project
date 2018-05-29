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

#include <alld_hmc5883l.h>

#if defined(AMIROLLD_CFG_USE_HMC5883L) || defined(__DOXYGEN__)

#include <string.h>

/**
 * @brief Reads the registers starting from idetification register A.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[out]  rxbuffer    The data read from the identification registers.
 * @param[in]   num         Number of registers to read.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_check(const HMC5883LDriver* const hmcd, uint8_t* const rxbuffer, const uint8_t num, const apalTime_t timeout)
{
  apalDbgAssert(hmcd != NULL && hmcd->i2cd != NULL);
  apalDbgAssert(rxbuffer != NULL);

  uint8_t txbuffer = HMC5883L_LLD_REGISTER_IDENTIFICATION_A;
  return apalI2CMasterTransmit(hmcd->i2cd,HMC5883L_LLD_I2C_ADDR,&txbuffer,1,rxbuffer,num,timeout);
}

/**
 * @brief Write data to consecutive registers.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]   regaddr     Address of first register.
 * @param[in]   data        Bytes to write to the registers.
 * @param[in]   num         Number of registers to write.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_write_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, const uint8_t* const data, const uint8_t num, const apalTime_t timeout)
{
  apalDbgAssert(hmcd != NULL && hmcd->i2cd != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[num+1];
  buffer[0] = regaddr;
  memcpy(buffer+1, data, num);
  return apalI2CMasterTransmit(hmcd->i2cd,HMC5883L_LLD_I2C_ADDR,buffer,num+1,NULL,0,timeout);
}

/**
 * @brief Write data to one registers.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]   regaddr     Address of the register.
 * @param[in]   data        Data to write ti the register.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_set_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, const uint8_t data, const apalTime_t timeout)
{
  apalDbgAssert(hmcd != NULL && hmcd->i2cd != NULL);

  uint8_t buffer[2];
  buffer[0] = regaddr;
  buffer[1] = data;
  return apalI2CMasterTransmit(hmcd->i2cd,HMC5883L_LLD_I2C_ADDR,buffer,2,NULL,0,timeout);
}

/**
 * @brief Read data from consecutive registers.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]   regaddr     Address of first register.
 * @param[out]  data        Bytes to read from the registers.
 * @param[in]   num         Number of registers to read.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, uint8_t* const data, const uint8_t num, const apalTime_t timeout)
{
  apalDbgAssert(hmcd != NULL && hmcd->i2cd != NULL);
  apalDbgAssert(data != NULL);

  return apalI2CMasterTransmit(hmcd->i2cd,HMC5883L_LLD_I2C_ADDR,(uint8_t*)&regaddr,1,data,num,timeout);
}

/**
 * @brief Read sensor data.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[out]  data        Sensor data.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_data(const HMC5883LDriver* const hmcd, uint16_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(data != NULL);

  uint8_t buffer[6];
  apalExitStatus_t status = hmc5883l_lld_read_register(hmcd, HMC5883L_LLD_REGISTER_DATA_OUT_X_MSB, buffer, 6, timeout);
  data[0] = ((uint16_t)buffer[0] << 8) | buffer[1];
  data[1] = ((uint16_t)buffer[2] << 8) | buffer[3];
  data[2] = ((uint16_t)buffer[4] << 8) | buffer[5];
  for (uint8_t dataIdx = 0; dataIdx < 3; dataIdx++) {
    if (data[dataIdx] >> 15) {
      data[dataIdx] = -1 * ~(data[dataIdx] - 1);
    }
  }
  return status;
}

/**
 * @brief Read the status register.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[out]  status      Content of status register,
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_status(const HMC5883LDriver* const hmcd, uint8_t* const status, const apalTime_t timeout)
{
  apalDbgAssert(status != NULL);

  return hmc5883l_lld_read_register(hmcd, HMC5883L_LLD_REGISTER_STATUS, status, 1, timeout);
}

/**
 * @brief Read the configuration of the hmc5883l.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[out]  cfg         Structure representing the current configuration.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_config(const HMC5883LDriver* const hmcd, hmc5883l_lld_config_t* const cfg, const apalTime_t timeout)
{
  apalDbgAssert(cfg != NULL);

  uint8_t conf[3];
  apalExitStatus_t status = hmc5883l_lld_read_register(hmcd, HMC5883L_LLD_REGISTER_CONFIG_A, conf, 3, timeout);
  cfg->avg = conf[0] & (3 << 5);
  cfg->outrate = conf[0] & (7 << 2);
  cfg->mbias = conf[0] & 3;
  cfg->gain = conf[1] & (7 << 5);
  cfg->highspeed = conf[2] >> 7;
  cfg->mode = conf[2] & 3;
  return status;
}

/**
 * @brief Write a configuration to the hmc5883l.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]  cfg         Structure representing the new configuration.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_write_config(const HMC5883LDriver* const hmcd, const hmc5883l_lld_config_t cfg, const apalTime_t timeout)
{
  uint8_t conf[3];
  conf[0] = cfg.avg | cfg.outrate | cfg.mbias;
  conf[1] = cfg.gain;
  conf[2] = (cfg.highspeed << 7) | cfg.mode;
  return hmc5883l_lld_write_register(hmcd, HMC5883L_LLD_REGISTER_CONFIG_A, conf, 3, timeout);
}

/**
 * @brief Read the lock bit of the status register.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]   lock        Status of the lock bit.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_lock(const HMC5883LDriver* const hmcd, uint8_t* const lock, const apalTime_t timeout)
{
  apalDbgAssert(lock != NULL);

  apalExitStatus_t status = hmc5883l_lld_read_register(hmcd, HMC5883L_LLD_REGISTER_STATUS, lock, 1, timeout);
  *lock >>= 1;
  return status;
}

/**
 * @brief Read the rdy bit of the status register.
 * @param[in]   hmcd        The HMC5883L driver to use.
 * @param[in]   rdy         Status of the rdy bit.
 * @param[in]   timeout     Timeout for the function to return (in microseconds).
 *
 * @return The return status indicates whether the function call was successfull or a timeout occured.
 */
inline apalExitStatus_t
hmc5883l_lld_read_rdy(const HMC5883LDriver* const hmcd, uint8_t* const rdy, const apalTime_t timeout)
{
  apalDbgAssert(rdy != NULL);

  apalExitStatus_t status = hmc5883l_lld_read_register(hmcd, HMC5883L_LLD_REGISTER_STATUS, rdy, 1, timeout);
  *rdy &= 1;
  return status;
}

#endif /* defined(AMIROLLD_CFG_USE_HMC5883L) */
