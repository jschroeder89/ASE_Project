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

#include <alld_vcnl4020.h>

#if defined(AMIROLLD_CFG_USE_VCNL4020) || defined(__DOXYGEN__)

/*============================================================================*/
/* general single register access                                             */
/*============================================================================*/

/**
 * @brief Read the 8-bit data from any register.
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[in]   reg       The register to read.
 * @param[out]  data      The data read from the VCNL4020.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readreg(const VCNL4020Driver* const vcnld, const vcnl4020_lld_regaddr_t reg, uint8_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(data != NULL);

  return apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, (uint8_t*)&reg, 1, data, 1, timeout);
}

/**
 * @brief Write data to any 8-bit register.
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[in]   reg       The register to write to.
 * @param[in]   data      The data to write.
 * @param[in]   timeout   Timeout for the function to return (in microseconds)
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_writereg(const VCNL4020Driver* const vcnld, const vcnl4020_lld_regaddr_t reg, const uint8_t data, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);

  uint8_t txbuf[2] = { reg, data };
  return apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 2, NULL, 0, timeout);
}

/*============================================================================*/
/* sensor result access                                                       */
/*============================================================================*/

/**
 * @brief Read the result of the latest ambient light measurement.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  als       Result of the latest ambient light measuremenet.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readals(const VCNL4020Driver* const vcnld, uint16_t* const als, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(als != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_ALRES_HIGH };
  uint8_t rxbuf[2];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 2, timeout);
  *als = (rxbuf[0] << 8) | rxbuf[1];
  return status;
}

/**
 * @brief Read the result of the latest proximity measurement.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  prox      Result of the latest proximity measuremenet.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readprox(const VCNL4020Driver* const vcnld, uint16_t* const prox, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(prox != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_PROXRES_HIGH };
  uint8_t rxbuf[2];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 2, timeout);
  *prox = (rxbuf[0] << 8) | rxbuf[1];
  return status;
}

/**
 * @brief Read the results of the latest ambient light and proximity measurements.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  als       Result of the latest ambient light measuremenet.
 * @param[out]  prox      Result of the latest proximity measuremenet.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readalsandprox(const VCNL4020Driver* const vcnld, uint16_t* const als, uint16_t* const prox, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(als != NULL);
  apalDbgAssert(prox != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_ALRES_HIGH };
  uint8_t rxbuf[4];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 4, timeout);
  *als = (rxbuf[0] << 8) | rxbuf[1];
  *prox = (rxbuf[2] << 8) | rxbuf[3];
  return status;
}

/*============================================================================*/
/* threshold register access                                                  */
/*============================================================================*/

/**
 * @brief Read the current value of the low threshold.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  th        Current value of the low threshold.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readlth(const VCNL4020Driver* const vcnld, uint16_t* const th, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(th != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_LTH_HIGH };
  uint8_t rxbuf[2];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 2, timeout);
  *th = (rxbuf[0] << 8) | rxbuf[1];
  return status;
}

/**
 * @brief Read the current value of the high threshold.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  th        Current value of the high threshold.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readhth(const VCNL4020Driver* const vcnld, uint16_t* const th, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(th != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_HTH_HIGH };
  uint8_t rxbuf[2];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 2, timeout);
  *th = (rxbuf[0] << 8) | rxbuf[1];
  return status;
}

/**
 * @brief Read the current value of both (high and low) thresholds.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[out]  lth       Current value of the low threshold.
 * @param[out]  hth       Current value of the high threshold.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_readth(const VCNL4020Driver* const vcnld, uint16_t* const lth, uint16_t* const hth, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);
  apalDbgAssert(lth != NULL);
  apalDbgAssert(hth != NULL);

  uint8_t txbuf[1] = { VCNL4020_LLD_REGADDR_LTH_HIGH };
  uint8_t rxbuf[4];
  apalExitStatus_t status = apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 1, rxbuf, 4, timeout);
  *lth = (rxbuf[0] << 8) | rxbuf[1];
  *hth = (rxbuf[2] << 8) | rxbuf[3];
  return status;
}

/**
 * @brief Set the low threshold value.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[in]   th        The new threshold value to write.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_writelth(const VCNL4020Driver* const vcnld, const uint16_t th, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);

  uint8_t txbuf[3] = { VCNL4020_LLD_REGADDR_LTH_HIGH ,(th & 0xFF00u) >> 8, th & 0x00FFu };
  return apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 3, NULL, 0, timeout);
}

/**
 * @brief Set the high threshold value.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[in]   th        The new threshold value to write.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_writehth(const VCNL4020Driver* const vcnld, const uint16_t th, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);

  uint8_t txbuf[3] = { VCNL4020_LLD_REGADDR_HTH_HIGH, (th & 0xFF00u) >> 8, th & 0x00FFu };
  return apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 3, NULL, 0, timeout);
}

/**
 * @brief Set both (high and low) threshold values.
 *
 * @param[in]   vcnld     The VCNL402 driver to use.
 * @param[in]   lth       The new value of the low threshold.
 * @param[in]   hth       The new value of the high threshold.
 * @param[in]   timeout   Timeout for the function to return (in microseconds).
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
vcnl4020_lld_writeth(const VCNL4020Driver* const vcnld, const uint16_t lth, const uint16_t hth, const apalTime_t timeout)
{
  apalDbgAssert(vcnld != NULL && vcnld->i2cd != NULL);

  uint8_t txbuf[5] = { VCNL4020_LLD_REGADDR_LTH_HIGH, (lth & 0xFF00u) >> 8, lth & 0x00FFu, (hth & 0xFF00u) >> 8, hth & 0x00FFu };
  return apalI2CMasterTransmit(vcnld->i2cd, (apalI2Caddr_t)VCNL4020_LLD_I2C_ADDR, txbuf, 5, NULL, 0, timeout);
}

#endif /* defined(AMIROLLD_CFG_USE_VCNL4020) */
