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

#include <alld_lis331dlh.h>

#if defined(AMIROLLD_CFG_USE_LIS331DLH) || defined(__DOXYGEN__)

#include <string.h>

/**
 * @brief Read the content of one or more registers.
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[in]   regaddr   The address of the first register.
 * @param[out]  data      The data read from the registers.
 * @param[in]   length    Number of registers to read.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_register(const LIS331DLHDriver* const lisd, const lis331dlh_lld_register_t regaddr, uint8_t *data, const uint8_t length)
{
  apalDbgAssert(lisd != NULL && lisd->spid != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[length+1];
  buffer[0] = regaddr | LIS331DLH_LLD_SPI_READ | ((length > 1) ? LIS331DLH_LLD_SPI_MULT : 0);
  apalExitStatus_t status = apalSPIExchange(lisd->spid, buffer, buffer, length+1);
  memcpy(data, buffer+1, length);
  return status;
}

/**
 * @brief Write to one or more registers.
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[in]   regaddr   The address of the first register.
 * @param[in]   data      The data to be written to the registers.
 * @param[in]   length    Number of registers to write.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_write_register(const LIS331DLHDriver* const lisd, const lis331dlh_lld_register_t regaddr, const uint8_t *data, const uint8_t length)
{
  apalDbgAssert(lisd != NULL && lisd->spid != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[length+1];
  buffer[0] = regaddr | LIS331DLH_LLD_SPI_WRITE | ((length > 1) ? LIS331DLH_LLD_SPI_MULT : 0);
  memcpy(buffer+1, data, length);
  return apalSPITransmit(lisd->spid, buffer, length+1);
}

/**
 * @brief Reset the high pass filter
 * @param[in]   lisd      The LIS331DLH driver to use.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_reset_hp_filter(const LIS331DLHDriver* const lisd)
{
  return lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_HP_FILTER_RESET, NULL, 0);
}

/**
 * @brief Read the sensor data of all 3 axes.
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[out]  data      The sensor data.
 * @param[in]   cfg       The current configuration. Needed to find out if the data is saved as little or big endian.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_all_data(const LIS331DLHDriver* const lisd, int16_t *data, const lis331dlh_lld_cfg_t *cfg)
{
  apalDbgAssert(data != NULL);
  apalDbgAssert(cfg != NULL);

  uint8_t buffer[6];
  apalExitStatus_t status = lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_OUT_X_L, buffer, 6);
  if (!(cfg->registers.ctrl_reg4 & LIS331DLH_LLD_BLE_BE)) {
    data[0] = (int16_t) (buffer[0] | (buffer[1] << 8));
    data[1] = (int16_t) (buffer[2] | (buffer[3] << 8));
    data[2] = (int16_t) (buffer[4] | (buffer[5] << 8));
  } else {
    data[0] = (int16_t) (buffer[1] | (buffer[0] << 8));
    data[1] = (int16_t) (buffer[3] | (buffer[2] << 8));
    data[2] = (int16_t) (buffer[5] | (buffer[4] << 8));
  }
  return status;
}

/**
 * @brief Read the sensor data of one axis.
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[out]  data      The sensor data.
 * @param[in]   axis      The axis for which the data should be read.
 * @param[in]   cfg       The current configuration. Needed to find out if the data is saved as little or big endian.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_data(const LIS331DLHDriver* const lisd, int16_t *data, const lis331dlh_lld_axis_t axis, const lis331dlh_lld_cfg_t *cfg)
{
  apalDbgAssert(data != NULL);
  apalDbgAssert(cfg != NULL);

  apalExitStatus_t status = APAL_STATUS_SUCCESS;
  uint8_t buffer[2];
  switch (axis) {
    case LIS331DLH_LLD_X_AXIS:
      status = lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_OUT_X_L, buffer, 2);
      break;
    case LIS331DLH_LLD_Y_AXIS:
      status = lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_OUT_Y_L, buffer, 2);
      break;
    case LIS331DLH_LLD_Z_AXIS:
      status = lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_OUT_Z_L, buffer, 2);
      break;
    default:
      return APAL_STATUS_INVALIDARGUMENTS;
  }
  // entweder jedes mal endian abfragen oder config mit übergeben
  if (!(cfg->registers.ctrl_reg4 & LIS331DLH_LLD_BLE_BE)) {
    *data = (int16_t) (buffer[0] | (buffer[1] << 8));
  } else {
    *data = (int16_t) (buffer[1] | (buffer[0] << 8));
  }
  return status;
}

/**
 * @brief Read the current configuration
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[out]  cfg       The current configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_config(const LIS331DLHDriver* const lisd, lis331dlh_lld_cfg_t *cfg)
{
  apalDbgAssert(cfg != NULL);

  return lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_CTRL_REG1, cfg->data, 5);
}

/**
 * @brief Write the a configuration
 * @param[in]   lisd      The LIS331DLH driver to use.
 * @param[in]   cfg       The new configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_write_config(const LIS331DLHDriver* const lisd, const lis331dlh_lld_cfg_t *cfg)
{
  apalDbgAssert(cfg != NULL);

  return lis331dlh_lld_write_register(lisd, LIS331DLH_LLD_REGISTER_CTRL_REG1, cfg->data, 5);
}

/**
 * @brief Read the interrupt configuration of one of the interrupts.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  cfg         The current interrupt configuration.
 * @param[in]   interrupt   The interrupt for which the configuration should be read.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_int_config(const LIS331DLHDriver* const lisd, lis331dlh_lld_int_cfg_t *cfg, const lis331dlh_lld_int_t interrupt)
{
  apalDbgAssert(cfg != NULL);

  uint8_t buffer[2];
  apalExitStatus_t status = APAL_STATUS_OK;
  if (interrupt == LIS331DLH_LLD_INT1) {
    status |= lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_INT1_CFG, &cfg->cfg_reg, 1);
    status |= lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_INT1_THS, buffer, 2);
  } else {
    status |= lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_INT2_CFG, &cfg->cfg_reg, 1);
    status |= lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_INT2_THS, buffer, 2);
  }
  if (status != APAL_STATUS_OK) {
    return status;
  }
  cfg->threshold = buffer[0];
  cfg->duration = buffer[1];
  return status;
}

/**
 * @brief Write the interrupt configuration of one of the interrupts.
 * @param[in]   spid        The SPI driver to use.
 * @param[in]   cfg         The current new configuration.
 * @param[in]   interrupt   The interrupt for which the configuration should be written.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_write_int_config(const LIS331DLHDriver* const lisd, const lis331dlh_lld_int_cfg_t *cfg, const lis331dlh_lld_int_t interrupt)
{
  apalDbgAssert(cfg != NULL);

  uint8_t buffer[2];
  buffer[0] = cfg->threshold;
  buffer[1] = cfg->duration;
  apalExitStatus_t status = APAL_STATUS_OK;
  if (interrupt == LIS331DLH_LLD_INT1) {
    status |= lis331dlh_lld_write_register(lisd, LIS331DLH_LLD_REGISTER_INT1_CFG, &(cfg->cfg_reg), 1);
    status |= lis331dlh_lld_write_register(lisd, LIS331DLH_LLD_REGISTER_INT1_THS, buffer, 2);
  } else {
    status |= lis331dlh_lld_write_register(lisd, LIS331DLH_LLD_REGISTER_INT2_CFG, &(cfg->cfg_reg), 1);
    status |= lis331dlh_lld_write_register(lisd, LIS331DLH_LLD_REGISTER_INT2_THS, buffer, 2);
  }
  return status;
}

/**
 * @brief Read the status register.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  status      The status register value.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
lis331dlh_lld_read_status_register(const LIS331DLHDriver* const lisd, uint8_t *status)
{
  apalDbgAssert(status != NULL);

  return lis331dlh_lld_read_register(lisd, LIS331DLH_LLD_REGISTER_STATUS_REG, status, 1);
}

#endif /* defined(AMIROLLD_CFG_USE_LIS331DLH) */
