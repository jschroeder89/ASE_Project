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

#include<alld_l3g4200d.h>

#if defined(AMIROLLD_CFG_USE_L3G4200D) || defined(__DOXYGEN__)

#include <string.h>

/**
 * @brief Read the content of one or more registers.
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[in]   regaddr   The address of the first register.
 * @param[out]  data      The data read from the registers.
 * @param[in]   length    Number of registers to read.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_register(const L3G4200DDriver* const l3gd, const l3g4200d_lld_register_t regaddr, uint8_t* const data, const uint8_t length)
{
  apalDbgAssert(l3gd != NULL && l3gd->spid != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[length+1];
  buffer[0] = regaddr | L3G4200D_LLD_SPI_READ | ((length > 1) ? L3G4200D_LLD_SPI_MULT : 0);
  apalExitStatus_t status = apalSPIExchange(l3gd->spid, buffer, buffer, length+1);
  memcpy(data, buffer+1, length);
  return status;
}

/**
 * @brief Write to one or more registers.
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[in]   regaddr   The address of the first register.
 * @param[in]   data      The data to be written to the registers.
 * @param[in]   length    Number of registers to write.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_write_register(const L3G4200DDriver* const l3gd, const l3g4200d_lld_register_t regaddr, const uint8_t* const data, const uint8_t length)
{
  apalDbgAssert(l3gd != NULL && l3gd->spid != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[length+1];
  buffer[0] = regaddr | L3G4200D_LLD_SPI_WRITE | ((length > 1) ? L3G4200D_LLD_SPI_MULT : 0);
  memcpy(buffer+1, data, length);
  return apalSPITransmit(l3gd->spid, buffer, length+1);
}

/**
 * @brief Read the sensor data of all 3 axes.
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[out]  data      The sensor data.
 * @param[in]   cfg       The current configuration. Needed to find out if the data is saved as little or big endian.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_all_data(const L3G4200DDriver* const l3gd, int16_t* const data, const l3g4200d_lld_cfg_t* const cfg)
{
  apalDbgAssert(data != NULL);
  apalDbgAssert(cfg != NULL);

  uint8_t buffer[6];
  apalExitStatus_t status = l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_OUT_X_L, buffer, 6);
  if (!(cfg->registers.ctrl_reg4 & L3G4200D_LLD_BLE_MSB)) {
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
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[out]  data      The sensor data.
 * @param[in]   axis      The axis for which the data should be read.
 * @param[in]   cfg       The current configuration. Needed to find out if the data is saved as little or big endian.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_data(const L3G4200DDriver* const l3gd, int16_t* const data, const l3g4200d_lld_axis_t axis, const l3g4200d_lld_cfg_t* const cfg)
{
  apalDbgAssert(data != NULL);
  apalDbgAssert(cfg != NULL);

  apalExitStatus_t status = APAL_STATUS_SUCCESS;
  uint8_t buffer[2];
  switch (axis) {
    case L3G4200D_LLD_X_AXIS:
      status = l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_OUT_X_L, buffer, 2);
      break;
    case L3G4200D_LLD_Y_AXIS:
      status = l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_OUT_Y_L, buffer, 2);
      break;
    case L3G4200D_LLD_Z_AXIS:
      status = l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_OUT_Z_L, buffer, 2);
      break;
    default:
      return APAL_STATUS_INVALIDARGUMENTS;
  }
  // entweder jedes mal endian abfragen oder config mit übergeben
  if (!(cfg->registers.ctrl_reg4 & L3G4200D_LLD_BLE_MSB)) {
    *data = (int16_t) (buffer[0] | (buffer[1] << 8));
  } else {
    *data = (int16_t) (buffer[1] | (buffer[0] << 8));
  }
  return status;
}

/**
 * @brief Read the current configuration
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[out]  cfg       The current configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_config(const L3G4200DDriver* const l3gd, l3g4200d_lld_cfg_t* const cfg)
{
  apalDbgAssert(cfg != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_CTRL_REG1, cfg->data, 5);
}

/**
 * @brief Write the a configuration
 * @param[in]   l3gd      The L3G4200D driver to use.
 * @param[in]   cfg       The new configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_write_config(const L3G4200DDriver* const l3gd, const l3g4200d_lld_cfg_t cfg)
{
  return l3g4200d_lld_write_register(l3gd, L3G4200D_LLD_REGISTER_CTRL_REG1, cfg.data, 5);
}

/**
 * @brief Read the interrupt configuration of one of the interrupts.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  cfg         The current interrupt configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_int_config(const L3G4200DDriver* const l3gd, l3g4200d_lld_int_cfg_t* const cfg)
{
  apalDbgAssert(cfg != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_INT1_CFG, cfg->data, 9);
}

/**
 * @brief Write the interrupt configuration.
 * @param[in]   spid        The SPI driver to use.
 * @param[in]   cfg         The current new configuration.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_write_int_config(const L3G4200DDriver* const l3gd, const l3g4200d_lld_int_cfg_t cfg)
{
  apalExitStatus_t status = l3g4200d_lld_write_register(l3gd, L3G4200D_LLD_REGISTER_INT1_CFG, cfg.data, 1);
  if (status != APAL_STATUS_OK) {
    return status;
  }
  return l3g4200d_lld_write_register(l3gd, L3G4200D_LLD_REGISTER_INT1_TSH_XH, &cfg.data[2], 7);
}

/**
 * @brief Read the status register.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  status      The current status register.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_status_register(const L3G4200DDriver* const l3gd, uint8_t* const status)
{
  apalDbgAssert(status != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_STATUS_REG, status, 1);
}

/**
 * @brief Read the fifo ctrl register.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  fifo        The current fifo ctrl register.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_fifo_ctrl_register(const L3G4200DDriver* const l3gd, uint8_t* const fifo)
{
  apalDbgAssert(fifo != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_FIFO_CTRL_REG, fifo, 1);
}

/**
 * @brief Write the fifo ctrl register.
 * @param[in]   spid        The SPI driver to use.
 * @param[in]   fifo        The new fifo ctrl register.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_write_fifo_ctrl_register(const L3G4200DDriver* const l3gd, const uint8_t fifo)
{
  return l3g4200d_lld_write_register(l3gd, L3G4200D_LLD_REGISTER_FIFO_CTRL_REG, &fifo, 1);
}

/**
 * @brief Read the fifo src register.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  fifo        The current fifo src register.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_fifo_src_register(const L3G4200DDriver* const l3gd, uint8_t* const fifo)
{
  apalDbgAssert(fifo != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_FIFO_SRC_REG, fifo, 1);
}

/**
 * @brief Read the interrupt src register.
 * @param[in]   spid        The SPI driver to use.
 * @param[out]  cfg         The current fifo src register.
 *
 * @return  The return status indicates whether the function call was succesfull or a timeout occurred.
 */
inline apalExitStatus_t
l3g4200d_lld_read_int_src(const L3G4200DDriver* const l3gd, uint8_t* const cfg)
{
  apalDbgAssert(cfg != NULL);

  return l3g4200d_lld_read_register(l3gd, L3G4200D_LLD_REGISTER_INT1_SRC, cfg, 1);
}

#endif /* defined(AMIROLLD_CFG_USE_L3G4200D) */
