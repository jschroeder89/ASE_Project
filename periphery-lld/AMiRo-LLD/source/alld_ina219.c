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

#include <alld_ina219.h>

#if defined(AMIROLLD_CFG_USE_INA219) || defined(__DOXYGEN__)

/**
 * @brief Read the value of one or more of the registers.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[in]   addr        register address
 * @param[out]  data        register content
 * @param[in]   num         number of subsequent registers to read
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_register(const INA219Driver* const ina219, const ina219_lld_register_t addr, uint16_t* const data, const uint8_t num, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(ina219->i2cd != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[num*2];
  apalExitStatus_t status = apalI2CMasterTransmit(ina219->i2cd, (INA219_LLD_I2C_ADDR_FIXED | ina219->addr), (uint8_t*)&addr, 1, buffer, 2*num, timeout);
  for (uint8_t dataIdx = 0; dataIdx < num; dataIdx++) {
    data[dataIdx] = (buffer[2*dataIdx] << 8) | buffer[2*dataIdx+1];
  }
  return status;
}

/**
 * @brief Write the value of one or more of the registers.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[in]   addr        register address
 * @param[in]   data        data to write
 * @param[in]   num         number of subsequent registers to read
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_write_register(const INA219Driver* const ina219, const ina219_lld_register_t addr, const uint16_t* const data, const uint8_t num, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(ina219->i2cd != NULL);
  apalDbgAssert(data != NULL);

  uint8_t buffer[1+2*num];
  buffer[0] = addr;
  for (uint8_t dataIdx = 0; dataIdx < num; dataIdx++) {
    buffer[dataIdx*2+1] = data[dataIdx] >> 8;
    buffer[dataIdx*2+2] = data[dataIdx] & (0x00FFu);
  }
  return apalI2CMasterTransmit(ina219->i2cd, (INA219_LLD_I2C_ADDR_FIXED | ina219->addr), buffer, 1+2*num, NULL, 0, timeout);
}

/**
 * @brief Calibrate the ina219.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        APAL_STATUS_SUCCESS;
 * @param[in]   calib_in    calibration input data
 * @param[out]  calib_out   output of the calibration
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_calibration(INA219Driver* const ina219, const ina219_lld_calib_input_t* const calib_in, ina219_lld_calib_output_t* const calib_out)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(calib_in != NULL);
  apalDbgAssert(calib_out != NULL);

  uint16_t current_lsb_uA = calib_in->current_lsb_uA;
  if (current_lsb_uA < calib_in->max_expected_current_A / 0.032768f) {
    current_lsb_uA = calib_in->max_expected_current_A / 0.032768f;
  } else if (current_lsb_uA > calib_in->max_expected_current_A / 0.004096f) {
    current_lsb_uA = calib_in->max_expected_current_A / 0.004096f;
  }

  uint16_t calib_value = (uint16_t) (40960 / (current_lsb_uA * calib_in->shunt_resistance_0));

  float V_shunt_max;
  switch (calib_in->cfg_reg.options.gain) {
    case INA219_LLD_GAIN_1_40mV:
        V_shunt_max = 0.04f;
        break;
    case INA219_LLD_GAIN_2_80mV:
        V_shunt_max = 0.08f;
        break;
    case INA219_LLD_GAIN_4_160mV:
        V_shunt_max = 0.16f;
        break;
    case INA219_LLD_GAIN_8_320mV:
        V_shunt_max = 0.32f;
        break;
  }

  calib_out->max_current_before_overflow_A = (current_lsb_uA * 0.032767f >= V_shunt_max /calib_in->shunt_resistance_0) ?
                                                V_shunt_max / calib_in->shunt_resistance_0 :
                                                current_lsb_uA * 0.032767f;
  calib_out->max_shunt_voltage_before_overflow_V =
                                ((calib_out->max_current_before_overflow_A * calib_in->shunt_resistance_0) >= V_shunt_max) ?
                                                   V_shunt_max :
                                                   calib_out->max_current_before_overflow_A * calib_in->shunt_resistance_0;

  calib_out->current_lsb_uA = current_lsb_uA;
  calib_out->calibration = calib_value;

  ina219->current_lsb_uA = current_lsb_uA;

  return APAL_STATUS_SUCCESS;
}

/**
 * @brief Read the current configuration.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  cfg         current configuration
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_config(const INA219Driver* const ina219, ina219_lld_cfg_t* const cfg, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(cfg != NULL);

  return ina219_lld_read_register(ina219, INA219_LLD_REGISTER_CONFIGURATION, &cfg->data, 1, timeout);
}

/**
 * @brief Write a new configuration.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[in]   cfg         new configuration
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_write_config(const INA219Driver* const ina219, ina219_lld_cfg_t cfg, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);

  return ina219_lld_write_register(ina219, INA219_LLD_REGISTER_CONFIGURATION, &cfg.data, 1, timeout);
}

/**
 * @brief Read the current calibration value.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  calib       current calibration value
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_calibration(const INA219Driver* const ina219, uint16_t* const calib, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(calib != NULL);

  return ina219_lld_read_register(ina219, INA219_LLD_REGISTER_CALIBRATION, calib, 1, timeout);
}

/**
 * @brief Write a new calibration value.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[in]   calib       new calibration value
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_write_calibration(const INA219Driver* const ina219, const uint16_t calib, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);

  return ina219_lld_write_register(ina219, INA219_LLD_REGISTER_CALIBRATION, &calib, 1, timeout);
}

/**
 * @brief Reset the ina219.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_reset(const INA219Driver* const ina219, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);

  uint16_t data = 0x8000;
  return ina219_lld_write_register(ina219, INA219_LLD_REGISTER_CONFIGURATION, &data, 1, timeout);
}

/**
 * @brief Read the current shunt voltage (uV).
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  data        shunt voltage
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_shunt_voltage(const INA219Driver* const ina219, int32_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(data != NULL);

  uint16_t buffer = 0;
  apalExitStatus_t status = ina219_lld_read_register(ina219, INA219_LLD_REGISTER_SHUNT_VOLTAGE, &buffer, 1, timeout);

  switch (ina219->config->options.gain) {
    case INA219_LLD_GAIN_8_320mV:
      *data = (int32_t)(buffer & 0x7FFFu) * ((buffer & 0x8000u) ? -10 : 10);
      break;
    case INA219_LLD_GAIN_4_160mV:
      *data = (int32_t)(buffer & 0x3FFFu) * ((buffer & 0xC000u) ? -10 : 10);
      break;
    case INA219_LLD_GAIN_2_80mV:
      *data = (int32_t)(buffer & 0x1FFFu) * ((buffer & 0xE000u) ? -10 : 10);
      break;
    case INA219_LLD_GAIN_1_40mV:
      *data = (int32_t)(buffer & 0x0FFFu) * ((buffer & 0xF000u) ? -10 : 10);
      break;
  }

  return status;
}

/**
 * @brief Read the current bus voltage (uV).
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  data        bus voltage
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_bus_voltage(const INA219Driver* const ina219, uint32_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(data != NULL);

  uint16_t buffer = 0;
  apalExitStatus_t status = ina219_lld_read_register(ina219, INA219_LLD_REGISTER_BUS_VOLTAGE, &buffer, 1, timeout);
  if (buffer & 0x01) {
    status = APAL_STATUS_ERROR;
  }
  *data = (uint32_t)(buffer >> 3) * 4000;
  return status;
}

/**
 * @brief Read the power (uW).
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  data        power
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_power(const INA219Driver* const ina219, uint32_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(data != NULL);

  uint16_t buffer = 0;
  apalExitStatus_t status = ina219_lld_read_register(ina219, INA219_LLD_REGISTER_POWER, &buffer, 1, timeout);
  *data = (uint32_t)buffer * 20 * ina219->current_lsb_uA;
  return status;
}

/**
 * @brief Read the current (uA).
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  data        current
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_read_current(const INA219Driver* const ina219, int16_t* const data, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(data != NULL);

  uint16_t buffer = 0;
  apalExitStatus_t status = ina219_lld_read_register(ina219, INA219_LLD_REGISTER_CURRENT, &buffer, 1, timeout);
  *data = ((int16_t) buffer) * ina219->current_lsb_uA;
  return status;
}

/**
 * @brief Read the bus conversion ready flag.
 * @param[in]   i2cd        i2c driver
 * @param[in]   inad        ina219 driver
 * @param[out]  buscnv      conversion ready flag
 * @param[in]   timeout     timeout
 * @return                  An indicator whether the call was successfull
 */
inline apalExitStatus_t
ina219_lld_bus_conversion_ready(const INA219Driver* const ina219, uint16_t* const buscnv, const apalTime_t timeout)
{
  apalDbgAssert(ina219 != NULL);
  apalDbgAssert(buscnv != NULL);

  apalExitStatus_t status = ina219_lld_read_register(ina219, INA219_LLD_REGISTER_BUS_VOLTAGE, buscnv, 1, timeout);
  *buscnv >>= 1;
  *buscnv &= 1;
  return status;
}

#endif /* defined(AMIROLLD_CFG_USE_INA219) */
