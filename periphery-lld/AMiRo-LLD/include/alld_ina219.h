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

#ifndef _AMIROLLD_INA219_H_
#define _AMIROLLD_INA219_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_INA219) || defined(__DOXYGEN__)

/**
 * @brief Maximum I2C frequency.
 */
#define INA219_LLD_I2C_MAXFREQUENCY   2560000

/**
 * @brief I2C address masks.
 */
enum {
  INA219_LLD_I2C_ADDR_FIXED   = 0x40u,
  INA219_LLD_I2C_ADDR_A0      = 0x01u,
  INA219_LLD_I2C_ADDR_A1      = 0x04u,
};

/**
 * @brief Registers.
 */
typedef enum {
  INA219_LLD_REGISTER_CONFIGURATION = 0x00,
  INA219_LLD_REGISTER_SHUNT_VOLTAGE = 0x01,
  INA219_LLD_REGISTER_BUS_VOLTAGE   = 0x02,
  INA219_LLD_REGISTER_POWER         = 0x03,
  INA219_LLD_REGISTER_CURRENT       = 0x04,
  INA219_LLD_REGISTER_CALIBRATION   = 0x05,
} ina219_lld_register_t;

/**
 * @brief BRNG.
 */
typedef enum {
  INA219_LLD_BRNG_16 = 0x00,
  INA219_LLD_BRNG_32 = 0x01,
} ina219_lld_brng_t;

/**
 * @brief Gain settings.
 */
typedef enum {
  INA219_LLD_GAIN_1_40mV = 0x00,
  INA219_LLD_GAIN_2_80mV = 0x01,
  INA219_LLD_GAIN_4_160mV = 0x02,
  INA219_LLD_GAIN_8_320mV = 0x03,
} ina219_lld_gain_t;

/**
 * @brief ADC settings.
 */
typedef enum {
  INA219_LLD_ADC_9BIT = 0x00,
  INA219_LLD_ADC_10BIT = 0x01,
  INA219_LLD_ADC_11BIT = 0x02,
  INA219_LLD_ADC_12BIT = 0x03,
  INA219_LLD_ADC_2 = 0x09,
  INA219_LLD_ADC_4 = 0x0A,
  INA219_LLD_ADC_8 = 0x0B,
  INA219_LLD_ADC_16 = 0x0C,
  INA219_LLD_ADC_32 = 0x0D,
  INA219_LLD_ADC_64 = 0x0E,
  INA219_LLD_ADC_128 = 0x0f,
} ina219_lld_adc_t;

/**
 * @brief Mode settings.
 */
typedef enum {
  INA219_LLD_MODE_POWERDOWN = 0x00,
  INA219_LLD_MODE_SHUNT_TRIGGERED = 0x01,
  INA219_LLD_MODE_BUS_TRIGGERED = 0x02,
  INA219_LLD_MODE_SHUNT_BUS_TRIGGERED = 0x03,
  INA219_LLD_MODE_ADC_OFF = 0x04,
  INA219_LLD_MODE_SHUNT_CONTINUOUS = 0x05,
  INA219_LLD_MODE_BUS_CONTINUOUS = 0x06,
  INA219_LLD_MODE_SHUNT_BUS_CONTINUOUS = 0x07,
} ina219_lld_mode_t;

/**
 * @brief Config register.
 */
typedef union {
  uint16_t data;
  struct {
    ina219_lld_mode_t mode : 3;
    ina219_lld_adc_t sadc : 4;
    ina219_lld_adc_t badc : 4;
    ina219_lld_gain_t gain : 2;
    ina219_lld_brng_t brng : 1;
    uint8_t zero : 1;
    uint8_t reset : 1;
  } options;
} ina219_lld_cfg_t;

/**
 * @brief Calibration input struct.
 */
typedef struct {
  float shunt_resistance_0;
  float max_expected_current_A;
  uint16_t current_lsb_uA;
  ina219_lld_cfg_t cfg_reg;
} ina219_lld_calib_input_t;

/**
 * @brief Calibration output struct.
 */
typedef struct {
  float max_current_before_overflow_A;
  float max_shunt_voltage_before_overflow_V;
  uint16_t current_lsb_uA;
  uint16_t calibration;
} ina219_lld_calib_output_t;

/**
 * @brief The INA219Driver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;
  uint8_t addr; /**<The address of the INA219 for I2C communication, which is defined by the wiring of the A0 and A1 pins */
  uint16_t current_lsb_uA;
  ina219_lld_cfg_t *config;
} INA219Driver;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t ina219_lld_calibration(INA219Driver* const ina219, const ina219_lld_calib_input_t* const calib_data, ina219_lld_calib_output_t* const calib_out);
  apalExitStatus_t ina219_lld_read_register(const INA219Driver* const ina219, const ina219_lld_register_t addr, uint16_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_write_register(const INA219Driver* const ina219, const ina219_lld_register_t addr, const uint16_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_config(const INA219Driver* const ina219, ina219_lld_cfg_t* const cfg, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_write_config(const INA219Driver* const ina219, const ina219_lld_cfg_t cfg, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_calibration(const INA219Driver* const ina219, uint16_t* const calib, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_write_calibration(const INA219Driver* const ina219, const uint16_t calib, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_reset(const INA219Driver* const ina219, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_shunt_voltage(const INA219Driver* const ina219, int32_t* const data, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_bus_voltage(const INA219Driver* const ina219, uint32_t* const data, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_power(const INA219Driver* const ina219, uint32_t* const data, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_read_current(const INA219Driver* const ina219, int16_t* const data, const apalTime_t timeout);
  apalExitStatus_t ina219_lld_bus_conversion_ready(const INA219Driver* const ina219, uint16_t* const buscnv, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_INA219) */

#endif /* _AMIROLLD_INA219_H_ */
