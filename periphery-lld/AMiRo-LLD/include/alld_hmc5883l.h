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

#ifndef _AMIROLLD_HMC5883L_H_
#define _AMIROLLD_HMC5883L_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_HMC5883L) || defined(__DOXYGEN__)

/**
 * @brief The HMC5883L driver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;        /**< @brief The I2C Driver */
} HMC5883LDriver;

/**
 * @brief Constant I2C address.
 */
#define HMC5883L_LLD_I2C_ADDR           0x1Eu

/**
 * @brief Maximum I2C frequency.
 */
#define HMC5883L_LLD_I2C_MAXFREQUENCY   400000

/**
 * @brief A falling edge indicates an interrupt.
 */
#define HMC5883L_LLD_INT_EDGE           APAL_GPIO_EDGE_FALLING

/**
 * @brief Register enum.
 */
typedef enum {
  HMC5883L_LLD_REGISTER_CONFIG_A         = 0x00u,
  HMC5883L_LLD_REGISTER_CONFIG_B         = 0x01u,
  HMC5883L_LLD_REGISTER_MODE             = 0x02u,
  HMC5883L_LLD_REGISTER_DATA_OUT_X_MSB   = 0x03u,
  HMC5883L_LLD_REGISTER_DATA_OUT_X_LSB   = 0x04u,
  HMC5883L_LLD_REGISTER_DATA_OUT_Z_MSB   = 0x05u,
  HMC5883L_LLD_REGISTER_DATA_OUT_Z_LSB   = 0x06u,
  HMC5883L_LLD_REGISTER_DATA_OUT_Y_MSB   = 0x07u,
  HMC5883L_LLD_REGISTER_DATA_OUT_Y_LSB   = 0x08u,
  HMC5883L_LLD_REGISTER_STATUS           = 0x09u,
  HMC5883L_LLD_REGISTER_IDENTIFICATION_A = 0x0Au,
  HMC5883L_LLD_REGISTER_IDENTIFICATION_B = 0x0Bu,
  HMC5883L_LLD_REGISTER_IDENTIFICATION_C = 0x0Cu,
} hmc5883l_lld_register_t;

/**
 * @brief Averaging options.
 */
typedef enum {
  HMC5883L_LLD_AVG1 = 0x00,
  HMC5883L_LLD_AVG2 = 0x20,
  HMC5883L_LLD_AVG4 = 0x40,
  HMC5883L_LLD_AVG8 = 0x60,
} hmc5883l_lld_averaging_t;

/**
 * @brief Data output rate.
 */
typedef enum {
  HMC5883L_LLD_0_75_HZ = 0x00,
  HMC5883L_LLD_1_5_HZ  = 0x04,
  HMC5883L_LLD_3_HZ    = 0x08,
  HMC5883L_LLD_7_5_HZ  = 0x0C,
  HMC5883L_LLD_15_HZ   = 0x10,
  HMC5883L_LLD_30_HZ   = 0x14,
  HMC5883L_LLD_75_HZ   = 0x18,
} hmc5883l_lld_data_output_rate_t;

/**
 * @brief Measurement bias.
 */
typedef enum {
  HMC5883L_LLD_MB_NORMAL        = 0x00,
  HMC5883L_LLD_MB_POSITIVE_BIAS = 0x01,
  HMC5883L_LLD_MB_NEGATIVE_BIAS = 0x02,
} hmc5883l_lld_measurement_bias_t;

/**
 * @brief Gain settings.
 */
typedef enum {
  HMC5883L_LLD_GN_0_GA = 0x00,
  HMC5883L_LLD_GN_1_GA = 0x20,
  HMC5883L_LLD_GN_2_GA = 0x40,
  HMC5883L_LLD_GN_3_GA = 0x60,
  HMC5883L_LLD_GN_4_GA = 0x80,
  HMC5883L_LLD_GN_5_GA = 0xA0,
  HMC5883L_LLD_GN_6_GA = 0xC0,
  HMC5883L_LLD_GN_7_GA = 0xE0,
} hmc5883l_lld_gain_t;

/**
 * @brief Highspeed enable settings.
 */
typedef enum {
  HMC5883L_LLD_HS_DISABLE = 0x00,
  HMC5883L_LLD_HS_ENABLE  = 0x80,
} hmc5883l_lld_highspeed_t;

/**
 * @brief Measurement modes.
 */
typedef enum {
  HMC5883L_LLD_MM_CONTINUOUS = 0x00,
  HMC5883L_LLD_MM_SINGLE     = 0x01,
  HMC5883L_LLD_MM_IDLE       = 0x02,
} hmc5883l_lld_measurement_mode_t;

/**
 * @brief Read or write access.
 */
typedef enum {
  HMC5883L_LLD_READ  = 0x3Du,
  HMC5883L_LLD_WRITE = 0x3Cu,
} hmc5883l_lld_access_mode_t;

/**
 * @brief Status lock or ready.
 */
typedef enum {
  HMC5883L_LLD_LOCK = 0x0u,
  HMC5883L_LLD_RDY  = 0x1u,
} hmc5883l_lld_status_t;

/**
 * @brief Identification values.
 */
typedef enum {
  HMC5883L_LLD_IDENTIFICATION_A = 0x48,
  HMC5883L_LLD_IDENTIFICATION_B = 0x34,
  HMC5883L_LLD_IDENTIFICATION_C = 0x33,
} hmc5883l_lld_identification_t;

/**
 * @brief HMC5883L configuration struct.
 */
typedef struct {
  hmc5883l_lld_averaging_t avg;
  hmc5883l_lld_data_output_rate_t outrate;
  hmc5883l_lld_measurement_bias_t mbias;
  hmc5883l_lld_gain_t gain;
  hmc5883l_lld_highspeed_t highspeed;
  hmc5883l_lld_measurement_mode_t mode;
} hmc5883l_lld_config_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t hmc5883l_lld_check(const HMC5883LDriver* const hmcd, uint8_t* const rxbuffer, const uint8_t size, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_write_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, const uint8_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_set_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, const uint8_t data, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_register(const HMC5883LDriver* const hmcd, const hmc5883l_lld_register_t regaddr, uint8_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_data(const HMC5883LDriver* const hmcd, uint16_t* const data, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_write_config(const HMC5883LDriver* const hmcd, const hmc5883l_lld_config_t conf, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_config(const HMC5883LDriver* const hmcd, hmc5883l_lld_config_t* const conf, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_status(const HMC5883LDriver* const hmcd, uint8_t* const status, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_lock(const HMC5883LDriver* const hmcd, uint8_t* const lock, const apalTime_t timeout);
  apalExitStatus_t hmc5883l_lld_read_rdy(const HMC5883LDriver* const hmcd, uint8_t* const rdy, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_HMC5883L) */

#endif /* _AMIROLLD_HMC5883L_H_ */
