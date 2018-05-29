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

#ifndef _AMIROLLD_MPR121_H_
#define _AMIROLLD_MPR121_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_MPR121) || defined(__DOXYGEN__)

/**
 * @brief Maximum I2C frequency.
 */
#define MPR121_LLD_I2C_MAXFREQUENCY   400000

/**
 * @brief A falling edge indicates an interrupt.
 */
#define MPR121_LLD_INT_EDGE           APAL_GPIO_EDGE_FALLING

/**
 * @brief The MPR121 driver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;        /**< @brief The I2C Driver */
} MPR121Driver;

enum {
  MPR121_LLD_I2C_ADDR_FIXED   = 0x5Au,
  MPR121_LLD_I2C_ADDR_VSS     = 0x00u,
  MPR121_LLD_I2C_ADDR_VDD     = 0x01u,
  MPR121_LLD_I2C_ADDR_SDA     = 0x04u,
  MPR121_LLD_I2C_SOFTRESET    = 0x63,
};

/**
 * @brief Registers of the mpr121.
 */
typedef enum {
  MPR121_LLD_REGISTER_TOUCH_STATUS  = 0x00,
  MPR121_LLD_REGISTER_OOR_STATUS    = 0x02,
  MPR121_LLD_REGISTER_FILTERED_DATA = 0x04,
  MPR121_LLD_REGISTER_BASELINE      = 0x1E,

  MPR121_LLD_REGISTER_MHD_RISING    = 0x2B,
  MPR121_LLD_REGISTER_NHD_RISING    = 0x2C,
  MPR121_LLD_REGISTER_NCL_RISING    = 0x2D,
  MPR121_LLD_REGISTER_FDL_RISING    = 0x2E,
  MPR121_LLD_REGISTER_MHD_FALLING   = 0x2F,
  MPR121_LLD_REGISTER_NHD_FALLING   = 0x30,
  MPR121_LLD_REGISTER_NCL_FALLING   = 0x31,
  MPR121_LLD_REGISTER_FDL_FALLING   = 0x32,
  MPR121_LLD_REGISTER_NHD_TOUCHED   = 0x33,
  MPR121_LLD_REGISTER_NCL_TOUCHED   = 0x34,
  MPR121_LLD_REGISTER_FDL_TOUCHED   = 0x35,

  MPR121_LLD_REGISTER_ELEPROX_MHD_RISING    = 0x36,
  MPR121_LLD_REGISTER_ELEPROX_NHD_RISING    = 0x37,
  MPR121_LLD_REGISTER_ELEPROX_NCL_RISING    = 0x38,
  MPR121_LLD_REGISTER_ELEPROX_FDL_RISING    = 0x39,
  MPR121_LLD_REGISTER_ELEPROX_MHD_FALLING   = 0x3A,
  MPR121_LLD_REGISTER_ELEPROX_NHD_FALLING   = 0x3B,
  MPR121_LLD_REGISTER_ELEPROX_NCL_FALLING   = 0x3C,
  MPR121_LLD_REGISTER_ELEPROX_FDL_FALLING   = 0x3D,
  MPR121_LLD_REGISTER_ELEPROX_NHD_TOUCHED   = 0x3E,
  MPR121_LLD_REGISTER_ELEPROX_NCL_TOUCHED   = 0x3F,
  MPR121_LLD_REGISTER_ELEPROX_FDL_TOUCHED   = 0x40,

  MPR121_LLD_REGISTER_THRESHOLDS  = 0x41,
  MPR121_LLD_REGISTER_DEBOUNCE    = 0x5B,
  MPR121_LLD_REGISTER_CONFIG_1    = 0x5C,
  MPR121_LLD_REGISTER_CONFIG_2    = 0x5D,
  MPR121_LLD_REGISTER_CONFIG_ECR  = 0x5E,
  MPR121_LLD_REGISTER_ELE_CURRENT = 0x5F,
  MPR121_LLD_REGISTER_CHARGE_TIME = 0x6C,

  MPR121_LLD_REGISTER_GPIO_CTRL_0         = 0x73,
  MPR121_LLD_REGISTER_GPIO_CTRL_1         = 0x74,
  MPR121_LLD_REGISTER_GPIO_DATA           = 0x75,
  MPR121_LLD_REGISTER_GPIO_DIRECTION_CTRL = 0x76,
  MPR121_LLD_REGISTER_GPIO_ENABLE         = 0x77,
  MPR121_LLD_REGISTER_GPIO_DATA_SET       = 0x78,
  MPR121_LLD_REGISTER_GPIO_DATA_CLEAR     = 0x79,
  MPR121_LLD_REGISTER_GPIO_DATA_TOGGLE    = 0x7A,

  MPR121_LLD_REGISTER_AUTOCFG_CTRL_0       = 0x7B,
  MPR121_LLD_REGISTER_AUTOCFG_CTRL_1       = 0x7C,
  MPR121_LLD_REGISTER_AUTOCFG_USL          = 0x7D,
  MPR121_LLD_REGISTER_AUTOCFG_LSL          = 0x7E,
  MPR121_LLD_REGISTER_AUTOCFG_TARGET_LEVEL = 0x7F,

  MPR121_LLD_REGISTER_SOFT_RESET           = 0x80
} mpr121_lld_register_t;

/**
 * @brief Configuration data type.
 */
typedef union {
  uint8_t values[8];
  struct {
    uint8_t auto_config_1;
    uint8_t auto_config_2;
    uint8_t up_side_limit;
    uint8_t low_side_limit;
    uint8_t target_level;
    uint8_t config_1;
    uint8_t config_2;
    uint8_t ele_config;
  } registers;
} mpr121_lld_config_t;

/**
 * €brief Configutation register 1 masks.
 */
typedef enum {
  MPR121_LLD_FFI_6     = 0x00u,
  MPR121_LLD_FFI_10    = 0x40u,
  MPR121_LLD_FFI_18    = 0x80u,
  MPR121_LLD_FFI_34    = 0xC0u,
} mpr121_lld_config1_t;

/**
 * @brief Configuration register 2 masks.
 */
typedef enum {
  MPR121_LLD_CDT_OFF   = 0x00u,
  MPR121_LLD_CDT_0_5   = 0x20u,
  MPR121_LLD_CDT_1     = 0x40u,
  MPR121_LLD_CDT_2     = 0x60u,
  MPR121_LLD_CDT_4     = 0x80u,
  MPR121_LLD_CDT_8     = 0xA0u,
  MPR121_LLD_CDT_16    = 0xC0u,
  MPR121_LLD_CDT_32    = 0xE0u,
  MPR121_LLD_SFI_4     = 0x00u,
  MPR121_LLD_SFI_6     = 0x08u,
  MPR121_LLD_SFI_10    = 0x10u,
  MPR121_LLD_SFI_18    = 0x18u,
  MPR121_LLD_ESI_1     = 0x00u,
  MPR121_LLD_ESI_2     = 0x01u,
  MPR121_LLD_ESI_4     = 0x02u,
  MPR121_LLD_ESI_8     = 0x03u,
  MPR121_LLD_ESI_16    = 0x04u,
  MPR121_LLD_ESI_32    = 0x05u,
  MPR121_LLD_ESI_64    = 0x06u,
  MPR121_LLD_ESI_128   = 0x07u,
} mpr121_lld_config2_t;

/**
 * @brief Ele configuration register masks.
 */
typedef enum {
  MPR121_LLD_CL_ON_CURRENT  = 0x00u,
  MPR121_LLD_CL_OFF         = 0x40u,
  MPR121_LLD_CL_ON_HIGH5    = 0x80u,
  MPR121_LLD_CL_ON_ALL      = 0xC0u,
  MPR121_LLD_ELEPROX_0      = 0x00u,
  MPR121_LLD_ELEPROX_2      = 0x10u,
  MPR121_LLD_ELEPROX_4      = 0x20u,
  MPR121_LLD_ELEPROX_12     = 0x30u,
} mpr121_lld_ele_config_t;

/**
 * @brief Auto configuration register 0 masks.
 */
typedef enum {
  MPR121_LLD_AC_ENABLE      = 0x01u,
  MPR121_LLD_AC_RECONF_EN   = 0x02u,
  MPR121_LLD_BVA_ON_CURRENT = 0x00u,
  MPR121_LLD_BVA_OFF        = 0x04u,
  MPR121_LLD_BVA_ON_HIGH5   = 0x08u,
  MPR121_LLD_BVA_ON_ALL     = 0x0Cu,
  MPR121_LLD_RETRY_OFF      = 0x00u,
  MPR121_LLD_RETRY_2        = 0x10u,
  MPR121_LLD_RETRY_4        = 0x20u,
  MPR121_LLD_RETRY_8        = 0x30u,
  /* skip FFI */
} mpr121_lld_auto_config0_t;

/**
 * @brief Auto configuration register 1 masks.
 */
typedef enum {
  MPR121_LLD_AC_FAIL_INT_EN         = 0x01u,
  MPR121_LLD_AC_RECONF_FAIL_INT_EN  = 0x02u,
  MPR121_LLD_AC_OUT_OF_RANGE_INT_EN = 0x04u,
  MPR121_LLD_AC_SKIP_CHRG_TIME_SRCH = 0x80u,
} mpr121_lld_auto_config1_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t mpr121_lld_read_register(const MPR121Driver* const mprd, const mpr121_lld_register_t regaddr, const uint8_t offset, const uint8_t size, uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_write_register(const MPR121Driver* const mprd, const mpr121_lld_register_t regaddr, const uint8_t offset, const uint8_t size, const uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_soft_reset(const MPR121Driver* const mprd, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_read_filtered_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint16_t* const data, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_read_baseline_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_read_electrode_data(const MPR121Driver* const mprd, const uint8_t index, const uint8_t num, uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_write_config(const MPR121Driver* const mprd, const mpr121_lld_config_t cfg, const apalTime_t timeout);
  apalExitStatus_t mpr121_lld_read_config(const MPR121Driver* const mprd, mpr121_lld_config_t* const cfg, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_MPR121) */

#endif /* _AMIROLLD_MPR121_H_ */
