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

#ifndef _AMIROLLD_LIS331DLH_H_
#define _AMIROLLD_LIS331DLH_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_LIS331DLH) || defined(__DOXYGEN__)

/**
 * @brief A falling edge indicates an interrupt.
 */
#define LIS331DLH_LLD_INT_EDGE  APAL_GPIO_EDGE_FALLING

/**
 * @brief The LIS331DLH driver struct
 */
typedef struct {
  apalSPIDriver_t* spid;        /**< @brief The SPI Driver */
} LIS331DLHDriver;

/**
 * @brief SPI access modes.
 */
typedef enum {
  LIS331DLH_LLD_SPI_MULT  = 0x40u,
  LIS331DLH_LLD_SPI_READ  = 0x80u,
  LIS331DLH_LLD_SPI_WRITE = 0x00u,
} lis331dlh_lld_SPI_mode_t;

/**
 * @brief Registers.
 */
typedef enum {
  LIS331DLH_LLD_REGISTER_WHO_AM_I        = 0x0fu,
  LIS331DLH_LLD_REGISTER_CTRL_REG1       = 0x20u,
  LIS331DLH_LLD_REGISTER_CTRL_REG2       = 0x21u,
  LIS331DLH_LLD_REGISTER_CTRL_REG3       = 0x22u,
  LIS331DLH_LLD_REGISTER_CTRL_REG4       = 0x23u,
  LIS331DLH_LLD_REGISTER_CTRL_REG5       = 0x24u,
  LIS331DLH_LLD_REGISTER_HP_FILTER_RESET = 0x25u,
  LIS331DLH_LLD_REGISTER_REFERENCE       = 0x26u,
  LIS331DLH_LLD_REGISTER_STATUS_REG      = 0x27u,
  LIS331DLH_LLD_REGISTER_OUT_X_L         = 0x28u,
  LIS331DLH_LLD_REGISTER_OUT_X_H         = 0x29u,
  LIS331DLH_LLD_REGISTER_OUT_Y_L         = 0x2Au,
  LIS331DLH_LLD_REGISTER_OUT_Y_H         = 0x2Bu,
  LIS331DLH_LLD_REGISTER_OUT_Z_L         = 0x2Cu,
  LIS331DLH_LLD_REGISTER_OUT_Z_H         = 0x2Du,
  LIS331DLH_LLD_REGISTER_INT1_CFG        = 0x30u,
  LIS331DLH_LLD_REGISTER_INT1_SOURCE     = 0x31u,
  LIS331DLH_LLD_REGISTER_INT1_THS        = 0x32u,
  LIS331DLH_LLD_REGISTER_INT1_DURATION   = 0x33u,
  LIS331DLH_LLD_REGISTER_INT2_CFG        = 0x34u,
  LIS331DLH_LLD_REGISTER_INT2_SOURCE     = 0x35u,
  LIS331DLH_LLD_REGISTER_INT2_THS        = 0x36u,
  LIS331DLH_LLD_REGISTER_INT2_DURATION   = 0x37u,
} lis331dlh_lld_register_t;

/**
 * @brief WHO_AM_I register constant content.
 */
typedef enum {
  LIS331DLH_LLD_WHO_AM_I = 0x32,
} lis331dlh_lld_whoami_t;

/**
 * @brief Control register 1 flags.
 */
typedef enum {
  LIS331DLH_LLD_PM_OFF          = 0x00u,
  LIS331DLH_LLD_PM_ODR          = 0x20u,
  LIS331DLH_LLD_PM_0_5_HZ       = 0x40u,
  LIS331DLH_LLD_PM_1_HZ         = 0x60u,
  LIS331DLH_LLD_PM_2_HZ         = 0x80u,
  LIS331DLH_LLD_PM_5_HZ         = 0xA0u,
  LIS331DLH_LLD_PM_10_HZ        = 0xC0u,
  LIS331DLH_LLD_DR_50HZ_37LP    = 0x00u,
  LIS331DLH_LLD_DR_100HZ_74LP   = 0x08u,
  LIS331DLH_LLD_DR_400HZ_292LP  = 0x10u,
  LIS331DLH_LLD_DR_1000HZ_780LP = 0x18u,
  LIS331DLH_LLD_X_AXIS_ENABLE   = 0x01u,
  LIS331DLH_LLD_Y_AXIS_ENABLE   = 0x02u,
  LIS331DLH_LLD_Z_AXIS_ENABLE   = 0x04u,
} lis331dlh_lld_ctrl_reg1_t;

/**
 * @brief Control register 2 flags.
 */
typedef enum {
  LIS331DLH_LLD_REBOOT = 0x80u,
  LIS331DLH_LLD_HPM_NORMAL0  = 0x00u,
  LIS331DLH_LLD_HPM_REF      = 0x20u,
  LIS331DLH_LLD_HPM_NORMAL1  = 0x40u,
  LIS331DLH_LLD_FDS_BYPASS = 0x00u,
  LIS331DLH_LLD_FDS_FILTER = 0x10u,
  LIS331DLH_LLD_HP_EN_INT2  = 0x08u,
  LIS331DLH_LLD_HP_EN_INT1  = 0x04u,
  LIS331DLH_LLD_HPCF_8       = 0x00u,
  LIS331DLH_LLD_HPCF_16      = 0x01u,
  LIS331DLH_LLD_HPCF_32      = 0x02u,
  LIS331DLH_LLD_HPCF_64      = 0x03u,
} lis331dlh_lld_ctrl_reg2_t;

/**
 * @brief Control register 3 flags.
 */
typedef enum {
  LIS331DLH_LLD_INT_HIGH     = 0x00u,
  LIS331DLH_LLD_INT_LOW      = 0x80u,
  LIS331DLH_LLD_PUSH_PULL    = 0x00u,
  LIS331DLH_LLD_OPEN_DRAIN   = 0x40u,
  LIS331DLH_LLD_LIR2_LATCH    = 0x20u,
  LIS331DLH_LLD_I2_CFG_I2    = 0x00u,
  LIS331DLH_LLD_I2_CFG_I1_I2 = 0x08u,
  LIS331DLH_LLD_I2_CFG_DRY   = 0x10u,
  LIS331DLH_LLD_I2_CFG_BOOT  = 0x18u,
  LIS331DLH_LLD_LIR1_LATCH    = 0x04u,
  LIS331DLH_LLD_LIR1_NO_LATCH = 0x00u,
  LIS331DLH_LLD_I1_CFG_I1    = 0x00u,
  LIS331DLH_LLD_I1_CFG_I1_I2 = 0x01u,
  LIS331DLH_LLD_I1_CFG_DRY   = 0x02u,
  LIS331DLH_LLD_I1_CFG_BOOT  = 0x03u,
} lis331dlh_lld_ctrl_reg3_t;

/**
 * @brief Control register 4 flags.
 */
typedef enum {
  LIS331DLH_LLD_BDU_CONT = 0x00u,
  LIS331DLH_LLD_BDU_STOP = 0x80u,
  LIS331DLH_LLD_BLE_LE   = 0x00u,
  LIS331DLH_LLD_BLE_BE   = 0x40u,
  LIS331DLH_LLD_FS_2G    = 0x00u,
  LIS331DLH_LLD_FS_4G    = 0x10u,
  LIS331DLH_LLD_FS_8G    = 0x30u,
  LIS331DLH_LLD_STSIGN_POS = 0x00u,
  LIS331DLH_LLD_STSIGN_NEG = 0x08u,
  LIS331DLH_LLD_ST_DISABLE = 0x00u,
  LIS331DLH_LLD_ST_ENABLE  = 0x02u,
  LIS331DLH_LLD_SIM_4WI    = 0x00u,
  LIS331DLH_LLD_SIM_3WI    = 0x01u,
} lis331dlh_lld_ctrl_reg4_t;

/**
 * @brief Control register 5 flags.
 */
typedef enum {
  LIS331DLH_LLD_SLEEP_TO_WAKE_OFF = 0x00u,
  LIS331DLH_LLD_SLEEP_TO_WAKE_ON  = 0x03u,
} lis331dlh_lld_ctrl_reg5_t;

/**
 * @brief Axis enum.
 */
typedef enum {
  LIS331DLH_LLD_X_AXIS = 0,
  LIS331DLH_LLD_Y_AXIS = 1,
  LIS331DLH_LLD_Z_AXIS = 2,
} lis331dlh_lld_axis_t;

/**
 * @brief Interrupt enum.
 */
typedef enum {
  LIS331DLH_LLD_INT1 = 0x01,
  LIS331DLH_LLD_INT2 = 0x02,
} lis331dlh_lld_int_t;

/**
 * @brief Config register struct.
 */
typedef union {
  uint8_t data[5];
  struct {
    uint8_t ctrl_reg1;
    uint8_t ctrl_reg2;
    uint8_t ctrl_reg3;
    uint8_t ctrl_reg4;
    uint8_t ctrl_reg5;
  } registers;
} lis331dlh_lld_cfg_t;

/**
 * @brief Interrupt config register flags.
 */
typedef enum {
  LIS331DLH_LLD_INT_CFG_X_LOW_ENABLE  = 0x01,
  LIS331DLH_LLD_INT_CFG_X_HIGH_ENABLE = 0x02,
  LIS331DLH_LLD_INT_CFG_Y_LOW_ENABLE  = 0x04,
  LIS331DLH_LLD_INT_CFG_Y_HIGH_ENABLE = 0x08,
  LIS331DLH_LLD_INT_CFG_Z_LOW_ENABLE  = 0x10,
  LIS331DLH_LLD_INT_CFG_Z_HIGH_ENABLE = 0x20,
  LIS331DLH_LLD_INT_CFG_D6            = 0x40,
  LIS331DLH_LLD_INT_CFG_AOI           = 0x80,
} lis331dlh_lld_int_cfg_reg_t;

/**
 * @brief Status register flags.
 */
typedef enum {
  LIS331DLH_LLD_STATUS_ZYXOR = 0x01,
  LIS331DLH_LLD_STATUS_Z_OR  = 0x02,
  LIS331DLH_LLD_STATUS_Y_OR  = 0x04,
  LIS331DLH_LLD_STATUS_X_OR  = 0x08,
  LIS331DLH_LLD_STATUS_ZYXDA = 0x10,
  LIS331DLH_LLD_STATUS_Z_DA  = 0x20,
  LIS331DLH_LLD_STATUS_Y_DA  = 0x40,
  LIS331DLH_LLD_STATUS_X_DA  = 0x80,
} lis331dlh_lld_status_t;

/**
 * @brief Interrupt config struct.
 */
typedef struct {
  uint8_t cfg_reg;
  uint8_t threshold;
  uint8_t duration;
} lis331dlh_lld_int_cfg_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t lis331dlh_lld_read_register(const LIS331DLHDriver* const lisd, const lis331dlh_lld_register_t regaddr, uint8_t *data, const uint8_t length);
  apalExitStatus_t lis331dlh_lld_write_register(const LIS331DLHDriver* const lisd, const lis331dlh_lld_register_t regaddr, const uint8_t *data, const uint8_t length);
  apalExitStatus_t lis331dlh_lld_reset_hp_filter(const LIS331DLHDriver* const lisd);
  apalExitStatus_t lis331dlh_lld_read_all_data(const LIS331DLHDriver* const lisd, int16_t *data, const lis331dlh_lld_cfg_t *cfg);
  apalExitStatus_t lis331dlh_lld_read_data(const LIS331DLHDriver* const lisd, int16_t *data, const lis331dlh_lld_axis_t axis, const lis331dlh_lld_cfg_t *cfg);
  apalExitStatus_t lis331dlh_lld_read_config(const LIS331DLHDriver* const lisd, lis331dlh_lld_cfg_t *cfg);
  apalExitStatus_t lis331dlh_lld_write_config(const LIS331DLHDriver* const lisd, const lis331dlh_lld_cfg_t *cfg);
  apalExitStatus_t lis331dlh_lld_read_int_config(const LIS331DLHDriver* const lisd, lis331dlh_lld_int_cfg_t *cfg, const lis331dlh_lld_int_t interrupt);
  apalExitStatus_t lis331dlh_lld_write_int_config(const LIS331DLHDriver* const lisd, const lis331dlh_lld_int_cfg_t *cfg, const lis331dlh_lld_int_t interrupt);
  apalExitStatus_t lis331dlh_lld_read_status_register(const LIS331DLHDriver* const lisd, uint8_t *status);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_LIS331DLH) */

#endif /* _AMIROLLD_LIS331DLH_H_ */
