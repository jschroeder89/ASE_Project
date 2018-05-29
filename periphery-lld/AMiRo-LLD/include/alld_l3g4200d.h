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

#ifndef _AMIROLLD_L3G4200D_H_
#define _AMIROLLD_L3G4200D_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_L3G4200D) || defined(__DOXYGEN__)

/**
 * @brief A rising edge indicates an interrupt.
 */
#define L3G4200D_LLD_INT_EDGE   APAL_GPIO_EDGE_RISING

/**
 * @brief The L3G4200D driver struct
 */
typedef struct {
  apalSPIDriver_t* spid;        /**< @brief The SPI Driver */
} L3G4200DDriver;

/**
 * @brief SPI access modes.
 */
typedef enum {
  L3G4200D_LLD_SPI_MULT  = 0x40u,
  L3G4200D_LLD_SPI_READ  = 0x80u,
  L3G4200D_LLD_SPI_WRITE = 0x00u,
} l3g4200d_lld_SPI_mode_t;

/**
 * @brief Registers.
 */
typedef enum {
  L3G4200D_LLD_REGISTER_WHO_AM_I = 0x0F,
  L3G4200D_LLD_REGISTER_CTRL_REG1 = 0x20,
  L3G4200D_LLD_REGISTER_CTRL_REG2 = 0x21,
  L3G4200D_LLD_REGISTER_CTRL_REG3 = 0x22,
  L3G4200D_LLD_REGISTER_CTRL_REG4 = 0x23,
  L3G4200D_LLD_REGISTER_CTRL_REG5 = 0x24,
  L3G4200D_LLD_REGISTER_REFERECE = 0x25,
  L3G4200D_LLD_REGISTER_OUT_TEMP = 0x26,
  L3G4200D_LLD_REGISTER_STATUS_REG = 0x27,
  L3G4200D_LLD_REGISTER_OUT_X_L = 0x28,
  L3G4200D_LLD_REGISTER_OUT_X_H = 0x29,
  L3G4200D_LLD_REGISTER_OUT_Y_L = 0x2A,
  L3G4200D_LLD_REGISTER_OUT_Y_H = 0x2B,
  L3G4200D_LLD_REGISTER_OUT_Z_L = 0x2C,
  L3G4200D_LLD_REGISTER_OUT_Z_H = 0x2D,
  L3G4200D_LLD_REGISTER_FIFO_CTRL_REG = 0x2E,
  L3G4200D_LLD_REGISTER_FIFO_SRC_REG = 0x2F,
  L3G4200D_LLD_REGISTER_INT1_CFG = 0x30,
  L3G4200D_LLD_REGISTER_INT1_SRC = 0x31,
  L3G4200D_LLD_REGISTER_INT1_TSH_XH = 0x32,
  L3G4200D_LLD_REGISTER_INT1_TSH_XL = 0x33,
  L3G4200D_LLD_REGISTER_INT1_TSH_YH = 0x34,
  L3G4200D_LLD_REGISTER_INT1_TSH_YL = 0x35,
  L3G4200D_LLD_REGISTER_INT1_TSH_ZH = 0x36,
  L3G4200D_LLD_REGISTER_INT1_TSH_ZL = 0x37,
  L3G4200D_LLD_REGISTER_INT1_DURATION = 0x38,
} l3g4200d_lld_register_t;

/**
 * @brief WHO_AM_I register constant content.
 */
typedef enum {
  L3G4200D_LLD_WHO_AM_I = 0xD3,
} l3g4200d_lld_whoami_t;

/**
 * @brief Control register 1 flags.
 */
typedef enum {
  L3G4200D_LLD_DR_100_HZ   = 0x00,
  L3G4200D_LLD_DR_200_HZ   = 0x40,
  L3G4200D_LLD_DR_400_HZ   = 0x80,
  L3G4200D_LLD_DR_800_HZ   = 0xC0,
  L3G4200D_LLD_BW_12_5  = 0x00,
  L3G4200D_LLD_BW_20    = 0x00,
  L3G4200D_LLD_BW_25    = 0x10,
  L3G4200D_LLD_BW_30    = 0x00,
  L3G4200D_LLD_BW_35    = 0x10,
  L3G4200D_LLD_BW_50    = 0x20,
  L3G4200D_LLD_BW_70    = 0x30,
  L3G4200D_LLD_BW_110   = 0x30,
  L3G4200D_LLD_PD  = 0x08,
  L3G4200D_LLD_ZEN = 0x04,
  L3G4200D_LLD_YEN = 0x02,
  L3G4200D_LLD_XEN = 0x01,
} l3g4200d_lld_ctrl_reg1_t;

/**
 * @brief Control register 2 flags.
 */
typedef enum {
  L3G4200D_LLD_HPM_NORMAL_RST = 0x00,
  L3G4200D_LLD_HPM_REFERENCE  = 0x10,
  L3G4200D_LLD_HPM_NORMAL     = 0x20,
  L3G4200D_LLD_HPM_AUTO_RST   = 0x30,
  L3G4200D_LLD_HPCF_2    = 0x00,
  L3G4200D_LLD_HPCF_4    = 0x01,
  L3G4200D_LLD_HPCF_8    = 0x02,
  L3G4200D_LLD_HPCF_16   = 0x03,
  L3G4200D_LLD_HPCF_32   = 0x04,
  L3G4200D_LLD_HPCF_64   = 0x05,
  L3G4200D_LLD_HPCF_128  = 0x06,
  L3G4200D_LLD_HPCF_256  = 0x07,
  L3G4200D_LLD_HPCF_512  = 0x08,
  L3G4200D_LLD_HPCF_1024 = 0x09,
} l3g4200d_lld_ctrl_reg2_t;

/**
 * @brief Control register 3 flags.
 */
typedef enum {
  L3G4200D_LLD_I1_INT1   = 0x80,
  L3G4200D_LLD_I1_BOOT   = 0x40,
  L3G4200D_LLD_H_IACTIVE = 0x20,
  L3G4200D_LLD_PP_OD     = 0x10,
  L3G4200D_LLD_I2_DRDY   = 0x08,
  L3G4200D_LLD_I2_WTM    = 0x04,
  L3G4200D_LLD_I2_ORUN   = 0x02,
  L3G4200D_LLD_I2_EMPTY  = 0x01,
} l3g4200d_lld_ctrl_reg3_t;

/**
 * @brief Control register 4 flags.
 */
typedef enum {
  L3G4200D_LLD_BDU_CONT    = 0x00,
  L3G4200D_LLD_BDU_SINGLE  = 0x80,
  L3G4200D_LLD_BLE_MSB     = 0x40,
  L3G4200D_LLD_BLE_LSB     = 0x00,
  L3G4200D_LLD_FS_250_DPS  = 0x00,
  L3G4200D_LLD_FS_500_DPS  = 0x10,
  L3G4200D_LLD_FS_2000_DPS = 0x20,
  L3G4200D_LLD_ST_SIGN_M   = 0x04,
  L3G4200D_LLD_ST_SIGN_P   = 0x00,
  L3G4200D_LLD_ST_EN       = 0x02,
  L3G4200D_LLD_SIM_3W      = 0x01,
  L3G4200D_LLD_SIM_4W      = 0x00,
} l3g4200d_lld_ctrl_reg4_t;

/**
 * @brief Control register 5 flags.
 */
typedef enum {
  L3G4200D_LLD_BOOT          = 0x80,
  L3G4200D_LLD_FIFO_EN       = 0x40,
  L3G4200D_LLD_HP_EN         = 0x10,
  L3G4200D_LLD_INT1_SEL_NOHP = 0x00,
  L3G4200D_LLD_INT1_SEL_HP   = 0x04,
  L3G4200D_LLD_INT1_SEL_LP   = 0x08,
  L3G4200D_LLD_OUT_SEL_NOHP  = 0x00,
  L3G4200D_LLD_OUT_SEL_HP    = 0x01,
  L3G4200D_LLD_OUT_SEL_LP    = 0x02,
} l3g4200d_lld_ctrl_reg5_t;

/**
 * @brief Status register flags.
 */
typedef enum {
  L3G4200D_LLD_ZYXOR = 0x80,
  L3G4200D_LLD_ZOR   = 0x40,
  L3G4200D_LLD_YOR   = 0x20,
  L3G4200D_LLD_XOR   = 0x10,
  L3G4200D_LLD_ZYXDA = 0x08,
  L3G4200D_LLD_ZDA   = 0x04,
  L3G4200D_LLD_YDA   = 0x02,
  L3G4200D_LLD_XDA   = 0x01,
} l3g4200d_lld_status_reg_t;

/**
 * @brief Fifo control register masks.
 */
typedef enum {
  L3G4200D_LLD_FM_BYPASS        = 0x00,
  L3G4200D_LLD_FM_FMMODE        = 0x20,
  L3G4200D_LLD_FM_STREAM        = 0x40,
  L3G4200D_LLD_FM_STREAM2FIFO   = 0x60,
  L3G4200D_LLD_FM_BYPASS2STREAM = 0x80,
  L3G4200D_LLD_WTM_MASK         = 0x1F,
} l3g4200d_lld_fifo_ctrl_reg_t;

/**
 * @brief  FIFO source register masks.
 */
typedef enum {
  L3G4200D_LLD_WTM      = 0x80,
  L3G4200D_LLD_OVRN     = 0x40,
  L3G4200D_LLD_EMPTY    = 0x20,
  L3G4200D_LLD_FSS_MASK = 0x1F,
} l3g4200d_lld_fifo_src_reg_t;

/**
 * @brief Interrupt 1 config register flags.
 */
typedef enum {
  L3G4200D_LLD_ANDOR = 0x80,
  L3G4200D_LLD_LIR   = 0x40,
  L3G4200D_LLD_ZHIE  = 0x20,
  L3G4200D_LLD_ZLIE  = 0x10,
  L3G4200D_LLD_YHIE  = 0x08,
  L3G4200D_LLD_YLIE  = 0x04,
  L3G4200D_LLD_XHIE  = 0x02,
  L3G4200D_LLD_XLIE  = 0x01,
} l3g4200d_lld_int1_cfg_reg_t;

/**
 * @brief Interrupt 1 source register flags.
 */
typedef enum {
  L3G4200D_LLD_IA = 0x40,
  L3G4200D_LLD_ZH = 0x20,
  L3G4200D_LLD_ZL = 0x10,
  L3G4200D_LLD_YH = 0x08,
  L3G4200D_LLD_YL = 0x04,
  L3G4200D_LLD_XH = 0x02,
  L3G4200D_LLD_XL = 0x01,
} l3g4200d_lld_int1_src_t;

/**
 * @brief Threshold mask.
 */
enum {
  L3G4200D_LLD_THS_L_MASK = 0x7F
};

/**
 * @brief Interrupt duration register masks.
 */
typedef enum {
  L3G4200D_LLD_INT1_WAIT          = 0x80,
  L3G4200D_LLD_INT1_DURATION_MASK = 0x7F,
} l3g4200d_lld_int1_duration_reg_t;

/**
 * @brief Axis enum.
 */
typedef enum {
  L3G4200D_LLD_X_AXIS = 0x00,
  L3G4200D_LLD_Y_AXIS = 0x01,
  L3G4200D_LLD_Z_AXIS = 0x02,
} l3g4200d_lld_axis_t;

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
} l3g4200d_lld_cfg_t;

/**
 * @brief Interrupt config struct.
 */
typedef union {
  uint8_t data[9];
  struct {
    uint8_t int1_cfg;
    uint8_t int1_src;
    uint8_t int1_tsh_xh;
    uint8_t int1_tsh_xl;
    uint8_t int1_tsh_yh;
    uint8_t int1_tsh_yl;
    uint8_t int1_tsh_zh;
    uint8_t int1_tsh_zl;
    uint8_t int1_duration;
  } registers;
} l3g4200d_lld_int_cfg_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t l3g4200d_lld_read_register(const L3G4200DDriver* const l3gd, const l3g4200d_lld_register_t regaddr, uint8_t* const data, const uint8_t length);
  apalExitStatus_t l3g4200d_lld_write_register(const L3G4200DDriver* const l3gd, const l3g4200d_lld_register_t regaddr, const uint8_t* const data, const uint8_t length);
  apalExitStatus_t l3g4200d_lld_read_all_data(const L3G4200DDriver* const l3gd, int16_t* const data, const l3g4200d_lld_cfg_t* const cfg);
  apalExitStatus_t l3g4200d_lld_read_data(const L3G4200DDriver* const l3gd, int16_t* const data, const l3g4200d_lld_axis_t axis, const l3g4200d_lld_cfg_t* const cfg);
  apalExitStatus_t l3g4200d_lld_read_config(const L3G4200DDriver* const l3gd, l3g4200d_lld_cfg_t* const cfg);
  apalExitStatus_t l3g4200d_lld_write_config(const L3G4200DDriver* const l3gd, const l3g4200d_lld_cfg_t cfg);
  apalExitStatus_t l3g4200d_lld_read_int_config(const L3G4200DDriver* const l3gd, l3g4200d_lld_int_cfg_t* const cfg);
  apalExitStatus_t l3g4200d_lld_write_int_config(const L3G4200DDriver* const l3gd, const l3g4200d_lld_int_cfg_t cfg);
  apalExitStatus_t l3g4200d_lld_read_int_src(const L3G4200DDriver* const l3gd, uint8_t* const cfg);
  apalExitStatus_t l3g4200d_lld_read_status_register(const L3G4200DDriver* const l3gd, uint8_t* const status);
  apalExitStatus_t l3g4200d_lld_read_fifo_ctrl_register(const L3G4200DDriver* const l3gd, uint8_t* const fifo);
  apalExitStatus_t l3g4200d_lld_write_fifo_ctrl_register(const L3G4200DDriver* const l3gd, const uint8_t fifo);
  apalExitStatus_t l3g4200d_lld_read_fifo_src_register(const L3G4200DDriver* const l3gdd, uint8_t* const fifo);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_L3G4200D) */

#endif /* _AMIROLLD_L3G4200D_H_ */
