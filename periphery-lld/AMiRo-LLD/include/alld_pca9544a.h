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

#ifndef _AMIROLLD_PCA9544A_H_
#define _AMIROLLD_PCA9544A_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_PCA9544A) || defined(__DOXYGEN__)

/**
 * @brief Maximum I2C frequency.
 */
#define PCA9544A_LLD_I2C_MAXFREQUENCY   400000

/**
 * @brief The PCA9544ADriver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;  /**< @brief The I²C driver to use. */
  uint8_t addr;           /**< @brief The address of the PCA9544A for I2C communication, which is defined by the wiring of the A0, A1, A2 pins */
} PCA9544ADriver;

/**
 * @brief The fixed address prefix of any PCA9544A.
 */
enum {
  PCA9544A_LLD_I2C_ADDR_FIXED   = 0x70u,
  PCA9544A_LLD_I2C_ADDR_A0      = 0x01u,
  PCA9544A_LLD_I2C_ADDR_A1      = 0x02u,
  PCA9544A_LLD_I2C_ADDR_A2      = 0x04u,
};

/**
 * @brief The control register of the PCA9544A.
 */
typedef enum {
  PCA9544A_LLD_CTRLREG_INT3     = 0x80u, /**< interrupt 3 flag (1 = active) */
  PCA9544A_LLD_CTRLREG_INT2     = 0x40u, /**< interrupt 2 flag (1 = active) */
  PCA9544A_LLD_CTRLREG_INT1     = 0x20u, /**< interrupt 1 flag (1 = active) */
  PCA9544A_LLD_CTRLREG_INT0     = 0x10u, /**< interrupt 0 flag (1 = active) */
  PCA9544A_LLD_CTRLREG_EN       = 0x04u, /**< multiplexing enable flag (1 = enabled) */
  PCA9544A_LLD_CTRLREG_CH_MASK  = 0x03u, /**< bitmask for all channel selection bits */
  PCA9544A_LLD_CTRLREG_CH0      = 0x00u, /**< channel 0 selection mask */
  PCA9544A_LLD_CTRLREG_CH1      = 0x01u, /**< channel 1 selection mask */
  PCA9544A_LLD_CTRLREG_CH2      = 0x02u, /**< channel 2 selection mask */
  PCA9544A_LLD_CTRLREG_CH3      = 0x03u, /**< channel 3 selection mask */
  PCA9544A_LLD_CTRLREG_DEFAULT  = 0x00u, /**< default register value after power-up */
} pca9544a_lld_ctrlreg_t;

/**
 * @brief Interrupt identifiers for external use.
 */
typedef enum {
  PCA9544A_LLD_INT0 = 0x01u, /**< interrupt 0 identifier */
  PCA9544A_LLD_INT1 = 0x02u, /**< interrupt 1 identifier */
  PCA9544A_LLD_INT2 = 0x04u, /**< interrupt 2 identifier */
  PCA9544A_LLD_INT3 = 0x08u, /**< interrupt 3 identifier */
} pca9544a_lld_intid_t;

/**
 * @brief Channel selection identifier for external use.
 */
typedef enum {
  PCA9544A_LLD_CH0      = 0x00u, /**< identifier to select channel 0 */
  PCA9544A_LLD_CH1      = 0x01u, /**< identifier to select channel 1*/
  PCA9544A_LLD_CH2      = 0x02u, /**< identifier to select channel 2 */
  PCA9544A_LLD_CH3      = 0x03u, /**< identifier to select channel 3 */
  PCA9544A_LLD_CH_NONE  = 0xFFu, /**< identifier to select no channel */
} pca9544a_lld_chid_t;

typedef uint8_t pca9544a_lld_intstatus_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t pca9544a_lld_read(const PCA9544ADriver* const pca9544a, uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t pca9544a_lld_write(const PCA9544ADriver* const pca9544a, const uint8_t data, const apalTime_t timeout);
  apalExitStatus_t pca9544a_lld_getintstatus(const PCA9544ADriver* const pca9544a, pca9544a_lld_intstatus_t* const  status, const apalTime_t timeout);
  apalExitStatus_t pca9544a_lld_getcurrentchannel(const PCA9544ADriver* const pca9544a, pca9544a_lld_chid_t* const channel, const apalTime_t timeout);
  apalExitStatus_t pca9544a_lld_setchannel(const PCA9544ADriver* const pca9544a, const pca9544a_lld_chid_t channel, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_PCA9544A) */

#endif /* _AMIROLLD_PCA9544A_H_ */
