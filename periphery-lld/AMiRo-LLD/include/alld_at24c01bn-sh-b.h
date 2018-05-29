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

#ifndef _AMIROLLD_AT24C01BN_H_
#define _AMIROLLD_AT24C01BN_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_AT24C01BN) || defined(__DOXYGEN__)

/**
 * @brief Memory size of the EEPROM in bits.
 */
#define AT24C01BN_LLD_SIZE_BITS             1024

/**
 * @brief Memory size of the EEPROM in bytes
 */
#define AT24C01BN_LLD_SIZE_BYTES            128


/**
 * @brief Size of a page in bytes
 */
#define AT24C01BN_LLD_PAGE_SIZE_BYTES       8

/**
 * @brief  Time in microseconds a write operation takes to complete (I2C will not respond).
 * @note   The system should wait slightly longer.
 */
#define AT24C01BN_LLD_WRITECYCLETIME_US     5000

/**
 * @brief Maximum I2C frequency.
 */
#define AT24C01BN_LLD_I2C_MAXFREQUENCY      400000

/**
 * @brief Maximum I2C frequency at 5V.
 */
#define AT24C01BN_LLD_I2C_MAXFREQUENCY_5V   1000000

/**
 * @brief The AT24C01BN driver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;  /**< @brief The I2C Driver */
  uint8_t addr;           /**< @brief The address of the AT24C01B for I2C communication, which is defined by the wiring of the A0, A1, A2 pins */
} AT24C01BNDriver;

/**
 * @brief Bitmasks for the I2C address, including the wiring of the A0, A1, A2 pins.
 */
enum {
  AT24C01BN_LLD_I2C_ADDR_FIXED      = 0x50u,
  AT24C01BN_LLD_I2C_ADDR_A0         = 0x01u,
  AT24C01BN_LLD_I2C_ADDR_A1         = 0x02u,
  AT24C01BN_LLD_I2C_ADDR_A2         = 0x04u,
};

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t at24c01bn_lld_poll_ack(const AT24C01BNDriver* const at24c01bn, const apalTime_t timeout);
  apalExitStatus_t at24c01bn_lld_read_current_address(const AT24C01BNDriver* const at24c01bn, uint8_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t at24c01bn_lld_read(const AT24C01BNDriver* const at24c01bn, const uint8_t addr, uint8_t* const data, const uint8_t num, const apalTime_t timeout);
  apalExitStatus_t at24c01bn_lld_write_byte(const AT24C01BNDriver* const at24c01bn, const uint8_t addr, const uint8_t data, const apalTime_t timeout);
  apalExitStatus_t at24c01bn_lld_write_page(const AT24C01BNDriver* const at24c01bn, const uint8_t addr, const uint8_t* const data, const uint8_t num, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_AT24C01BN) */

#endif /* _AMIROLLD_AT24C01BN_H_ */

