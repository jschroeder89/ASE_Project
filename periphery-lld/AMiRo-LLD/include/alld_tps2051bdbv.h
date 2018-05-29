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

#ifndef _AMIROLLD_TPS2051B_H_
#define _AMIROLLD_TPS2051B_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_TPS2051BDBV) || defined(__DOXYGEN__)

/**
 * @brief   Enable pin active state.
 */
#define TPS2051B_LLD_ENABLE_ACTIVE_STATE        APAL_GPIO_ACTIVE_HIGH

/**
 * @brief   Overcurrent pin active state.
 */
#define TPS2051B_LLD_OVERCURRENT_ACTIVE_STATE   APAL_GPIO_ACTIVE_LOW

/**
 * @brief The TPS2051B driver struct.
 */
typedef struct {
  apalControlGpio_t enable;      /**< @brief GPIO to enable/disable the tps2051b. */
  apalControlGpio_t overcurrent; /**< @brief GPIO to indicate overcurrent */
} TPS2051BDriver;

/**
 * @brief Enable/Disable enum.
 */
typedef enum {
  TPS2051B_LLD_ENABLE = 0x01,
  TPS2051B_LLD_DISABLE = 0x00,
} tps2051b_lld_enable_t;

/**
 * @brief Overcurrent enum.
 */
typedef enum {
  TPS2051B_LLD_OVERCURRENT = 0x01,
  TPS2051B_LLD_NO_OVERCURRENT = 0x00,
} tps2051b_lld_overcurrent_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t tps2051b_lld_set_enable(const TPS2051BDriver* const tps2051b, const tps2051b_lld_enable_t enable);
  apalExitStatus_t tps2051b_lld_read_enable(const TPS2051BDriver* const tps2051b, tps2051b_lld_enable_t* const enable);
  apalExitStatus_t tps2051b_lld_read_overcurrent(const TPS2051BDriver* const tps2051b, tps2051b_lld_overcurrent_t* const oc);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_TPS2051BDBV) */

#endif /* _AMIROLLD_TPS2051B_H_ */
