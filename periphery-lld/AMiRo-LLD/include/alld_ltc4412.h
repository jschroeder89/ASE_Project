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

#ifndef _AMIROLLD_LTC4412_H_
#define _AMIROLLD_LTC4412_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_LTC4412) || defined(__DOXYGEN__)

/**
 * @brief CTRL GPIO active state.
 */
#define LTC4412_LLD_CTRL_ACTIVE_STATE   APAL_GPIO_ACTIVE_HIGH

/**
 * @brief STAT GPIO active state.
 */
#define LTC4412_LLD_STAT_ACTIVE_STATE   APAL_GPIO_ACTIVE_HIGH

/**
 * @brief LTC4412 driver struct.
 */
typedef struct {
  apalControlGpio_t gpio_ctrl; /**< @brief Control GPIO */
  apalControlGpio_t gpio_stat; /**< @brief Status GPIO */
} LTC4412Driver;

typedef enum {
  LTC4412_LLD_CTRL_INACTIVE = 0x0,
  LTC4412_LLD_CTRL_ACTIVE   = 0x1,
} ltc4412_lld_ctrl_t;

typedef enum {
  LTC4412_LLD_STAT_INACTIVE = 0x0,
  LTC4412_LLD_STAT_ACTIVE   = 0x1,
} ltc4412_lld_stat_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t ltc4412_lld_get_ctrl(const LTC4412Driver* const ltc4412, ltc4412_lld_ctrl_t* const ctrl);
  apalExitStatus_t ltc4412_lld_set_ctrl(const LTC4412Driver* const ltc4412, const ltc4412_lld_ctrl_t ctrl);
  apalExitStatus_t ltc4412_lld_get_stat(const LTC4412Driver* const ltc4412, ltc4412_lld_stat_t* const stat);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_LTC4412) */

#endif /* _AMIROLLD_LTC4412_H_ */
