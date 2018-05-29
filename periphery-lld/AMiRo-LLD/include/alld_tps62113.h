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

#ifndef _AMIROLLD_TPS62113_H_
#define _AMIROLLD_TPS62113_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_TPS62113) || defined(__DOXYGEN__)

/**
 * @brief   Power pin active state.
 */
#define TPS62113_LLD_POWER_ACTIVE_STATE   APAL_GPIO_ACTIVE_HIGH

/**
 * @brief The TPS62113 driver struct.
 */
typedef struct {
  apalControlGpio_t gpio_power; /**< GPIO to turn power on/off */
} TPS62113Driver;

/**
 * @brief Power enable enum.
 */
typedef enum {
  TPS62113_LLD_POWER_ENABLED  = 0x01,
  TPS62113_LLD_POWER_DISABLED = 0x00,
} tps62113_lld_power_en_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t tps62113_lld_get_power_en(const TPS62113Driver* const tps62113, tps62113_lld_power_en_t* const power);
  apalExitStatus_t tps62113_lld_set_power_en(const TPS62113Driver* const tps62113, const tps62113_lld_power_en_t power);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_TPS62113) */

#endif /* _AMIROLLD_TPS62113_H_ */
