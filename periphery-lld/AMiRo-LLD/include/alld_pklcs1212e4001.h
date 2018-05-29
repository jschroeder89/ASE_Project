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

#ifndef _AMIROLLD_PKLCS1212E4001_H_
#define _AMIROLLD_PKLCS1212E4001_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_PKLCS1212E4001) || defined(__DOXYGEN__)

/**
 * @brief   Specified frequency of the PKLCS1212E4001.
 */
#define PKLCS1212E4001_LLD_FREQUENCY_SPEC       4000

/**
 * @brief   Minimum frequency for the buzzer to sound.
 */
#define PKLCS1212E4001_LLD_FREQUENCY_MIN        2000

/**
 * @brief   Maximum frequency for the buzzer to sound.
 */
#define PKLCS1212E4001_LLD_FREQUENCY_MAX        6000

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t pklcs1212e4001_lld_checkPWMconfiguration(apalPWMDriver_t* pwm);
  apalExitStatus_t pklcs1212e4001_lld_enable(apalPWMDriver_t* pwm, const apalPWMchannel_t channel, const bool enable);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_PKLCS1212E4001) */

#endif /* _AMIROLLD_PKLCS1212E4001_H_ */
