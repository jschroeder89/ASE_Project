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

#ifndef _AMIROLLD_H_
#define _AMIROLLD_H_

/**
 * @brief   AMIRO-LLD identification macro.
 */
#define _AMIRO_LLD_

/**
 * @name   AMiRo-LLD version and relase information.
 * @{
 */

/**
 * @brief   Realease type of this version.
 *          IPossible values are "pre-alpha", "alpha", "beta", "release candidate", "release".
 */
#define AMIRO_LLD_RELEASE_TYPE     "release"

/**
 * @brief   The periphery abstraction layer interface major version.
 * @note    Changes of the major version imply incompatibilities.
 */
#define AMIRO_LLD_VERSION_MAJOR    1

/**
 * @brief   The periphery abstraction layer interface minor version.
 * @note    A higher minor version implies new functionalty, but all old interfaces are still available.
 */
#define AMIRO_LLD_VERSION_MINOR    0

/**
 * @brief   The periphery abstraction layer interface patch level.
 */
#define AMIRO_LLD_VERSION_PATCH    0

/**
 * @brief   The periphery abstraction layer interface required major version.
 */
#define PERIPHAL_REQUIRED_MAJOR    1

/**
 * @brief   The periphery abstraction layer interface required minor version.
 */
#define PERIPHAL_REQUIRED_MINOR    0

/** @} */

#include <alldconf.h>

#if !defined(_AMIRO_LLD_CFG_)
#error "invalid AMiRo-LLD configuration file"
#endif
#if (AMIRO_LLD_CFG_VERSION_MAJOR != AMIRO_LLD_VERSION_MAJOR) || (AMIRO_LLD_CFG_VERSION_MINOR < AMIRO_LLD_VERSION_MINOR)
#error "incompatible AMiRo-LLD configuration file"
#endif

#include <periphAL.h>
#if !defined(PERIPHAL_VERSION_MAJOR) || !defined(PERIPHAL_VERSION_MINOR)
#error "invalid periphAL implementation"
#endif
#if (PERIPHAL_VERSION_MAJOR != PERIPHAL_REQUIRED_MAJOR) || (PERIPHAL_VERSION_MINOR < PERIPHAL_REQUIRED_MINOR)
#error "incompatible periphAL implementation"
#endif

#endif /* _AMIROLLD_H_ */
