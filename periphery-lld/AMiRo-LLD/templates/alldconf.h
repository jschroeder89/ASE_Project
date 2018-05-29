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

#ifndef _ALLDCONF_H_
#define _ALLDCONF_H_

/*
 * compatibility guards
 */
#define _AMIRO_LLD_CFG_
#define AMIRO_LLD_CFG_VERSION_MAJOR         1
#define AMIRO_LLD_CFG_VERSION_MINOR         0

/*
 * DEACTIVATE DRIVERS BY UNDEFINING (i.e. commenting) THE ACCORDING MACROS.
 */

/**
 * @brief   Enable flag for the A3906 motor driver.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_A3906

/**
 * @brief   Enable flag for the AT24C01BN-SH-B EEPROM.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_AT24C01BN

/**
 * @brief   Enable flag for the BQ24103A battery charger.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_BQ24103A

/**
 * @brief   Enable flag for the BQ27500 fuel gauge.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_BQ27500

/**
 * @brief   Enable flag for the MHC5883L compass.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_HMC5883L

/**
 * @brief   Enable flag for the INA219 power monitor.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_INA219

/**
 * @brief   Enable flag for the L3G4200D gyroscope.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_L3G4200D

/**
 * @brief   Enable flag for the status LED.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_LED

/**
 * @brief   Enable flag for the LIS331DLH accelerometer.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_LIS331DLH

/**
 * @brief   Enable flag for the LTC4412 power path controller.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_LTC4412

/**
 * @brief   Enable flag for the MPR121 touch sensor.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_MPR121

/**
 * @brief   Enable flag for the PCA9544A I2C multiplexer.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_PCA9544A

/**
 * @brief   Enable flag for the PKLCS1212E4001 buzzer.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_PKLCS2121E4001

/**
 * @brief   Enable flag for the TLC5947 LED driver.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_TLC5947

/**
 * @brief   Enable flag for the TPS2051BDBV power switch.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_TPS2051BDBV

/**
 * @brief   Enable flag for the TPS62113 step-down converter.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_TPS62113

/**
 * @brief   Enable flag for the VCNL4020 proximity sensor.
 * @note    Comment to disable.
 */
#define AMIROLLD_CFG_USE_VCNL4020

#endif /* _ALLDCONF_H_ */
