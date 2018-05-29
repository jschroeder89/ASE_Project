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

#ifndef _AMIROLLD_VCNL4020_H_
#define _AMIROLLD_VCNL4020_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_VCNL4020) || defined(__DOXYGEN__)

/**
 * @brief The VCNL4020 driver struct
 */
typedef struct {
  apalI2CDriver_t* i2cd;      /**< @brief The I2C Driver.     */
} VCNL4020Driver;

/**
 * @brief Fixed slave address to access the VCNL4020 via I2C.
 */
#define VCNL4020_LLD_I2C_ADDR           0x13u

/**
 * @brief Maximum frequency for I2C communication.
 */
#define VCNL4020_LLD_I2C_MAXFREQUENCY   3400000

/**
 * @brief A falling edge indicates an interrupt.
 */
#define VCNL4020_LLD_INT_EDGE           APAL_GPIO_EDGE_FALLING

/**
 * @brief Adresses for the several I/O registers of the VCNL4020.
 */
typedef enum {
  VCNL4020_LLD_REGADDR_CMD          = 0x80u, /**< command register */
  VCNL4020_LLD_REGADDR_IDREV        = 0x81u, /**< product ID revision register */
  VCNL4020_LLD_REGADDR_PROXRATE     = 0x82u, /**< proximity rate register */
  VCNL4020_LLD_REGADDR_LEDCURRENT   = 0x83u, /**< IR LED current register */
  VCNL4020_LLD_REGADDR_ALPARAM      = 0x84u, /**< ambient light parameter register */
  VCNL4020_LLD_REGADDR_ALRES_HIGH   = 0x85u, /**< ambient light result register (high byte) */
  VCNL4020_LLD_REGADDR_ALRES_LOW    = 0x86u, /**< ambient light result register (low byte) */
  VCNL4020_LLD_REGADDR_PROXRES_HIGH = 0x87u, /**< proximity result register (high byte) */
  VCNL4020_LLD_REGADDR_PROXRES_LOW  = 0x88u, /**< proximity result register (low byte) */
  VCNL4020_LLD_REGADDR_INTCTRL      = 0x89u, /**< interrupt control register */
  VCNL4020_LLD_REGADDR_LTH_HIGH     = 0x8Au, /**< low threshold register (high byte) */
  VCNL4020_LLD_REGADDR_LTH_LOW      = 0x8Bu, /**< low threshold register (low byte) */
  VCNL4020_LLD_REGADDR_HTH_HIGH     = 0x8Cu, /**< high threshold register (high byte) */
  VCNL4020_LLD_REGADDR_HTH_LOW      = 0x8Du, /**< high threshold register (low byte) */
  VCNL4020_LLD_REGADDR_INTSTATUS    = 0x8Eu, /**< interrupt status register */
  VCNL4020_LLD_REGADDR_PMTADJ       = 0x8Fu, /**< prximity modulator timing adjustment */
  VCNL4020_LLD_REGADDR_AMBIRLEVEL   = 0x90u, /**< ambient IR light level register (not intended to be used) */
} vcnl4020_lld_regaddr_t;

/**
 * @brief Commands to control the VCNL4020.
 */
typedef enum {
  VCNL4020_LLD_CMDREG_CFGLOCK   = 0x80u, /**< config lock (read only, constant = 1) */
  VCNL4020_LLD_CMDREG_ALSRDY    = 0x40u, /**< ambient light measurement data ready flag (read only) */
  VCNL4020_LLD_CMDREG_PROXRDY   = 0x20u, /**< proximity measurement data ready flag (read only) */
  VCNL4020_LLD_CMDREG_ALSOD     = 0x10u, /**< start single on-demand ambient light measurement */
  VCNL4020_LLD_CMDREG_PROXOD    = 0x08u, /**< start single on-demand proximity measurement */
  VCNL4020_LLD_CMDREG_ALSEN     = 0x04u, /**< start periodic ambient light measurement */
  VCNL4020_LLD_CMDREG_PROXEN    = 0x02u, /**< start periodic proximity measurement */
  VCNL4020_LLD_CMDREG_SELFTIMED = 0x01u, /**< enable state-machine and clock for periodic measurement */
} vcnl4020_lld_cmdreg_t;

/**
 * @brief Masks to read the prodict ID revision register.
 */
typedef enum {
  VCNL4020_LLD_IDREVREG_ID_MASK   = 0xF0u, /**< product ID (read only, constant = 2) */
  VCNL4020_LLD_IDREVREG_REV_MASK  = 0x0Fu, /**< revision ID (read only, constant = 1) */
} vcnl4020_lld_idrevreg_t;

/**
 * @brief Frequencies for the proximity sensor of the VCNL4020.
 * @note  Frequencies are calculated by the formula
 *        freq = 250Hz / (2 ^ (7 - <value>))
 */
typedef enum {
  VCNL4020_LLD_PROXRATEREG_MASK       = 0x07u, /**< mask of settable bits */
  VCNL4020_LLD_PROXRATEREG_1_95_HZ    = 0x00u, /**< 1.95Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_3_90625_HZ = 0x01u, /**< 3.90625Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_7_8125_HZ  = 0x02u, /**< 7.8125Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_15_625_HZ  = 0x03u, /**< 16.625Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_31_25_HZ   = 0x04u, /**< 31.25Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_62_5_HZ    = 0x05u, /**< 62.5Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_125_HZ     = 0x06u, /**< 125Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_250_HZ     = 0x07u, /**< 250Hz sampling rate */
  VCNL4020_LLD_PROXRATEREG_DEFAULT    = 0x00u, /**< default sampling rate (1.95Hz) */
} vcnl4020_lld_proxratereg_t;

/**
 * @brief Current for the emmitting IR LED.
 * @note  The Current is calculated by the formula
 *        curr = <value> * 10mA
 */
typedef enum {
  VCNL4020_LLD_LEDCURRENTREG_FUSEID_MASK  = 0xC0u, /**< fuse program revision (read only) */
  VCNL4020_LLD_LEDCURRENTREG_CURRENT_MASK = 0x3Fu, /**< mask of bits to set the current */
  VCNL4020_LLD_LEDCURRENTREG_0_mA         = 0x00u, /**< 0mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_10_mA        = 0x01u, /**< 10mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_20_mA        = 0x02u, /**< 20mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_30_mA        = 0x03u, /**< 30mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_40_mA        = 0x04u, /**< 40mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_50_mA        = 0x05u, /**< 50mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_60_mA        = 0x06u, /**< 60mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_70_mA        = 0x07u, /**< 70mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_80_mA        = 0x08u, /**< 80mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_90_mA        = 0x09u, /**< 90mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_100_mA       = 0x0Au, /**< 100mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_110_mA       = 0x0Bu, /**< 110mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_120_mA       = 0x0Cu, /**< 120mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_130_mA       = 0x0Du, /**< 130mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_140_mA       = 0x0Eu, /**< 140mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_150_mA       = 0x0Fu, /**< 150mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_160_mA       = 0x10u, /**< 160mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_170_mA       = 0x11u, /**< 170mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_180_mA       = 0x12u, /**< 180mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_190_mA       = 0x13u, /**< 190mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_200_mA       = 0x14u, /**< 200mA IR LED current */
  VCNL4020_LLD_LEDCURRENTREG_DEFAULT      = 0x02u, /**< default IR LED current (20mA) */
} vcnl4020_lld_ledcurrentreg_t;

/**
 * @brief Ambient light measurement parameters.
 */
typedef enum {
  VCNL4020_LLD_ALPARAMREG_CONTCONV            = 0x80u, /**< Continuous conversion mode flag (set = enabled) */
  VCNL4020_LLD_ALPARAMREG_CONTCONV_DEFAULT    = 0x00u, /**< Continuous conversion mode flag default value (disabled)*/
  VCNL4020_LLD_ALPARAMREG_RATE_MASK           = 0x70u, /**< mask of bits to set the measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_1_HZ           = 0x00u, /**< 1Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_2_HZ           = 0x10u, /**< 2Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_3_HZ           = 0x20u, /**< 3Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_4_HZ           = 0x30u, /**< 4Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_5_HZ           = 0x40u, /**< 5Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_6_HZ           = 0x50u, /**< 6Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_8_HZ           = 0x60u, /**< 8Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_10_HZ          = 0x70u, /**< 10Hz measurement rate */
  VCNL4020_LLD_ALPARAMREG_RATE_DEFAULT        = 0x10u, /**< default measurement rate (2Hz) */
  VCNL4020_LLD_ALPARAMREG_AUTOOFFSET          = 0x08u, /**< Auto offset compensation flag (set = enabled) */
  VCNL4020_LLD_ALPARAMREG_AUTOOFFSET_DEFAULT  = 0x00u, /**< Auto offset compensation flag default value (disabled) */
  VCNL4020_LLD_ALPARAMREG_AVG_MASK            = 0x07u, /**< mask of bits to set the number of measurements for averaging */
  VCNL4020_LLD_ALPARAMREG_AVG_1_CONV          = 0x00u, /**< 1 conversion per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_2_CONV          = 0x01u, /**< 2 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_4_CONV          = 0x02u, /**< 4 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_8_CONV          = 0x03u, /**< 8 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_16_CONV         = 0x04u, /**< 16 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_32_CONV         = 0x05u, /**< 32 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_64_CONV         = 0x06u, /**< 64 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_128_CONV        = 0x07u, /**< 128 conversions per measurement */
  VCNL4020_LLD_ALPARAMREG_AVG_DEFAULT         = 0x05u, /**< default number of conversions per measurement (32) */
} vcnl4020_lld_alparamreg_t;

/**
 * @brief Interrupt control commands.
 */
typedef enum {
  VCNL4020_LLD_INTCTRLREG_CNT_MASK    = 0xE0u, /**< mask of bits to set the count value */
  VCNL4020_LLD_INTCTRLREG_CNT_1       = 0x00u, /**< 1 measurement above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_2       = 0x20u, /**< 2 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_4       = 0x40u, /**< 4 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_8       = 0x60u, /**< 8 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_16      = 0x80u, /**< 16 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_32      = 0xA0u, /**< 32 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_64      = 0xC0u, /**< 64 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_CNT_128     = 0xE0u, /**< 128 measurements above/below the threshold will cause an interrupt */
  VCNL4020_LLD_INTCTRLREG_PROXRDY_EN  = 0x08u, /**< enable interrupts at proximity data ready */
  VCNL4020_LLD_INTCTRLREG_ALSRDY_EN   = 0x04u, /**< enable interrupt at ambient data ready */
  VCNL4020_LLD_INTCTRLREG_THRES_EN    = 0x02u, /**< enable interrupt when above/below threshold */
  VCNL4020_LLD_INTCTRLREG_THRES_SEL   = 0x01u, /**< select threshold intterrupt source (0 = proximity, 1 = ambient) */
} vcnl4020_lld_intctrlreg_t;

/**
 * @brief Interrupt status flags.
 */
typedef enum {
  VCNL4020_LLD_INTSTATUSREG_PROXRDY = 0x08u, /**< proximity data ready */
  VCNL4020_LLD_INTSTATUSREG_ALSRDY  = 0x04u, /**< ambient light measurement data ready */
  VCNL4020_LLD_INTSTATUSREG_THLOW   = 0x02u, /**< low threshold exceeded */
  VCNL4020_LLD_INTSTATUSREG_THHIGH  = 0x01u, /**< high threshold exceeded */
} vcnl4020_lld_intstatusreg_t;

/**
 * @brief Proximuty modulation timing adjustment parameters.
 */
typedef enum {
  VCNL4020_LLD_PMTADJREG_DELAY_MASK           = 0xE0u, /**< mask of bits to set the modulation delay time */
  VCNL4020_LLD_PMTADJREG_DELAY_DEFAULT        = 0x00u, /**< default modulation delay time */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_MASK        = 0x18u, /**< mask of bits to set the signal requency */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_390_625_KHz = 0x00u, /**< 390.625KHz signal frequency */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_781_25_KHz  = 0x08u, /**< 781.25KHz signal frequency */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_1562_5_KHz  = 0x10u, /**< 1.5625MHz signal frequency */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_3125_KHz    = 0x18u, /**< 3.125MHz signal frequency */
  VCNL4020_LLD_PMTADJREG_PROXFREQ_DEFAULT     = 0x00u, /**< default signal frequency (390.625KHz) */
  VCNL4020_LLD_PMTADJREG_DEAD_MASK            = 0x07u, /**< mask of bits to set the modulation dead time */
  VCNL4020_LLD_PMTADJREG_DEAD_DEFAULT         = 0x01u, /**< default modulation dead time */
} vcnl4020_lld_pmtadjreg_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t vcnl4020_lld_readreg(const VCNL4020Driver* const vcnld, const vcnl4020_lld_regaddr_t reg, uint8_t* const data, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_writereg(const VCNL4020Driver* const vcnld, const vcnl4020_lld_regaddr_t reg, const uint8_t data, const apalTime_t timeout);

  apalExitStatus_t vcnl4020_lld_readals(const VCNL4020Driver* const vcnld, uint16_t* const als, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_readprox(const VCNL4020Driver* const vcnld, uint16_t* const prox, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_readalsandprox(const VCNL4020Driver* const vcnld, uint16_t* const als, uint16_t* const prox, const apalTime_t timeout);

  apalExitStatus_t vcnl4020_lld_readlth(const VCNL4020Driver* const vcnld, uint16_t* const th, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_readhth(const VCNL4020Driver* const vcnld, uint16_t* const th, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_readth(const VCNL4020Driver* const vcnld, uint16_t* const lth, uint16_t* const hth, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_writelth(const VCNL4020Driver* const vcnld, const uint16_t th, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_writehth(const VCNL4020Driver* const vcnld, const uint16_t th, const apalTime_t timeout);
  apalExitStatus_t vcnl4020_lld_writeth(const VCNL4020Driver* const vcnld, const uint16_t lth, const uint16_t hth, const apalTime_t timeout);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_VCNL4020) */

#endif /* _AMIROLLD_VCNL4020_H_ */

