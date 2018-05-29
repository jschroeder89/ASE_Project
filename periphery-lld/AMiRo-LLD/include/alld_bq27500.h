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

#ifndef _AMIROLLD_BQ27500_H_
#define _AMIROLLD_BQ27500_H_

#include <amiro-lld.h>

#if defined(AMIROLLD_CFG_USE_BQ27500) || defined(__DOXYGEN__)

/**
 * @brief fixed I2C address
 */
#define BQ27500_LLD_I2C_ADDR              0x55u

/**
 * @brief maximum I2C frequency in Hz
 */
#define BQ27500_LLD_I2C_MAXFREQUENCY      100000

/**
 * @brief default key0 to get into unsealed mode
 */
#define BQ27500_LLD_DEFAULT_UNSEAL_KEY0   0x3672u

/**
 * @brief  default key1 to get into unsealed mode
 */
#define BQ27500_LLD_DEFAULT_UNSEAL_KEY1   0x0414u

/**
 * @brief Active state of battery low gpio.
 */
#define BQ27500_LLD_BATLOW_ACTIVE_STATE   APAL_GPIO_ACTIVE_HIGH

/**
 * @brief Active state of battery good gpio.
 */
#define BQ27500_LLD_BATGOOD_ACTIVE_STATE  APAL_GPIO_ACTIVE_LOW

/**
 * @brief Possible standard commands.
 */
typedef enum {
                                                                                              //   unit   |      access
                                                                                              //          | sealed  | unsealed
                                                                                              // ---------+---------+----------
  BQ27500_LLD_STD_CMD_Control = 0x00u, BQ27500_LLD_STD_CMD_CNTL = 0x00u,                      // N/A      | R/W     | R/W
  BQ27500_LLD_STD_CMD_AtRate = 0x02u, BQ27500_LLD_STD_CMD_AR = 0x02u,                         // mA       | R/W     | R/W
  BQ27500_LLD_STD_CMD_AtRateTimeToEmpty = 0x04u, BQ27500_LLD_STD_CMD_ARTTE = 0x04u,           // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_Temperatur = 0x06u, BQ27500_LLD_STD_CMD_TEMP = 0x06u,                   // 0.1 K    | R       | R/W
  BQ27500_LLD_STD_CMD_Voltage = 0x08u, BQ27500_LLD_STD_CMD_VOLT = 0x08u,                      // mV       | R       | R/W
  BQ27500_LLD_STD_CMD_Flags = 0x0Au, BQ27500_LLD_STD_CMD_FLAGS = 0x0Au,                       // N/A      | R       | R/W
  BQ27500_LLD_STD_CMD_NominalAvailableCapacity = 0x0Cu, BQ27500_LLD_STD_CMD_NAC = 0x0Cu,      // mAh      | R       | R/W
  BQ27500_LLD_STD_CMD_FullAvailableCapacity = 0x0Eu, BQ27500_LLD_STD_CMD_FAC = 0x0Eu,         // mAh      | R       | R/W
  BQ27500_LLD_STD_CMD_RemainingCapacity = 0x10u, BQ27500_LLD_STD_CMD_RM = 0x10u,              // mAh      | R       | R/W
  BQ27500_LLD_STD_CMD_FullChargeCapacity = 0x12u, BQ27500_LLD_STD_CMD_FCC = 0x12u,            // mAh      | R       | R/W
  BQ27500_LLD_STD_CMD_AverageCurrent = 0x14u, BQ27500_LLD_STD_CMD_AI = 0x14u,                 // mA       | R       | R/W
  BQ27500_LLD_STD_CMD_TimeToEmpty = 0x16u, BQ27500_LLD_STD_CMD_TTE = 0x16u,                   // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_TimeToFull = 0x18u, BQ27500_LLD_STD_CMD_TTF = 0x18u,                    // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_StandbyCurrent = 0x1Au, BQ27500_LLD_STD_CMD_SI = 0x1Au,                 // mA       | R       | R/W
  BQ27500_LLD_STD_CMD_StandbyTimeToEmpty = 0x1Cu, BQ27500_LLD_STD_CMD_STTE = 0x1Cu,           // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_MaxLoadCurrent = 0x1Eu, BQ27500_LLD_STD_CMD_MLI = 0x1Eu,                // mA       | R       | R/W
  BQ27500_LLD_STD_CMD_MaxLoadTimeToEmpty = 0x20u, BQ27500_LLD_STD_CMD_MLTTE = 0x20u,          // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_AvailableEnergy = 0x22u, BQ27500_LLD_STD_CMD_AE = 0x22u,                // mWh      | R       | R/W
  BQ27500_LLD_STD_CMD_AveragePower = 0x24u, BQ27500_LLD_STD_CMD_AP = 0x24u,                   // mW       | R       | R/W
  BQ27500_LLD_STD_CMD_TimeToEmptyAtConstantPower = 0x26u, BQ27500_LLD_STD_CMD_TTECP = 0x26u,  // minutes  | R       | R/W
  BQ27500_LLD_STD_CMD_CycleCount = 0x2Au, BQ27500_LLD_STD_CMD_CC = 0x2Au,                     // counts   | R       | R/W
  BQ27500_LLD_STD_CMD_StateOfCharge = 0x2Cu, BQ27500_LLD_STD_CMD_SOC = 0x2Cu                  // %        | R       | R/W
} bq27500_lld_std_command_t;

/**
 * @brief Subcommands of the control command.
 */
typedef enum {
                                                  // sealed access
                                                  // -------------
  BQ27500_LLD_SUB_CMD_CONTROL_STATUS = 0x0000u,   // yes
  BQ27500_LLD_SUB_CMD_DEVICE_TYPE = 0x0001u,      // yes
  BQ27500_LLD_SUB_CMD_FW_VERSION = 0x0002u,       // yes
  BQ27500_LLD_SUB_CMD_HW_VERSION = 0x0003u,       // yes
  BQ27500_LLD_SUB_CMD_DF_CHECKSUM = 0x0004u,      // no
  BQ27500_LLD_SUB_CMD_RESET_DATA = 0x0005u,       // yes
  BQ27500_LLD_SUB_CMD_PREV_MACWRITE = 0x0007u,    // yes
  BQ27500_LLD_SUB_CMD_CHEM_ID = 0x0008u,          // yes
  BQ27500_LLD_SUB_CMD_BOARD_OFFSET = 0x0009u,     // no
  BQ27500_LLD_SUB_CMD_CC_INT_OFFSET = 0x000Au,    // no
  BQ27500_LLD_SUB_CMD_WRITE_OFFSET = 0x000Bu,     // no
  BQ27500_LLD_SUB_CMD_SET_HIBERNATE = 0x0011u,    // yes
  BQ27500_LLD_SUB_CMD_CLEAR_HIBERNATE = 0x0012u,  // yes
  BQ27500_LLD_SUB_CMD_SET_SLEEPp = 0x0013u,       // yes
  BQ27500_LLD_SUB_CMD_CLEAR_SLEEPp = 0x0014u,     // yes
  BQ27500_LLD_SUB_CMD_SEALED = 0x0020u,           // no
  BQ27500_LLD_SUB_CMD_IT_ENABLE = 0x0021u,        // no
  BQ27500_LLD_SUB_CMD_IF_CHECKSUM = 0x0022u,      // no
  BQ27500_LLD_SUB_CMD_CAL_MODE = 0x0040u,         // no
  BQ27500_LLD_SUB_CMD_RESET = 0x0041u,            // no
} bq27500_lld_control_subcmd_t;

/**
 * @brief Extended commands.
 */
typedef enum {
                                                                          // length | unit | sealed | unsealed
                                                                          // -------+------+--------+----------
  BQ27500_LLD_EXT_CMD_DesignCapacity = 0x3Cu, EXT_CMD_DCAP = 0x3Cu,       //      2 |  mAh |     R  |      R
  BQ27500_LLD_EXT_CMD_DataFlashClass = 0x3Eu, EXT_CMD_DFCLS = 0x3Eu,      //      1 |  N/A |    N/A |     R/W
  BQ27500_LLD_EXT_CMD_DataFlashBlock = 0x3Fu, EXT_CMD_DFBLK = 0x3Fu,      //      1 |  N/A |    R/W |     R/W
  BQ27500_LLD_EXT_CMD_BlockData = 0x40u, EXT_CMD_DFD = 0x40u,             //     32 |  N/A |     R  |     R/W
  BQ27500_LLD_EXT_CMD_BlockDataCheckSum = 0x60u,EXT_CMD_DFDCKS = 0x60u,   //      1 |  N/A |    R/W |     R/W
  BQ27500_LLD_EXT_CMD_BlockDataControl = 0x61u, EXT_CMD_DFDCNTL = 0x61u,  //      1 |  N/A |    N/A |     R/W
  BQ27500_LLD_EXT_CMD_DeviceNameLength = 0x62u, EXT_CMD_DNAMELEN = 0x62u, //      1 |  N/A |     R  |      R
  BQ27500_LLD_EXT_CMD_DeviceName = 0x63u, EXT_CMD_DNAME = 0x63u,          //      7 |  N/A |     R  |      R
  BQ27500_LLD_EXT_CMD_ApplicationStatus = 0x6Au, EXT_CMD_APPSTAT = 0x6Au  //      1 |  N/A |     R  |      R
} bq27500_lld_ext_command_t;

/**
 * @brief Read or write access to extended commands.
 */
typedef enum {
  BQ27500_LLD_EXT_CMD_READ = 0,
  BQ27500_LLD_EXT_CMD_WRITE = 1,
} bq27500_lld_ext_cmd_access_t;

/**
 * @brief Union to inspect the version value.
 */
typedef union {
  uint16_t value;
  struct {
    uint16_t minor_low : 4;
    uint16_t minor_high : 4;
    uint16_t major_low : 4;
    uint16_t major_high : 4;
  } content;
} bq27500_lld_version_t;

/**
 * @brief Union to inspect the flags register.
 */
typedef union {
  uint16_t value;
  struct {
    uint16_t dsg : 1;
    uint16_t socf : 1;
    uint16_t soc1 : 1;
    uint16_t bat_det : 1;
    uint16_t wait_id : 1;
    uint16_t ocv_gd : 1;
    uint16_t rsvd_6_7 : 2;
    uint16_t chg : 1;
    uint16_t fc : 1;
    uint16_t xchg : 1;
    uint16_t chg_inh : 1;
    uint16_t rsvd_12_13 : 2;
    uint16_t otd : 1;
    uint16_t otc : 1;
  } content;
} bq27500_lld_flags_t;

/**
 * @brief Union to inspect the control status.
 */
typedef union {
  uint16_t value;
  struct {
    uint16_t qen : 1;
    uint16_t vok : 1;
    uint16_t rup_dis : 1;
    uint16_t ldmd : 1;
    uint16_t sleep : 1;
    uint16_t snooze : 1;
    uint16_t hibernate : 1;
    uint16_t rsvd_7_8_9 : 3;
    uint16_t bca : 1;
    uint16_t cca : 1;
    uint16_t csv : 1;
    uint16_t ss : 1;
    uint16_t fas : 1;
    uint16_t rsvd_15 : 1;
  } content;
} bq27500_lld_control_status_t;

/**
 * @brief BQ27500 Driver struct.
 */
typedef struct {
  apalI2CDriver_t* i2cd;          /**< @brief I2C driver to access the BQ27500 */
  apalControlGpio_t gpio_batlow;  /**< @brief Gpio to indicate whether the battery is low  */
  apalControlGpio_t gpio_batgood; /**< @brief Gpio to indicate whether the battery is good */
} BQ27500Driver;

/**
 * @brief Battery low enum.
 */
typedef enum {
  BQ27500_LLD_BATTERY_LOW = 0x01,
  BQ27500_LLD_BATTERY_NOT_LOW = 0x00,
} bq27500_lld_batlow_t;

/**
 * @brief Battery good enum.
 */
typedef enum {
  BQ27500_LLD_BATTERY_GOOD = 0x01,
  BQ27500_LLD_BATTERY_NOT_GOOD = 0x00,
} bq27500_lld_batgood_t;

#ifdef __cplusplus
extern "C" {
#endif
  apalExitStatus_t bq27500_lld_read_batlow(const BQ27500Driver* const bq27500, bq27500_lld_batlow_t* const batlow);
  apalExitStatus_t bq27500_lld_read_batgood(const BQ27500Driver* const bq27500, bq27500_lld_batgood_t* const batgood);
  apalExitStatus_t bq27500_lld_std_command(const BQ27500Driver* const bq27500, const bq27500_lld_std_command_t cmd, uint16_t* const dst, const apalTime_t timeout);
  apalExitStatus_t bq27500_lld_sub_command_call(const BQ27500Driver* const bq27500, const bq27500_lld_control_subcmd_t cmd, const apalTime_t timeout);
  apalExitStatus_t bq27500_lld_sub_command_read(const BQ27500Driver* const bq27500, uint16_t* const data, const apalTime_t timeout);
  apalExitStatus_t bq27500_lld_ext_command(const BQ27500Driver* const bq27500, const bq27500_lld_ext_command_t cmd, const bq27500_lld_ext_cmd_access_t rw, uint8_t* const buffer, const uint8_t length, const uint8_t offset, const apalTime_t timeout);
  apalExitStatus_t bq27500_lld_send_ctnl_data(const BQ27500Driver* const bq27500, const uint16_t data, const apalTime_t timeout);
  apalExitStatus_t bq27500_lld_compute_blockdata_checksum(const uint8_t* const blockdata, uint8_t* const sum);
#ifdef __cplusplus
}
#endif

#endif /* defined(AMIROLLD_CFG_USE_BQ27500) */

#endif /* _AMIROLLD_BQ27500_H_ */
