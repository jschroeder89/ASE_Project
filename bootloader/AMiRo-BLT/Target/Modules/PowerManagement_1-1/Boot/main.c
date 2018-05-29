/************************************************************************************//**
* \file         Demo\ARMCM4_STM32_Olimex_STM32E407_GCC\Boot\main.c
* \brief        Bootloader application source file.
* \ingroup      Boot_ARMCM4_STM32_Olimex_STM32E407_GCC
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2013  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with OpenBLT.
* If not, see <http://www.gnu.org/licenses/>.
*
* A special exception to the GPL is included to allow you to distribute a combined work
* that includes OpenBLT without being obliged to provide the source code for any
* proprietary components. The exception text is included at the bottom of the license
* file <license.html>.
*
* \endinternal
****************************************************************************************/

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#include "com.h"
#include "ARMCM4_STM32/types.h"
#include "AMiRo/amiroblt.h"
#include "helper.h"
#include "iodef.h"

/****************************************************************************************
* Defines
****************************************************************************************/
#define HIBERNATE_TIME_MS       5000

/****************************************************************************************
* Function prototypes and static variables
****************************************************************************************/
static void Init(void);

static void initGpio();
static void initExti();
void configGpioForShutdown();
void systemPowerDown();

ErrorStatus handleColdReset();
ErrorStatus handleSoftwareReset();
ErrorStatus handleUartDnWakeup();
ErrorStatus handlePathDcWakeup();
ErrorStatus handleTouchWakeup();
ErrorStatus handleIwdgWakeup();

static void indicateHibernate();
static void AdcSingleMeasurement();

ADC_TypeDef* setupADC(ADC_TypeDef* adc, const uint16_t low_th, const uint16_t high_th);
uint16_t configIwdg(const uint16_t ms);

ErrorStatus shutdownDisambiguationProcedure(const uint8_t type);
void shutdownToTransportation();
void shutdownToDeepsleep();
void shutdownToHibernate();
void shutdownAndRestart();

volatile blBackupRegister_t backup_reg;

/****************************************************************************************
* Callback configuration
****************************************************************************************/
void blCallbackShutdownTransportation(void);
void blCallbackShutdownDeepsleep(void);
void blCallbackShutdownHibernate(void);
void blCallbackShutdownRestart(void);
void blCallbackHandleShutdownRequest(void);

const blCallbackTable_t cbtable __attribute__ ((section ("_callback_table"))) = {
  .magicNumber = BL_MAGIC_NUMBER,
  .vBootloader = {BL_VERSION_ID_AMiRoBLT_Release, BL_VERSION_MAJOR, BL_VERSION_MINOR, 1},
  .vSSSP = {BL_VERSION_ID_SSSP, SSSP_VERSION_MAJOR, SSSP_VERSION_MINOR, 0},
  .vCompiler = {BL_VERSION_ID_GCC, __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__},  // currently only GCC is supported
  .cbShutdownHibernate = blCallbackShutdownHibernate,
  .cbShutdownDeepsleep = blCallbackShutdownDeepsleep,
  .cbShutdownTransportation = blCallbackShutdownTransportation,
  .cbShutdownRestart = blCallbackShutdownRestart,
  .cbHandleShutdownRequest = blCallbackHandleShutdownRequest,
  .cb5 = (void*)0,
  .cb6 = (void*)0,
  .cb7 = (void*)0,
  .cb8 = (void*)0,
  .cb9 = (void*)0,
  .cb10 = (void*)0,
  .cb11 = (void*)0
};

/************************************************************************************//**
** \brief     This is the entry point for the bootloader application and is called
**            by the reset interrupt vector after the C-startup routines executed.
** \return    none.
**
****************************************************************************************/
void main(void)
{
  /* initialize the microcontroller */
  Init();

  /* activate some required clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  /* initialize GPIOs and EXTI lines */
  initGpio();
  setLed(BLT_TRUE);
  initExti();

  /* initialize the timer */
  TimerInit(); // do not use saTimerInit() in order to initialize the static variable.

  /* read the backup register */
  backup_reg.raw = RTC_ReadBackupRegister(BL_RTC_BACKUP_REG);

  /* detect the primary reason for this wakeup/restart */
  backup_reg.wakeup_pri_reason =
      ((RCC_GetFlagStatus(RCC_FLAG_LPWRRST) == SET) ? BL_WAKEUP_PRI_RSN_LPWRRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET) ? BL_WAKEUP_PRI_RSN_WWDGRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) ? BL_WAKEUP_PRI_RSN_IWDGRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET) ? BL_WAKEUP_PRI_RSN_SFTRST : 0)   |
      ((RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET) ? BL_WAKEUP_PRI_RSN_PORRST : 0)   |
      ((RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET) ? BL_WAKEUP_PRI_RSN_PINRST : 0)   |
      ((RCC_GetFlagStatus(RCC_FLAG_BORRST) == SET) ? BL_WAKEUP_PRI_RSN_BORRST : 0)   |
      ((PWR_GetFlagStatus(PWR_FLAG_WU) == SET) ? BL_WAKEUP_PRI_RSN_WKUP : 0);

  /* when woken from standby mode, detect the secondary reason for this wakeup/reset */
  if ( (backup_reg.wakeup_pri_reason & BL_WAKEUP_PRI_RSN_WKUP) && (PWR_GetFlagStatus(PWR_FLAG_SB) == SET) ) {
    if (GPIO_ReadInputDataBit(SYS_UART_DN_GPIO, SYS_UART_DN_PIN) == Bit_RESET) {
      backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UART;
    } else if (GPIO_ReadInputDataBit(PATH_DC_GPIO, PATH_DC_PIN) == Bit_SET) {
      backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_PWRPLUG;
    } else {
      backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_TOUCH;
    }
  } else {
    backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;
  }

  /* store the information about this wakeup/restart in the backup register */
  PWR_BackupAccessCmd(ENABLE);
  RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup_reg.raw);

  /* clear the flags */
  RCC_ClearFlag();
  PWR_ClearFlag(PWR_FLAG_WU);

  setLed(BLT_FALSE);

  /* handle different wakeup/reset reasons */
  ErrorStatus status = ERROR;
  if (backup_reg.wakeup_pri_reason & BL_WAKEUP_PRI_RSN_SFTRST) {
    /* system was reset by software */
    status = handleSoftwareReset();
  } else if (backup_reg.wakeup_pri_reason & BL_WAKEUP_PRI_RSN_WKUP) {
    /* system was woken via WKUP pin */
    /* differeciate between thre wakeup types */
    switch (backup_reg.wakeup_sec_reason) {
      case BL_WAKEUP_SEC_RSN_UART:
        status = handleUartDnWakeup();
        break;
      case BL_WAKEUP_SEC_RSN_PWRPLUG:
        status = handlePathDcWakeup();
        break;
      case BL_WAKEUP_SEC_RSN_TOUCH:
        status = handleTouchWakeup();
        break;
      default:
        status = ERROR;
        break;
    }
  } else if (backup_reg.wakeup_pri_reason & BL_WAKEUP_PRI_RSN_IWDGRST) {
    /* system was woken by IWDG */
    status = handleIwdgWakeup();
  } else if (backup_reg.wakeup_pri_reason == BL_WAKEUP_PRI_RSN_PINRST) {
    /* system was reset via NRST pin */
    status = handleColdReset();
  } else {
    /* system was woken/reset for an unexpected reason.
     * In this case the LED blinks "SOS" (... --- ...) and the system resets.
     */
    blinkSOS(1);
    status = ERROR;
    backup_reg.shutdown_pri_reason = BL_SHUTDOWN_PRI_RSN_RESTART;
    backup_reg.shutdown_sec_reason = BL_SHUTDOWN_SEC_RSN_UNKNOWN;
    RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup_reg.raw);
    NVIC_SystemReset();
  }

  /* if something went wrong, signal this failure */
  if (status != SUCCESS) {
    blinkSOSinf();
  }

  return;
} /*** end of main ***/


/************************************************************************************//**
** \brief     Initializes the microcontroller.
** \return    none.
**
****************************************************************************************/
static void Init(void)
{
#if (BOOT_COM_UART_ENABLE > 0 || BOOT_GATE_UART_ENABLE > 0)
  GPIO_InitTypeDef  GPIO_InitStructure;
#elif (BOOT_FILE_SYS_ENABLE > 0)
  GPIO_InitTypeDef  GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
#elif (BOOT_COM_CAN_ENABLE > 0 || BOOT_GATE_CAN_ENABLE > 0)
  GPIO_InitTypeDef  GPIO_InitStructure;
#endif

  /* initialize the system and its clocks */
  SystemInit();
#if (BOOT_COM_UART_ENABLE > 0 || BOOT_GATE_UART_ENABLE > 0)
  /* enable UART peripheral clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
  /* enable GPIO peripheral clock for transmitter and receiver pins */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* connect the pin to the peripherals alternate function */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
  /* configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  /* configure USART Rx as alternate function */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
  /* enable UART peripheral clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  /* enable GPIO peripheral clock for transmitter and receiver pins */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* connect the pin to the peripherals alternate function */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
  /* configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  /* configure USART Rx as alternate function */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure Bluetooth reset pin */
  GPIO_InitTypeDef  gpio_init;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  gpio_init.GPIO_Pin   = BT_RST_PIN;
  gpio_init.GPIO_OType = GPIO_OType_OD;
  gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
  gpio_init.GPIO_Mode = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BT_RST_GPIO, &gpio_init);
  /* Reset Bluetooth reset pin */
  GPIO_ResetBits(BT_RST_GPIO, BT_RST_PIN);
#endif


#if (BOOT_COM_CAN_ENABLE > 0 || BOOT_GATE_CAN_ENABLE > 0)
  /* enable clocks for CAN transmitter and receiver pins */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  /* select alternate function for the CAN pins */
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
  /* configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif

} /*** end of Init ***/

/*
 * Initializes all GPIO used by the bootloader
 */
static void initGpio() {
  GPIO_InitTypeDef gpio_init;

  /*
   * OUTPUTS
   */

  /* initialize LED and push it up (inactive) */
  GPIO_SetBits(LED_GPIO, LED_PIN);
  gpio_init.GPIO_Pin    = LED_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(LED_GPIO, &gpio_init);

  /* initialize SYS_PD_N and push it up (inactive) */
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  gpio_init.GPIO_Pin    = SYS_PD_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_OD;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(SYS_PD_N_GPIO, &gpio_init);

  /* initialize SYS_SYNC_N and pull it down (active) */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  gpio_init.GPIO_Pin    = SYS_SYNC_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_OD;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(SYS_SYNC_N_GPIO, &gpio_init);

  /* initialize SYS_WARMRST_N and pull it down (active) */
  GPIO_ResetBits(SYS_WARMRST_N_GPIO, SYS_WARMRST_N_PIN);
  gpio_init.GPIO_Pin    = SYS_WARMRST_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_OD;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(SYS_WARMRST_N_GPIO, &gpio_init);

  /* initialize SYS_UART_DN and push it up (inactive) */
  GPIO_SetBits(SYS_UART_DN_GPIO, SYS_UART_DN_PIN);
  gpio_init.GPIO_Pin    = SYS_UART_DN_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_OD;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(SYS_UART_DN_GPIO, &gpio_init);

  /* initialize POWER_EN and pull it down (inactive) */
  GPIO_ResetBits(POWER_EN_GPIO, POWER_EN_PIN);
  gpio_init.GPIO_Pin    = POWER_EN_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(POWER_EN_GPIO, &gpio_init);

  /* initialize SYS_REG_EN and pull it down (inactive) */
  GPIO_ResetBits(SYS_REG_EN_GPIO, SYS_REG_EN_PIN);
  gpio_init.GPIO_Pin    = SYS_REG_EN_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(SYS_REG_EN_GPIO, &gpio_init);

  /* initialize CHARGE_EN1_N and CHARGE_EN2_N and push them up (inactive) */
  GPIO_SetBits(CHARGE_EN1_N_GPIO, CHARGE_EN1_N_PIN);
  GPIO_SetBits(CHARGE_EN2_N_GPIO, CHARGE_EN2_N_PIN);
  gpio_init.GPIO_Pin    = CHARGE_EN1_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_OUT;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(CHARGE_EN1_N_GPIO, &gpio_init);
  gpio_init.GPIO_Pin    = CHARGE_EN2_N_PIN;
  GPIO_Init(CHARGE_EN2_N_GPIO, &gpio_init);

  /*
   * INPUTS
   */

  /* initialize PATH_DC */
  gpio_init.GPIO_Pin    = PATH_DC_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_IN;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(PATH_DC_GPIO, &gpio_init);

  /* initialize TOUCH_INT_N */
  gpio_init.GPIO_Pin    = TOUCH_INT_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_IN;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(TOUCH_INT_N_GPIO, &gpio_init);

  /* initialize VSYS_SENSE as analog input */
  gpio_init.GPIO_Pin    = VSYS_SENSE_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_AN;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(VSYS_SENSE_GPIO, &gpio_init);

  /* initialize GPIOB4, since it is configured in alternate function mode on reset */
  gpio_init.GPIO_Pin    = CHARGE_STAT2A_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_IN;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;
  GPIO_Init(CHARGE_STAT2A_GPIO, &gpio_init);

  return;
} /*** end of initGpio ***/

/*
 * Initialize all EXTI lines
 */
static void initExti() {
  /* configure EXTI lines */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource0); // IR_INT1_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource0); // CHARGE_STAT1A
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource1); // GAUGE_BATLOW1
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource2); // GAUGE_BATGD1_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource3); // SYS_UART_DN
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4); // CHARGE_STAT2A
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4); // IR_INT2_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5); // TOUCH_INT_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource6); // GAUGE_BATLOW2
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7); // GAUGE_BATGD2_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8); // PATH_DC
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9); // SYS_SPI_DIR
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource12); // SYS_SYNC_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13); // SYS_PD_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14); // SYS_WARMRST_N
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource15); // SYS_UART_UP

  return;
} /*** end of initExti ***/

/*
 * Signals, which type of low-power mode the system shall enter after the shutdown sequence.
 */
ErrorStatus shutdownDisambiguationProcedure(const uint8_t type) {
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  ErrorStatus ret_val = ERROR;

  switch (type) {
    case BL_SHUTDOWN_PRI_RSN_UNKNOWN:
    case BL_SHUTDOWN_PRI_RSN_HIBERNATE:
    case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
    case BL_SHUTDOWN_PRI_RSN_TRANSPORT:
    {
      // broadcast a number of pulses, depending on the argument
      uint8_t pulse_counter = 0;
      for (pulse_counter = 0; pulse_counter < type; ++pulse_counter) {
        msleep(1);
        GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
        msleep(1);
        GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
      }
      // wait for timeout
      msleep(10);
      ret_val = SUCCESS;
      break;
    }
    case BL_SHUTDOWN_PRI_RSN_RESTART:
    {
      // since there is no ambiguity for restart requests, no pulses are generated
      msleep(10);
      ret_val = SUCCESS;
      break;
    }
    default:
      ret_val = ERROR;
      break;
  }

  return ret_val;
} /*** end of shutdownDisambiguationProcedure ***/

/*
 * Final shutdown of the system to enter transportation mode.
 */
void shutdownToTransportation() {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* power down the system */
  systemPowerDown();

  /* deactivate the WKUP pin */
  PWR_WakeUpPinCmd(DISABLE);

  /* deactivate any RTC related events */
  RTC_WakeUpCmd(DISABLE);
  RTC_TamperCmd(RTC_Tamper_1, DISABLE);
  RTC_TimeStampCmd(RTC_TimeStampEdge_Rising, DISABLE);
  RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, DISABLE);
  RTC_ClearFlag(~0);

  /* disable the IWDG */
  IWDG_ReloadCounter();

  /* write some information to the backup register */
  blBackupRegister_t backup;
  backup.shutdown_pri_reason = BL_SHUTDOWN_PRI_RSN_TRANSPORT;
  backup.shutdown_sec_reason = BL_SHUTDOWN_SEC_RSN_UNKNOWN;
  backup.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_UNKNOWN;
  backup.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;
  PWR_BackupAccessCmd(ENABLE);
  RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup.raw);

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode */
  PWR_EnterSTANDBYMode();

  return;
} /*** end of shutdownToTransportation ***/

/*
 * Final shutdown of the system to enter deepseleep mode.
 */
void shutdownToDeepsleep() {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* power down the system */
  systemPowerDown();

  /* activate the WKUP pin */
  PWR_WakeUpPinCmd(ENABLE);

  /*
   * Configuration of RTC and IWDG belongs to the OS.
   */

  /* write some information to the backup register */
  blBackupRegister_t backup;
  backup.shutdown_pri_reason = BL_SHUTDOWN_PRI_RSN_DEEPSLEEP;
  backup.shutdown_sec_reason = BL_SHUTDOWN_SEC_RSN_UNKNOWN;
  backup.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_UNKNOWN;
  backup.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;
  PWR_BackupAccessCmd(ENABLE);
  RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup.raw);

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode or restart the system in case a power plug is already present */
  if (GPIO_ReadInputDataBit(PATH_DC_GPIO, PATH_DC_PIN) != Bit_SET) {
    PWR_EnterSTANDBYMode();
  } else {
    NVIC_SystemReset();
  }

  return;
} /*** end of shutdownToDeepsleep ***/

/*
 * Final shutdown of the system to enter hibernate mode.
 */
void shutdownToHibernate() {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* power down the system */
  systemPowerDown();

  /* write some information to the backup register */
  blBackupRegister_t backup;
  backup.shutdown_pri_reason = BL_SHUTDOWN_PRI_RSN_HIBERNATE;
  backup.shutdown_sec_reason = BL_SHUTDOWN_SEC_RSN_UNKNOWN;
  backup.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_UNKNOWN;
  backup.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;
  PWR_BackupAccessCmd(ENABLE);
  RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup.raw);

  /* morse 'OK' via the LED to signal that shutodnw was successful */
  blinkOK(1);

  /* reset the MCU */
  NVIC_SystemReset();

  return;
} /*** end of shutdownToHibernate ***/

/*
 * Final shutdown of the system and restart.
 */
void shutdownAndRestart() {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* power down the system */
  systemPowerDown();

  /* write some information to the backup register */
  blBackupRegister_t backup;
  backup.shutdown_pri_reason = BL_SHUTDOWN_PRI_RSN_RESTART;
  backup.shutdown_sec_reason = BL_SHUTDOWN_SEC_RSN_UNKNOWN;
  backup.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_UNKNOWN;
  backup.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;
  PWR_BackupAccessCmd(ENABLE);
  RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup.raw);

  /* morse 'OK' via the LED to signal that shutodnw was successful */
  blinkOK(1);

  /* reset the MCU */
  NVIC_SystemReset();

  return;
} /*** end of shutdownAndRestart ***/

/*
 * Configures some GPIO pins as inputs for safety reasons.
 * Under certain circumstances, these pins might power hardware that is supposed to be shut down.
 */
void configGpioForShutdown() {
  /* setup the configuration */
  GPIO_InitTypeDef  gpio_init;
  gpio_init.GPIO_Mode   = GPIO_Mode_IN;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  gpio_init.GPIO_OType  = GPIO_OType_PP;
  gpio_init.GPIO_PuPd   = GPIO_PuPd_NOPULL;

  /* configure SYS_UART_TX */
  gpio_init.GPIO_Pin = SYS_UART_TX_PIN;
  GPIO_Init(SYS_UART_TX_GPIO, &gpio_init);

  /* configure all SYS_SPI signals */
  gpio_init.GPIO_Pin = SYS_SPI_SS0_N_PIN;
  GPIO_Init(SYS_SPI_SS0_N_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = SYS_SPI_SCLK_PIN;
  GPIO_Init(SYS_SPI_SCLK_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = SYS_SPI_MISO_PIN;
  GPIO_Init(SYS_SPI_MISO_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = SYS_SPI_MOSI_PIN;
  GPIO_Init(SYS_SPI_MOSI_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = SYS_SPI_SS1_N_PIN;
  GPIO_Init(SYS_SPI_SS1_N_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = SYS_SPI_DIR_PIN;
  GPIO_Init(SYS_SPI_DIR_GPIO, &gpio_init);

  /* configure CAN_TX */
  gpio_init.GPIO_Pin = CAN_TX_PIN;
  GPIO_Init(CAN_TX_GPIO, &gpio_init);

  /* configure all Bluetooth signals */
  gpio_init.GPIO_Pin = BT_CTS_PIN;
  GPIO_Init(BT_CTS_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = BT_RX_PIN;
  GPIO_Init(BT_RX_GPIO, &gpio_init);

  return;
} /*** end of configGpioForShutdown ***/

/*
 * Disables all regulated voltages and finally cuts power to the rest of the system.
 */
void systemPowerDown() {
  setLed(BLT_TRUE);

  /* make sure that all other modules are shut down */
  msleep(10);

  /* reset slave modules */
  GPIO_ResetBits(SYS_WARMRST_N_GPIO, SYS_WARMRST_N_PIN);

  /* disable voltage regulators */
  GPIO_ResetBits(SYS_REG_EN_GPIO, SYS_REG_EN_PIN);

  /* cut power */
  GPIO_ResetBits(POWER_EN_GPIO, POWER_EN_PIN);

  /* make sure, all capacitors are discharged */
  msleep(100);

  setLed(BLT_FALSE);

  return;
} /*** end of systemPowerDown ***/

/*
 * Cofigures the independent watchdog (IWDG) to fire after the specified time when it is enabled.
 * The argument is the requested time in milliseconds.
 * The time that was actually set for the IWDG is returned by the function (again in milliseconds).
 * In some cases the returned value might differ from the requested one, but if so, it will alwyas be smaller.
 * Although the IWDG provides higher resolutions than milliseconds, these are not supported by this function.
 */
uint16_t configIwdg(const uint16_t ms) {
  /* apply an upper bound to the ms argument */
  uint16_t ms_capped = (ms >= 0x8000) ? 0x7FFF : ms;

  /* detect the best fitting prescaler and compute the according reload value */
  uint8_t prescaler = 0;
  uint16_t reload_val = 0;
  if (ms_capped >= 0x4000) {
    prescaler = IWDG_Prescaler_256;
    reload_val = ms_capped >> 3;  // note: this corresponds to a floor function
    ms_capped = reload_val << 3;  // this applies the floor function to ms_capped
  } else if (ms_capped >= 0x2000) {
    prescaler = IWDG_Prescaler_128;
    reload_val = ms_capped >> 2;  // note: this corresponds to a floor function
    ms_capped = reload_val << 2;  // this applies the floor function to ms_capped
  } else if (ms_capped >= 0x1000) {
    ms_capped &= ~(0x0001);
    prescaler = IWDG_Prescaler_64;
    reload_val = ms_capped >> 1;  // note: this corresponds to a floor function
    ms_capped = reload_val << 1;  // this applies the floor function to ms_capped
  } else {
    prescaler = IWDG_Prescaler_32;
    reload_val = ms_capped;
  }

  /* configure the IWDG */
  if (reload_val > 0) {
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prescaler);
    IWDG_SetReload(reload_val);
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Disable);
  }

  return ms_capped;
} /*** end of configIWDG ***/

/*
 * System was reset via the NRST pin or the reason could not be detected.
 * In this case, everything is started up.
 * If an attempt for an OS update is detected, flashing mode is entered.
 * Otherwise, the system will boot the OS.
 */
ErrorStatus handleColdReset() {
  /* activate system power and wait some time to ensure stable voltages */
  setLed(BLT_TRUE);
  GPIO_SetBits(POWER_EN_GPIO, POWER_EN_PIN);
  msleep(10);
  GPIO_SetBits(SYS_REG_EN_GPIO, SYS_REG_EN_PIN);
  msleep(10);
  setLed(BLT_FALSE);

  /* drive SYS_WARMRST_N high (inactive) */
  GPIO_SetBits(SYS_WARMRST_N_GPIO, SYS_WARMRST_N_PIN);

  /* enable CAN clock
   * Note that CAN1 shares reception filters with CAN1 so for CAN2 the CAN1 peripheral also needs to be enabled. */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2 | RCC_APB1Periph_CAN1, ENABLE);

  /* wait 1ms to make sure that all modules are running and started the bootloader */
  msleep(1);

  /* initialize the bootloader */
  BootInit();

  /* start the infinite program loop */
  uint32_t loopStartTime = 0;
  saTimerUpdate(&loopStartTime);
  uint32_t currentTime = loopStartTime;
  while (1)
  {
//    /* make the LED "double-blink" */
//    saTimerUpdate(&currentTime);
//    if (currentTime < loopStartTime + 50) {
//      setLed(BLT_TRUE);
//    } else if (currentTime < loopStartTime + 50+100) {
//      setLed(BLT_FALSE);
//    } else if (currentTime < loopStartTime + 50+100+50) {
//      setLed(BLT_TRUE);
//    } else if (currentTime < loopStartTime + 50+100+50+300) {
//      setLed(BLT_FALSE);
//    } else {
//      loopStartTime = currentTime;
//    }

    /* run the bootloader task */
    BootTask();

    /* check the SYS_PD_N signal */
    if (GPIO_ReadInputDataBit(SYS_PD_N_GPIO, SYS_PD_N_PIN) == Bit_RESET) {
      blCallbackHandleShutdownRequest();
      return SUCCESS;
    }
  }

  return ERROR;
} /*** end of handleColdReset ***/

/*
 * System was reset by software.
 * Depending on the argument, which was read from the 1st backup register (see main function) the effect of this function differs.
 * There are three cases that can occur:
 * - The system was reset to enter hibernate mode.
 *   In this case the system will enter a medium power saving mode (hibernate mode), but can be charged via the charging pins.
 *   The system can be woken up in the same way as in deepsleep mode (cf. blCallbackShutdownDeepsleep() function).
 * - The system was reset to reboot.
 *   In this case the system will restart in the same way as after a cold reset.
 * - The reason is unknown.
 *   This case will cause an error.
 */
ErrorStatus handleSoftwareReset() {
  /* action depends on original shutdown reason */
  switch (backup_reg.shutdown_pri_reason) {
    case BL_SHUTDOWN_PRI_RSN_HIBERNATE:
    {
      /* activate the WKUP pin */
      PWR_WakeUpPinCmd(ENABLE);

      /* deactivate any RTC related events */
      RTC_WakeUpCmd(DISABLE);
      RTC_TamperCmd(RTC_Tamper_1, DISABLE);
      RTC_TimeStampCmd(RTC_TimeStampEdge_Rising, DISABLE);
      RTC_TimeStampCmd(RTC_TimeStampEdge_Falling, DISABLE);

      /* configure the IWDG to wake the system from standby mode */
      uint16_t iwdg_ms = 1;
      if (GPIO_ReadInputDataBit(PATH_DC_GPIO, PATH_DC_PIN) != Bit_SET) {
        /* if a power plug is detected, fire immediately (1ms), else fire after the defined hibernate time */
        iwdg_ms = HIBERNATE_TIME_MS;
      }
      configIwdg(iwdg_ms);
      IWDG_Enable();

      /* enter standby mode */
      PWR_EnterSTANDBYMode();

      return SUCCESS;
      break;
    }
    case BL_SHUTDOWN_PRI_RSN_RESTART:
    {
      return handleColdReset();
      break;
    }
    case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
    {
      if (GPIO_ReadInputDataBit(PATH_DC_GPIO, PATH_DC_PIN) == Bit_SET) {
        return handlePathDcWakeup();
      } else {
        blCallbackShutdownDeepsleep();
      }
      break;
    }
    default:
      return ERROR;
  }
  return ERROR;
} /*** end of handleSoftwareReset ***/

/*
 * System was woken up via the WKUP pin and the SYS_UART_DN signal was found to be responsible.
 * In this case, the system starts as after a cold reset.
 * this function is identical to handleTouchWakeup().
 */
ErrorStatus handleUartDnWakeup() {
  return handleColdReset();
} /*** end of hanldeUartDnWakeup ***/

/*
 * System was woken up via the WKUP pin and the PATH_DC signal was found to be responsible.
 * If the system was woken from deepsleep mode, it will enter hibernate mode to enable charging as long as the power plug is present.
 * In any other case, the system will just enter the previous low-power mode again.
 */
ErrorStatus handlePathDcWakeup() {
  /* reenter the previous low-power mode */
  switch (backup_reg.shutdown_pri_reason) {
    case BL_SHUTDOWN_PRI_RSN_HIBERNATE:
      blCallbackShutdownHibernate();
      return SUCCESS;
      break;
    case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
      /* visualize that the power plug was detected
       * This is helpful for feedback, and required for the follwing reason:
       * When the power plug is detected, it takes some additional time for the ADC to detect a high voltage.
       * If the ADC detects a low voltage at the first attempt, the system will enter hibernate mode.
       * Thus, the ADC will measure the voltage again after several seconds and charging will start.
       * However, this behaviour does not meet the user expection.
       * Hence, the voltage has some to adapt at this point
       */
      setLed(BLT_TRUE);
      msleep(500);
      setLed(BLT_FALSE);

      return handleIwdgWakeup();
      break;
    case BL_SHUTDOWN_PRI_RSN_TRANSPORT:
      blCallbackShutdownTransportation();
      return SUCCESS;
      break;
    default:
      return ERROR;
      break;
  }

  return ERROR;
} /*** end of handlePathDcWakeup ***/

/*
 * System was woken up via the WKUP pin and the touch sensors were found to be responsible.
 * In this case the system starts as after an cold reset.
 * This function is identical to handleUartDnWakeup().
 */
ErrorStatus handleTouchWakeup() {
  return handleColdReset();
} /*** end of handleTouchWakeup ***/

/*
 * System was woken up via the IWDG.
 * In this case the ADC is configured and VSYS is measured once.
 * If VSYS is found to be high enough to charge the batteries, the system will stay active until VSYS drops or an EXTI event occurs.
 * Otherwise, the system will configure the IWDG to wake the system again after five seconds and enter standby mode.
 */
ErrorStatus handleIwdgWakeup() {
  /* handle different situations, depending on the backup data */
  if ((backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_HIBERNATE) ||
      (backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_DEEPSLEEP)) {
    /* handle periodic wakeup in hibernate mode and in deepsleep mode when a power plug was detetced */

    /* if in hibernate mode, indicate the DiWheelDrive to enter hibernate mode as well, so it will activate the charging pins */
    if (backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_HIBERNATE) {
      indicateHibernate();
    }

    /* measure the current voltage of VSYS */
    AdcSingleMeasurement();

    /* evaluate the value
     * The ADC value represents the analog voltage between Vref- (= GND = 0.0V) and Vref+ (= VDD = 3.3V) as 12-bit value.
     * Hence, the value read from the register is first scaled to [0V .. 3.3V].
     * Then, an additional factor 5.33 is applied to account the downscaling on the board.
     * Actually, the factor should be 5.0, but due to too large resistors it was corrected to 5.33.
     */
    if ( (((float)(ADC_GetConversionValue(ADC1)) / (float)(0x0FFF)) * 3.3f * 5.33f) < 9.0f ) {
      /* VSYS was found to be < 9V */

      /* re-enter power saving mode
       * If the system was shut down to deepsleep mode and the power plug was removed, re-enter deepsleep mode.
       * (This could be done earlier in this function, but since charging via the pins of the DeWheelDrive may be
       *  supported in the future, this is done after measuring VSYS)
       */
      if (backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_DEEPSLEEP &&
          GPIO_ReadInputDataBit(PATH_DC_GPIO, PATH_DC_PIN) == Bit_RESET) {
        blCallbackShutdownDeepsleep();
      } else {
        /* reconfigure the IWDG and power down for five seconds */
        configIwdg(HIBERNATE_TIME_MS);
        IWDG_Enable();

        /* enter standby mode */
        PWR_EnterSTANDBYMode();
      }

      return SUCCESS;
    } else {
      /* VSYS was found to be >= 9V */
      setLed(BLT_TRUE);

      /* charge the battieries */
      GPIO_ResetBits(CHARGE_EN1_N_GPIO, CHARGE_EN1_N_PIN);
      GPIO_ResetBits(CHARGE_EN2_N_GPIO, CHARGE_EN2_N_PIN);

      /* configure analog watchdoch to fire as soon as the voltage drops below 9V */
      ADC_DeInit();
      setupADC(ADC1, (uint16_t)(9.0f / 5.33f / 3.3f * (float)0x0FFF), 0x0FFF);

      EXTI_InitTypeDef exti;
      /* configure UART_DN EXTI */
      exti.EXTI_Line = EXTI_Line3;
      exti.EXTI_Mode = EXTI_Mode_Interrupt;
      exti.EXTI_Trigger = EXTI_Trigger_Falling;
      exti.EXTI_LineCmd = ENABLE;
      EXTI_Init(&exti);

      /* configure TOUCH_INT_N EXTI */
      exti.EXTI_Line = EXTI_Line5;
      exti.EXTI_Mode = EXTI_Mode_Interrupt;
      exti.EXTI_Trigger = EXTI_Trigger_Falling;
      exti.EXTI_LineCmd = ENABLE;
      EXTI_Init(&exti);

      /* configure PATH_DC EXTI */
      if (backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_DEEPSLEEP) {
        exti.EXTI_Line = EXTI_Line8;
        exti.EXTI_Mode = EXTI_Mode_Interrupt;
        exti.EXTI_Trigger = EXTI_Trigger_Falling;
        exti.EXTI_LineCmd = ENABLE;
        EXTI_Init(&exti);
      }

      /* configure the NVIC so ADC and EXTI will be handled */
      NVIC_InitTypeDef nvic;
      nvic.NVIC_IRQChannel = ADC_IRQn;
      nvic.NVIC_IRQChannelPreemptionPriority = 6;
      nvic.NVIC_IRQChannelSubPriority = 6;
      nvic.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&nvic);
      nvic.NVIC_IRQChannel = EXTI3_IRQn;
      nvic.NVIC_IRQChannelPreemptionPriority = 6;
      nvic.NVIC_IRQChannelSubPriority = 6;
      nvic.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&nvic);
      NVIC_EnableIRQ(EXTI3_IRQn);
      nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
      nvic.NVIC_IRQChannelPreemptionPriority = 6;
      nvic.NVIC_IRQChannelSubPriority = 6;
      nvic.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&nvic);
      NVIC_EnableIRQ(EXTI9_5_IRQn);

      /* activate the ADC */
      ADC_SoftwareStartConv(ADC1);

      /* sleep until something happens */
      __WFI();

      /* disable the chargers */
      GPIO_SetBits(CHARGE_EN1_N_GPIO, CHARGE_EN1_N_PIN);
      GPIO_SetBits(CHARGE_EN2_N_GPIO, CHARGE_EN2_N_PIN);
      setLed(BLT_FALSE);

      /* evaluate wakeup reason */
      // note: since I (tschoepp) don't know the difference between 'pending' and 'active' IRQs, both flags are ORed.
      uint8_t wkup_rsn = BL_WAKEUP_SEC_RSN_UNKNOWN;
      if ((NVIC_GetActive(ADC_IRQn) != 0 || NVIC_GetPendingIRQ(ADC_IRQn) != 0) &&
          ADC_GetITStatus(ADC1, ADC_IT_AWD) == SET &&
          ADC_GetFlagStatus(ADC1, ADC_FLAG_AWD) == SET) {
        wkup_rsn |= BL_WAKEUP_SEC_RSN_VSYSLOW;
      }
      if ((NVIC_GetActive(EXTI3_IRQn) != 0 || NVIC_GetPendingIRQ(EXTI3_IRQn) != 0) &&
          EXTI_GetFlagStatus(EXTI_Line3) == SET) {
        wkup_rsn |= BL_WAKEUP_SEC_RSN_UART;
      }
      if ((NVIC_GetActive(EXTI9_5_IRQn) != 0 || NVIC_GetPendingIRQ(EXTI9_5_IRQn) != 0) &&
          EXTI_GetFlagStatus(EXTI_Line5) == SET) {
        wkup_rsn |= BL_WAKEUP_SEC_RSN_TOUCH;
      }
      if ((NVIC_GetActive(EXTI9_5_IRQn) != 0 || NVIC_GetPendingIRQ(EXTI9_5_IRQn) != 0) &&
          EXTI_GetFlagStatus(EXTI_Line8) == SET) {
        wkup_rsn |= BL_WAKEUP_SEC_RSN_PWRPLUG;
      }

      /* since only the first interrupt will be handles, clear any pending ones */
      NVIC_DisableIRQ(ADC_IRQn);
      NVIC_DisableIRQ(EXTI3_IRQn);
      NVIC_DisableIRQ(EXTI9_5_IRQn);
      NVIC_ClearPendingIRQ(ADC_IRQn);
      NVIC_ClearPendingIRQ(EXTI3_IRQn);
      NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

      /* clear all pending EXTI events */
      EXTI_DeInit();
      EXTI_ClearFlag(EXTI_Line3);
      EXTI_ClearFlag(EXTI_Line5);
      EXTI_ClearFlag(EXTI_Line8);

      /* make sure the LED was visibly turned off */
      msleep(100);

      /* depending on the wakup reason, handle accordingly */
      if (wkup_rsn & BL_WAKEUP_SEC_RSN_TOUCH) {
        /* the system was interrupted via the TOUCH_INT_N signal */

        /* act as if this was a normal touch wakeup */
        backup_reg.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_WKUP;
        backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_TOUCH;
        RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup_reg.raw);
        return handleTouchWakeup();
      } else if (wkup_rsn & BL_WAKEUP_SEC_RSN_UART) {
        /* the system was interrupted via the SYS_UARTDN signal */

        /* act as if this was a normal UART wakeup */
        backup_reg.wakeup_pri_reason = BL_WAKEUP_PRI_RSN_WKUP;
        backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UART;
        RTC_WriteBackupRegister(BL_RTC_BACKUP_REG, backup_reg.raw);
        return handleUartDnWakeup();
      } else if (wkup_rsn & BL_WAKEUP_SEC_RSN_VSYSLOW) {
        /* VSYS has dropped below 9V */

        /* depending on the original reason for shutdown, act differenty */
        switch (backup_reg.shutdown_pri_reason) {
          case BL_SHUTDOWN_PRI_RSN_HIBERNATE:
          {
            blCallbackShutdownHibernate();
            return SUCCESS;
          }
          case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
          {
            NVIC_SystemReset();
            return SUCCESS;
          }
          default:
            return ERROR;
        }
      } else if (wkup_rsn & BL_WAKEUP_SEC_RSN_PWRPLUG) {
        /* system was interrupted because the power plug was removed
         * note: when a power cord is plugged in, this will not trigger an interrupt because the NVIC is configured for a falling edge only */
        if (backup_reg.shutdown_pri_reason == BL_SHUTDOWN_PRI_RSN_DEEPSLEEP) {
          blCallbackShutdownDeepsleep();
          return SUCCESS;
        } else {
          /* this state is undefined, because the PATH_DC inerrupt is only configured when the primary shutdown reason was to enter deepsleep mode */
          return ERROR;
        }
      } else {
        /* the system was interrupted for an unknown reason */
        return ERROR;
      }
    } // end of ADC evaluation
  } else {
    /* since it is unknown why the IWDG was configured, act as after a cold reset */
    return handleColdReset();
  }

  return ERROR;
} /*** end of handleIwdgWakeup ***/

/*
 * Indicates the DiWheelDrive module to enter hibernate mode at wakeup.
 * This function should be called quite at the beginning of the according handleXXXReset/Wakeup() methods.
 */
static void indicateHibernate() {
  /* signal the DiWheelDrive to enter hibernate mode as well, so it will activate the charging pins */
  GPIO_ResetBits(SYS_UART_DN_GPIO, SYS_UART_DN_PIN);
  msleep(10); // this must be that long, because the DiWheelDrive sleeps some time before evaluating any signals
  GPIO_SetBits(SYS_UART_DN_GPIO, SYS_UART_DN_PIN);

  /* if the DiWheeDrive needs some time for setup it may pull down the signal */
  waitForSignal(SYS_UART_DN_GPIO, SYS_UART_DN_PIN, Bit_SET);

  return;
} /*** end of indicateHibernate ***/

/*
 *Performs a one-shot measurement of the VSYS voltage.
 */
static void AdcSingleMeasurement() {
  /* reset and initialize ADC for single-shot measurement */
//    ADC_DeInit();
  setupADC(ADC1, 0, 0);

  /* initialize the NVIC so ADC interrupts are handled */
  NVIC_InitTypeDef nvic;
  nvic.NVIC_IRQChannel = ADC_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 6;
  nvic.NVIC_IRQChannelSubPriority = 6;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  /* measure the voltage once */
  setLed(BLT_TRUE);
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  NVIC_EnableIRQ(ADC_IRQn);
  ADC_SoftwareStartConv(ADC1);
  while (ADC_GetITStatus(ADC1, ADC_IT_EOC) != SET && ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != SET) {
    __WFI();
  }
  NVIC_DisableIRQ(ADC_IRQn);
  ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
  ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
  NVIC_ClearPendingIRQ(ADC_IRQn);
  setLed(BLT_FALSE);

  return;
} /*** end of AdcSingleMeasurement ***/

/*
 * Configures the ADC for measuring VSYS.
 * ADCx is the ADC object to initialize.
 * low_th and high_th are the threshold values for the analog watchdor (must be 12-bit!).
 * If low_th >= high_th, the ADC is configured for single-shot measurements.
 * Otherwise, the watchdog is configured with the corresponding thresholds.
 */
ADC_TypeDef* setupADC(ADC_TypeDef* adc, const uint16_t low_th, const uint16_t high_th) {
  /* evaluate the arguments */
  blt_bool awd_enable = BLT_FALSE;
  if (low_th < high_th) {
    awd_enable = BLT_TRUE;
  }

  /* enable the clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* enable the ADC (wakes it from low-power mode) */
  ADC_Cmd(adc, ENABLE);

  /* initialize the common registers */
  ADC_CommonInitTypeDef adc_cinit;
  ADC_CommonStructInit(&adc_cinit);
  adc_cinit.ADC_Prescaler = ADC_Prescaler_Div8; // clock as slow as possible
  ADC_CommonInit(&adc_cinit);

  /* initialize the ADC */
  ADC_InitTypeDef adc_init;
  ADC_StructInit(&adc_init);
  adc_init.ADC_ContinuousConvMode = (awd_enable == BLT_TRUE) ? ENABLE : DISABLE;
  ADC_Init(adc, &adc_init);

  /* disable internal sensors */
  ADC_TempSensorVrefintCmd(DISABLE);
  ADC_VBATCmd(DISABLE);

  /* configure ADC channel and speed */
  ADC_RegularChannelConfig(adc, ADC_Channel_9, 1, ADC_SampleTime_480Cycles);
  ADC_EOCOnEachRegularChannelCmd(adc, (awd_enable == BLT_TRUE) ? DISABLE : ENABLE);
  ADC_DiscModeCmd(adc, DISABLE);

  /* disable DMA */
  ADC_DMACmd(adc, DISABLE);

  /* disable injected mode */
  ADC_AutoInjectedConvCmd(adc, DISABLE);
  ADC_InjectedDiscModeCmd(adc, DISABLE);

  /* configure the analog watchdog */
  if (awd_enable == BLT_TRUE) {
    ADC_AnalogWatchdogSingleChannelConfig(adc, ADC_Channel_9);
    ADC_AnalogWatchdogThresholdsConfig(adc, high_th, low_th);
    ADC_AnalogWatchdogCmd(adc, ADC_AnalogWatchdog_SingleRegEnable);
  } else {
    ADC_AnalogWatchdogCmd(adc, ADC_AnalogWatchdog_None);
  }

  /* configure the interrupts to be generated by the ADC */
  ADC_ITConfig(adc, ADC_IT_EOC, (awd_enable == BLT_TRUE) ? DISABLE : ENABLE);
  ADC_ITConfig(adc, ADC_IT_AWD, (awd_enable == BLT_TRUE) ? ENABLE : DISABLE);
  ADC_ITConfig(adc, ADC_IT_JEOC, DISABLE);
  ADC_ITConfig(adc, ADC_IT_OVR, DISABLE);

  return adc;
}

/*
 * Callback function that handles the system shutdown and enters transportation mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * In transportation low-power mode the system can only be woken up by pulling down the NRST signal.
 * Furthermore, the system can not be charged when in transportation mode.
 */
void blCallbackShutdownTransportation(void) {
  /* make sure that the required clocks are activated */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* initialized the standalone timer */
  saTimerInit();

  setLed(BLT_TRUE);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  if (GPIO_ReadOutputDataBit(SYS_REG_EN_GPIO, SYS_REG_EN_PIN) == Bit_SET) {
    // this must skipped if the pullup voltage (VIO3.3) is not active
    setLed(BLT_TRUE);
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    setLed(BLT_FALSE);
  }

  /* execute disambiguation procedure and signal all modules to enter transportation mode */
  if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_TRANSPORT) != SUCCESS) {
    blinkSOS(1);
    msleep(10);
  }

  shutdownToTransportation();

  return;
} /*** end of bLCallbackTransportation ***/

/*
 * Callback function that handles the system shutdown and enters deepsleep mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * In deepsleep low-power mode the system can only be woken up via the NRST or the WKUP signal, or the RTC or IWDG, if configured.
 * When a power plug is detected, the system will switch to hibernate mode, to provide charging capabilities (cf. handlePathDcWakeup()).
 * As soon as the plug is removed again, however, the system will return to deppsleep mode (cf. handleIwdgWakeup()).
 */
void blCallbackShutdownDeepsleep(void) {
  /* make sure that the required clocks are activated */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* initialized the standalone timer */
  saTimerInit();

  setLed(BLT_TRUE);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  if (GPIO_ReadOutputDataBit(SYS_REG_EN_GPIO, SYS_REG_EN_PIN) == Bit_SET) {
    // this must skipped if the pullup voltage (VIO3.3) is not active
    setLed(BLT_TRUE);
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    setLed(BLT_FALSE);
  }

  /* execute disambiguation procedure and signal all modules to enter deepsleep mode */
  if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_DEEPSLEEP) != SUCCESS) {
    blinkSOS(1);
    msleep(10);
  }

  shutdownToDeepsleep();

  return;
} /*** end of bLCallbackDeepsleep ***/

/*
 * Callback function that handles the system shutdown and enters hibernate mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * Since this function actually just configures the system in a way, that it will enter hibernate mode after the next reset and rests it,
 * see the handleSoftwareReset() function for more details about the hibernate low-power mode.
 */
void blCallbackShutdownHibernate(void) {
  /* make sure that the required clocks are activated */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* initialized the standalone timer */
  saTimerInit();

  setLed(BLT_TRUE);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  if (GPIO_ReadOutputDataBit(SYS_REG_EN_GPIO, SYS_REG_EN_PIN) == Bit_SET) {
    // this must skipped if the pullup voltage (VIO3.3) is not active
    setLed(BLT_TRUE);
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    setLed(BLT_FALSE);
  }

  /* execute disambiguation procedure and signal all modules to enter hibernate mode */
  if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_DEEPSLEEP) != SUCCESS) {
    blinkSOS(1);
    msleep(10);
  }

  shutdownToHibernate();

  return;
} /*** end of bLCallbackShutdownHibernate ***/

/*
 * Callback function that handles the system shutdown and initializes a restart.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * By configuration it is ensured, that the system will end up executing the handleSoftwareReset() function after reset.
 */
void blCallbackShutdownRestart(void) {
  /* make sure that the required clocks are activated */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signal active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* initialized the standalone timer */
  saTimerInit();

  setLed(BLT_TRUE);

  /* deactivate SYS_PD_N and ensure that all modules had a chance to detect the falling edge */
  msleep(1);
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  msleep(1);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  if (GPIO_ReadOutputDataBit(SYS_REG_EN_GPIO, SYS_REG_EN_PIN) == Bit_SET) {
    // this must skipped if the pullup voltage (VIO3.3) is not active
    setLed(BLT_TRUE);
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    setLed(BLT_FALSE);
  }

  /* execute disambiguation procedure and signal all modules to restart normally */
  if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_RESTART) != SUCCESS) {
    blinkSOS(1);
    msleep(10);
  }

  /* restart the system */
  shutdownAndRestart();

  return;
} /*** end of bLCallbackRestart ***/


/*
 * Callback function that handles a system shutdown/restart request from another module.
 * Depending on the result of the disambiguation procedure, the module will enter the according low-power mode or restart.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 */
void blCallbackHandleShutdownRequest(void) {
  /* make sure that the required clocks are activated */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signal active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* initialized the standalone timer */
  saTimerInit();

  setLed(BLT_TRUE);

  /* deactivate SYS_PD_N and ensure that all modules had a chance to detect the falling edge */
  msleep(1);
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  msleep(1);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  if (GPIO_ReadOutputDataBit(SYS_REG_EN_GPIO, SYS_REG_EN_PIN) == Bit_SET) {
    // this must skipped if the pullup voltage (VIO3.3) is not active
    setLed(BLT_TRUE);
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    setLed(BLT_FALSE);
  }

  /* check ths SYS_PD_N signal, whether the system shall shutdown or restart */
  blt_bool shutdown_nrestart = (GPIO_ReadInputDataBit(SYS_PD_N_GPIO, SYS_PD_N_PIN) == Bit_RESET) ? BLT_TRUE : BLT_FALSE;

  /* disambiguation procedure (passive) */
  uint32_t pulse_counter = 0;
  while (waitForSignalTimeout(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_RESET, 10)) {
    waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
    ++pulse_counter;
  }

  /* evaluate and hanlde disambiguation result */
  if (shutdown_nrestart == BLT_TRUE) {
    /* shutdown request */

    /* handle special cases */
    if (pulse_counter == BL_SHUTDOWN_PRI_RSN_UNKNOWN) {
      /* no pulse at all was received */
      pulse_counter = BL_SHUTDOWN_PRI_RSN_DEFAULT;
    } else if (pulse_counter != BL_SHUTDOWN_PRI_RSN_HIBERNATE &&
               pulse_counter != BL_SHUTDOWN_PRI_RSN_DEEPSLEEP &&
               pulse_counter != BL_SHUTDOWN_PRI_RSN_TRANSPORT) {
      /* invalid number of pulses received */
      blinkSOS(1);
      pulse_counter = BL_SHUTDOWN_PRI_RSN_DEFAULT;
    }

    switch (pulse_counter) {
      case BL_SHUTDOWN_PRI_RSN_HIBERNATE:
        shutdownToHibernate();
        break;
      case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
        shutdownToDeepsleep();
        break;
      case BL_SHUTDOWN_PRI_RSN_TRANSPORT:
        shutdownToTransportation();
        break;
    }
  } else {
    /* restart request */

    /* there is no ambiguity for restart, so it is ignored */
    shutdownAndRestart();
  }

  /* if this code is reached, the system did neither shut down, nor restart.
   * This must never be the case!
   */
  blinkSOSinf();
  return;
} /*** end of blCallbackHandleShutdownRequest ***/


/*********************************** end of main.c *************************************/
