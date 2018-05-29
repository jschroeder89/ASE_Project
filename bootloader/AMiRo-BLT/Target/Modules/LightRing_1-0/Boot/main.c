/************************************************************************************//**
* \file         Demo\ARMCM3_STM32_Olimex_STM32P103_GCC\Boot\main.c
* \brief        Bootloader application source file.
* \ingroup      Boot_ARMCM3_STM32_Olimex_STM32P103_GCC
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2012  by Feaser    http://www.feaser.com    All rights reserved
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
#include "timer.h"
#include "ARMCM3_STM32/types.h"
#include "AMiRo/amiroblt.h"
#include "helper.h"
#include "iodef.h"

/****************************************************************************************
* Defines
****************************************************************************************/

/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void Init(void);

static void initGpio();
static void initExti();
void configGpioForShutdown();

ErrorStatus handleWarmReset();

ErrorStatus shutdownDisambiguationProcedure(const uint8_t type);
void shutdownToTransportation(const blt_bool exec_disambiguation);
void shutdownToDeepsleep(const blt_bool exec_disambiguation);
void shutdownToHibernate(const blt_bool exec_disambiguation);
void shutdownAndRestart(const blt_bool exec_disambiguation);

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
** \return    Program return code.
**
****************************************************************************************/
int main(void)
{
  /* initialize the microcontroller */
  Init();

  /* activate some required cocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* initialize GPIOs and EXTI lines */
  initGpio();
  setLed(BLT_TRUE);
  initExti();

  /* initialize the timer */
  TimerInit();

  /* detect the primary reason for this wakeup/restart */
  backup_reg.wakeup_pri_reason =
      ((RCC_GetFlagStatus(RCC_FLAG_LPWRRST) == SET) ? BL_WAKEUP_PRI_RSN_LPWRRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_WWDGRST) == SET) ? BL_WAKEUP_PRI_RSN_WWDGRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_IWDGRST) == SET) ? BL_WAKEUP_PRI_RSN_IWDGRST : 0) |
      ((RCC_GetFlagStatus(RCC_FLAG_SFTRST) == SET) ? BL_WAKEUP_PRI_RSN_SFTRST : 0)   |
      ((RCC_GetFlagStatus(RCC_FLAG_PORRST) == SET) ? BL_WAKEUP_PRI_RSN_PORRST : 0)   |
      ((RCC_GetFlagStatus(RCC_FLAG_PINRST) == SET) ? BL_WAKEUP_PRI_RSN_PINRST : 0)   |
      ((PWR_GetFlagStatus(PWR_FLAG_WU) == SET) ? BL_WAKEUP_PRI_RSN_WKUP : 0);
  /* for this module there is no secondary wakeup reason */
  backup_reg.wakeup_sec_reason = BL_WAKEUP_SEC_RSN_UNKNOWN;

  /* clear the flags */
  RCC_ClearFlag();
  PWR_ClearFlag(PWR_FLAG_WU);

  setLed(BLT_FALSE);

  /* hanlde different wakeup/reset reasons */
  ErrorStatus status = ERROR;
  if (backup_reg.wakeup_pri_reason & BL_WAKEUP_PRI_RSN_PINRST) {
    /* system was woken via NRST pin */
    status = handleWarmReset();
  } else {
    /* system was woken/reset for an unexpected reason */
    blinkSOS(1);
    status = handleWarmReset();
  }

  /* if something went wrong, signal this failure */
  if (status != SUCCESS) {
    blinkSOSinf();
  }

  return 0;
} /*** end of main ***/


/************************************************************************************//**
** \brief     Initializes the microcontroller.
** \return    none.
**
****************************************************************************************/
static void Init(void)
{
  volatile blt_int32u StartUpCounter = 0, HSEStatus = 0;
  blt_int32u pll_multiplier;
#if (BOOT_FILE_LOGGING_ENABLE > 0) && (BOOT_COM_UART_ENABLE == 0) && (BOOT_GATE_UART_ENABLE == 0)
  GPIO_InitTypeDef  GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;
#endif

  /* reset the RCC clock configuration to the default reset state (for debug purpose) */
  /* set HSION bit */
  RCC->CR |= (blt_int32u)0x00000001;
  /* reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
  RCC->CFGR &= (blt_int32u)0xF8FF0000;
  /* reset HSEON, CSSON and PLLON bits */
  RCC->CR &= (blt_int32u)0xFEF6FFFF;
  /* reset HSEBYP bit */
  RCC->CR &= (blt_int32u)0xFFFBFFFF;
  /* reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
  RCC->CFGR &= (blt_int32u)0xFF80FFFF;
  /* disable all interrupts and clear pending bits  */
  RCC->CIR = 0x009F0000;
  /* enable HSE */
  RCC->CR |= ((blt_int32u)RCC_CR_HSEON);
  /* wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  }
  while((HSEStatus == 0) && (StartUpCounter != 1500));
  /* check if time out was reached */
  if ((RCC->CR & RCC_CR_HSERDY) == RESET)
  {
    /* cannot continue when HSE is not ready */
    ASSERT_RT(BLT_FALSE);
  }
  /* enable flash prefetch buffer */
  FLASH->ACR |= FLASH_ACR_PRFTBE;
  /* reset flash wait state configuration to default 0 wait states */
  FLASH->ACR &= (blt_int32u)((blt_int32u)~FLASH_ACR_LATENCY);
#if (BOOT_CPU_SYSTEM_SPEED_KHZ > 48000)
  /* configure 2 flash wait states */
  FLASH->ACR |= (blt_int32u)FLASH_ACR_LATENCY_2;
#elif (BOOT_CPU_SYSTEM_SPEED_KHZ > 24000)
  /* configure 1 flash wait states */
  FLASH->ACR |= (blt_int32u)FLASH_ACR_LATENCY_1;
#endif
  /* HCLK = SYSCLK */
  RCC->CFGR |= (blt_int32u)RCC_CFGR_HPRE_DIV1;
  /* PCLK2 = HCLK/2 */
  RCC->CFGR |= (blt_int32u)RCC_CFGR_PPRE2_DIV2;
  /* PCLK1 = HCLK/2 */
  RCC->CFGR |= (blt_int32u)RCC_CFGR_PPRE1_DIV2;
  /* reset PLL configuration */
  RCC->CFGR &= (blt_int32u)((blt_int32u)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | \
                                          RCC_CFGR_PLLMULL));
  /* assert that the pll_multiplier is between 2 and 16 */
  ASSERT_CT((BOOT_CPU_SYSTEM_SPEED_KHZ/BOOT_CPU_XTAL_SPEED_KHZ) >= 2);
  ASSERT_CT((BOOT_CPU_SYSTEM_SPEED_KHZ/BOOT_CPU_XTAL_SPEED_KHZ) <= 16);
  /* calculate multiplier value */
  pll_multiplier = BOOT_CPU_SYSTEM_SPEED_KHZ/BOOT_CPU_XTAL_SPEED_KHZ;
  /* convert to register value */
  pll_multiplier = (blt_int32u)((pll_multiplier - 2) << 18);
  /* set the PLL multiplier and clock source */
  RCC->CFGR |= (blt_int32u)(RCC_CFGR_PLLSRC_HSE | pll_multiplier);
  /* enable PLL */
  RCC->CR |= RCC_CR_PLLON;
  /* wait till PLL is ready */
  while((RCC->CR & RCC_CR_PLLRDY) == 0)
  {
  }
  /* select PLL as system clock source */
  RCC->CFGR &= (blt_int32u)((blt_int32u)~(RCC_CFGR_SW));
  RCC->CFGR |= (blt_int32u)RCC_CFGR_SW_PLL;
  /* wait till PLL is used as system clock source */
  while ((RCC->CFGR & (blt_int32u)RCC_CFGR_SWS) != (blt_int32u)0x08)
  {
  }
#if (BOOT_COM_CAN_ENABLE > 0 || BOOT_GATE_CAN_ENABLE > 0)
  /* enable clocks for CAN transmitter and receiver pins (GPIOB and AFIO) */
  RCC->APB2ENR |= (blt_int32u)(0x00000004 | 0x00000001);
  /* configure CAN Rx (GPIOA11) as alternate function input */
  /* first reset the configuration */
  GPIOA->CRH &= ~(blt_int32u)((blt_int32u)0xf << 12);
  /* CNF8[1:0] = %01 and MODE8[1:0] = %00 */
  GPIOA->CRH |= (blt_int32u)((blt_int32u)0x4 << 12);
  /* configure CAN Tx (GPIOA12) as alternate function push-pull */
  /* first reset the configuration */
  GPIOA->CRH &= ~(blt_int32u)((blt_int32u)0xf << 16);
  /* CNF9[1:0] = %11 and MODE9[1:0] = %11 */
  GPIOA->CRH |= (blt_int32u)((blt_int32u)0xb << 16);

  /* remap CAN1 pins to PortA */
  AFIO->MAPR &= ~(blt_int32u)((blt_int32u)0x3 << 13);
  AFIO->MAPR |=  (blt_int32u)((blt_int32u)0x0 << 13);
  /* enable clocks for CAN controller peripheral */
  RCC->APB1ENR |= (blt_int32u)0x02000000;
#endif
#if (BOOT_COM_UART_ENABLE > 0 || BOOT_GATE_UART_ENABLE > 0)
  /* enable clocks for USART1 peripheral, transmitter and receiver pins (GPIOA and AFIO) */
  RCC->APB2ENR |= (blt_int32u)(0x00004000 | 0x00000004 | 0x00000001);
  /* configure USART1 Tx (GPIOA9) as alternate function push-pull */
  /* first reset the configuration */
  GPIOA->CRH &= ~(blt_int32u)((blt_int32u)0xf << 4);
  /* CNF2[1:0] = %10 and MODE2[1:0] = %11 */
  GPIOA->CRH |= (blt_int32u)((blt_int32u)0xb << 4);
  /* configure USART1 Rx (GPIOA10) as alternate function input floating */
  /* first reset the configuration */
  GPIOA->CRH &= ~(blt_int32u)((blt_int32u)0xf << 8);
  /* CNF2[1:0] = %01 and MODE2[1:0] = %00 */
  GPIOA->CRH |= (blt_int32u)((blt_int32u)0x4 << 8);

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
  /* enable clocks for USART2 peripheral */
  RCC->APB1ENR |= (blt_int32u)(0x00020000);
  /* configure USART2 Tx (GPIOA2) as alternate function push-pull */
  /* first reset the configuration */
  GPIOA->CRL &= ~(blt_int32u)((blt_int32u)0xf << 8);
  /* CNF2[1:0] = %10 and MODE2[1:0] = %11 */
  GPIOA->CRL |= (blt_int32u)((blt_int32u)0xb << 8);
  /* configure USART2 Rx (GPIOA3) as alternate function input floating */
  /* first reset the configuration */
  GPIOA->CRL &= ~(blt_int32u)((blt_int32u)0xf << 12);
  /* CNF2[1:0] = %01 and MODE2[1:0] = %00 */
  GPIOA->CRL |= (blt_int32u)((blt_int32u)0x4 << 12);
#endif

#elif (BOOT_FILE_LOGGING_ENABLE > 0)
  /* enable UART peripheral clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  /* enable GPIO peripheral clock for transmitter and receiver pins */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  /* configure USART Tx as alternate function push-pull */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* Configure USART Rx as alternate function input floating */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* configure UART communcation parameters */
  USART_InitStruct.USART_BaudRate = BOOT_COM_UART_BAUDRATE;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStruct);
  /* enable UART */
  USART_Cmd(USART2, ENABLE);
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

  /* initialize the pseudo LED and push it up (inactive) */
  GPIO_SetBits(PSEUDO_LED_GPIO, PSEUDO_LED_PIN);
  gpio_init.GPIO_Pin    = PSEUDO_LED_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_Out_PP;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(PSEUDO_LED_GPIO, &gpio_init);

  /* initialize SYS_PD_N and let it go (inactive) */
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  gpio_init.GPIO_Pin    = SYS_PD_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_Out_OD;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(SYS_PD_N_GPIO, &gpio_init);

  /* initialize SYS_SYNC_N and pull it down (active) */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  gpio_init.GPIO_Pin    = SYS_SYNC_N_PIN;
  gpio_init.GPIO_Mode   = GPIO_Mode_Out_OD;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;
  GPIO_Init(SYS_SYNC_N_GPIO, &gpio_init);

  /*
   * INPUTS
   */

} /*** end of initGpio ***/

/*
 * Initialize all EXTI lines
 */
static void initExti() {
  /* configure EXTI lines */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource2); // SYS_SYNC_N
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5); // LASER_OC_N
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6); // SYS_UART_DN
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8); // WL_GDO2
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9); // WL_GDO0
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource14); // SYS_PD_N

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
void shutdownToTransportation(const blt_bool exec_disambiguation) {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  // this must no be skipped, since the pull-up voltage for SYS_SYNC_N (VIO3.3) must be configured at this point by definition.
  setLed(BLT_TRUE);
  waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
  setLed(BLT_FALSE);

  if (exec_disambiguation == BLT_TRUE) {
    /* execute disambiguation procedure and signal all modules to enter transportation mode */
    if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_TRANSPORT) != SUCCESS) {
      blinkSOS(1);
      msleep(10);
    }
  }

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode */
  PWR_EnterSTANDBYMode();

  return;
} /*** end of shutdownToTransportation ***/

/*
 * Final shutdown of the system to enter deepsleep mode.
 */
void shutdownToDeepsleep(const blt_bool exec_disambiguation) {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  // this must no be skipped, since the pull-up voltage for SYS_SYNC_N (VIO3.3) must be configured at this point by definition.
  setLed(BLT_TRUE);
  waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
  setLed(BLT_FALSE);

  if (exec_disambiguation == BLT_TRUE) {
    /* execute disambiguation procedure and signal all modules to enter deepsleep mode */
    if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_DEEPSLEEP) != SUCCESS) {
      blinkSOS(1);
      msleep(10);
    }
  }

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode */
  PWR_EnterSTANDBYMode();

  return;
} /*** end of shutdownToDeepsleep ***/

/*
 * Final shutdown of the system to enter hibernate mode.
 */
void shutdownToHibernate(const blt_bool exec_disambiguation) {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  // this must no be skipped, since the pull-up voltage for SYS_SYNC_N (VIO3.3) must be configured at this point by definition.
  setLed(BLT_TRUE);
  waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
  setLed(BLT_FALSE);

  if (exec_disambiguation == BLT_TRUE) {
    /* execute disambiguation procedure and signal all modules to enter hibernate mode */
    if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_HIBERNATE) != SUCCESS) {
      blinkSOS(1);
      msleep(10);
    }
  }

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode */
  PWR_EnterSTANDBYMode();

  return;
} /*** end of shutdownToHibernate ***/

void shutdownAndRestart(const blt_bool exec_disambiguation) {
  /* configure some criticpal GPIOs as input
   * This is required, because otherwise some hardware might be powered through these signals */
  configGpioForShutdown();

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  // this must no be skipped, since the pull-up voltage for SYS_SYNC_N (VIO3.3) must be configured at this point by definition.
  setLed(BLT_TRUE);
  waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
  setLed(BLT_FALSE);

  if (exec_disambiguation == BLT_TRUE) {
    /* execute disambiguation procedure and signal all modules to restart in default mode */
    if (shutdownDisambiguationProcedure(BL_SHUTDOWN_PRI_RSN_RESTART) != SUCCESS) {
      blinkSOS(1);
      msleep(10);
    }
  }

  /* morse 'OK' via the LED to signal that shutdown was successful */
  blinkOK(1);

  /* enter standby mode */
  PWR_EnterSTANDBYMode();

  /*
   * Even though this module will not restart the system by its own, the PowerManagement will reset the system.
   */

  return;
}

/*
 * Configures some GPIO pins as inputs for safety reasons.
 * Under certain circumstances, these pins might power hardware that is supposed to be shut down.
 */
void configGpioForShutdown() {
  /* setup the configuration */
  GPIO_InitTypeDef gpio_init;
  gpio_init.GPIO_Mode   = GPIO_Mode_IN_FLOATING;
  gpio_init.GPIO_Speed  = GPIO_Speed_50MHz;

  /* configure SYS_UART_TX */
  gpio_init.GPIO_Pin = SYS_UART_TX_PIN;
  GPIO_Init(SYS_UART_TX_GPIO, &gpio_init);

  /* configure CAN_TX */
  gpio_init.GPIO_Pin = CAN_TX_PIN;
  GPIO_Init(CAN_TX_GPIO, &gpio_init);

  /* configure all LASER signals */
  gpio_init.GPIO_Pin = LASER_EN_PIN;
  GPIO_Init(LASER_EN_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = LASER_TX_PIN;
  GPIO_Init(LASER_TX_GPIO, &gpio_init);

  /* configure all LIGHT (SPI) signals */
  gpio_init.GPIO_Pin = LIGHT_SCLK_PIN;
  GPIO_Init(LIGHT_SCLK_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = LIGHT_MOSI_PIN;
  GPIO_Init(LIGHT_MOSI_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = LIGHT_XLAT_PIN;
  GPIO_Init(LIGHT_XLAT_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = LIGHT_BLANK_PIN;
  GPIO_Init(LIGHT_BLANK_GPIO, &gpio_init);

  /* configure all WL (SPI) signals */
  gpio_init.GPIO_Pin = WL_SS_N_PIN;
  GPIO_Init(WL_SS_N_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = WL_SCLK_PIN;
  GPIO_Init(WL_SCLK_GPIO, &gpio_init);
  gpio_init.GPIO_Pin = WL_MOSI_PIN;
  GPIO_Init(WL_MOSI_GPIO, &gpio_init);

  return;
} /*** end of configGpioForShutdown ***/

/*
 * System was reset via the NRST pin or the reason could not be detected.
 * In this case, the system enters the boot loop and starts the OS.
 */
ErrorStatus handleWarmReset() {
  /* initialize the bootloader */
  BootInit();

  /* start the infinite program loop */
  uint32_t loopStartTime = 0;
  saTimerUpdate(&loopStartTime);
  uint32_t currentTime = loopStartTime;
  while (1)
  {
//    /* make the pseudo LED "double-blink" */
//    saTimerUpdate(&currentTime);
//    if (currentTime < loopStartTime + 50) {
//      setLed(BLT_TRUE);
//    } else if (currentTime < loopStartTime + 50+100) {
//      setLed(BLT_FALSE);
//    } else if (currentTime < loopStartTime + 50+100+50) {
//      setLed(BLT_TRUE);
//    } else if ( currentTime < loopStartTime + 50+100+50+300) {
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
} /*** end of handleWarmReset ***/

/*
 * Callback function that handles the system shutdown and enters transportation mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * In transportation low-power mode the system can only be woken up by pulling down the NRST signal.
 * Furthermore, the system can not be charged when in transportation mode.
 */
void blCallbackShutdownTransportation() {
  /* make sure that the required clocks are activated */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  saTimerInit();

  setLed(BLT_TRUE);

  shutdownToTransportation(BLT_TRUE);

  return;
} /*** end of blCallbackShutdownTransportation ***/

/*
 * Callback function that handles the system shutdown and enters deepsleep mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 * In deepsleep low-power mode the system can only be woken up via the NRST or the WKUP signal, or the RTC or IWDG, if configured.
 */
void blCallbackShutdownDeepsleep(void) {
  /* make sure that the required clocks are activated */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  saTimerInit();

  setLed(BLT_TRUE);

  shutdownToDeepsleep(BLT_TRUE);

  return;
} /*** end of blCallbackShutdownDeepsleep ***/

/*
 * Callback function that handles the system shutdown and enters hibernate mode.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 */
void blCallbackShutdownHibernate(void) {
  /* make sure that the required clocks are activated */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  saTimerInit();

  setLed(BLT_TRUE);

  shutdownToHibernate(BLT_TRUE);

  return;
} /*** end of blCallbackShutdownHibernate ***/

/*
 * Callback function that handles the system shutdown and initializes a restart.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 */
void blCallbackShutdownRestart(void) {
  /* make sure that the required clocks are activated */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* ensure that all modules had a chance to detect the pulse on SYS_PD_N */
  saTimerInit();
  msleep(1);
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  msleep(1);

  shutdownAndRestart(BLT_TRUE);

  return;
} /** end of blCallbackShutdownRestart ***/

/*
 * Callback function that handles a system shutdown/restart request from another module.
 * Depending on the result of the disambiguation procedure, the module will enter the according low-power mode or restart.
 * When called from a multithreaded environment, it must be ensured that no other thread will preempt this function.
 */
void blCallbackHandleShutdownRequest(void) {
  /* make sure that the required clocks are activated */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);

  /* set/keep the SYS_SYNC and SYS_PD signals active */
  GPIO_ResetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  GPIO_ResetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);

  /* ensure that all modules had a chance to detect the pulse on SYS_PD_N */
  saTimerInit();
  msleep(1);
  GPIO_SetBits(SYS_PD_N_GPIO, SYS_PD_N_PIN);
  msleep(1);

  /* wait for all boards to be ready for shutdown */
  GPIO_SetBits(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN);
  // this must not be skipped, since the pull-up voltage for SYS_SYNC_N (VIO3.3) must be configured at this point by definition.
  setLed(BLT_TRUE);
  waitForSignal(SYS_SYNC_N_GPIO, SYS_SYNC_N_PIN, Bit_SET);
  setLed(BLT_FALSE);

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
        shutdownToHibernate(BLT_FALSE);
        break;
      case BL_SHUTDOWN_PRI_RSN_DEEPSLEEP:
        shutdownToDeepsleep(BLT_FALSE);
        break;
      case BL_SHUTDOWN_PRI_RSN_TRANSPORT:
        shutdownToTransportation(BLT_FALSE);
        break;
    }
  } else {
    /* restart request */

    /* there is no ambiguity for restart, so it is ignored */
    shutdownAndRestart(BLT_FALSE);
  }

  /* if this code is reached, the system did neither shut down, nor restart.
   * This must never be the case!
   */
  blinkSOSinf();
  return;
} /*** end of blCallbackHandleShutdownRequest ***/

/*********************************** end of main.c *************************************/
