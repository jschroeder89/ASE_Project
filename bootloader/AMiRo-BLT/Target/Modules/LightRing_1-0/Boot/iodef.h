#ifndef IODEF_H
#define IODEF_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define PSEUDO_LED_GPIO     GPIOA
#define PSEUDO_LED_PIN      GPIO_Pin_1
#define LASER_RX_GPIO       GPIOA
#define LASER_RX_PIN        GPIO_Pin_2
#define LASER_TX_GPIO       GPIOA
#define LASER_TX_PIN        GPIO_Pin_3
#define LIGHT_BLANK_GPIO    GPIOA
#define LIGHT_BLANK_PIN     GPIO_Pin_4
#define LIGHT_SCLK_GPIO     GPIOA
#define LIGHT_SCLK_PIN      GPIO_Pin_5
#define LIGHT_MOSI_GPIO     GPIOA
#define LIGHT_MOSI_PIN      GPIO_Pin_7
#define PROG_RX_GPIO        GPIOA
#define PROG_RX_PIN         GPIO_Pin_9
#define PROG_TX_GPIO        GPIOA
#define PROG_TX_PIN         GPIO_Pin_10
#define CAN_RX_GPIO         GPIOA
#define CAN_RX_PIN          GPIO_Pin_11
#define CAN_TX_GPIO         GPIOA
#define CAN_TX_PIN          GPIO_Pin_12
#define SWDIO_GPIO          GPIOA
#define SWDIO_PIN           GPIO_Pin_13
#define SWCLK_GPIO          GPIOA
#define SWCLK_PIN           GPIO_Pin_14

#define LASER_EN_GPIO       GPIOB
#define LASER_EN_PIN        GPIO_Pin_2
#define LASER_OC_N_GPIO     GPIOB
#define LASER_OC_N_PIN      GPIO_Pin_5
#define SYS_UART_DN_GPIO    GPIOB
#define SYS_UART_DN_PIN     GPIO_Pin_6
#define WL_GDO2_GPIO        GPIOB
#define WL_GDO2_PIN         GPIO_Pin_8
#define WL_GDO0_GPIO        GPIOB
#define WL_GDO0_PIN         GPIO_Pin_9
#define MEM_SCL_GPIO        GPIOB
#define MEM_SCL_PIN         GPIO_Pin_10
#define MEM_SDA_GPIO        GPIOB
#define MEM_SDA_PIN         GPIO_Pin_11
#define WL_SS_N_GPIO        GPIOB
#define WL_SS_N_PIN         GPIO_Pin_12
#define WL_SCLK_GPIO        GPIOB
#define WL_SCLK_PIN         GPIO_Pin_13
#define WL_MISO_GPIO        GPIOB
#define WL_MISO_PIN         GPIO_Pin_14
#define WL_MOSI_GPIO        GPIOB
#define WL_MOSI_PIN         GPIO_Pin_15

#define LIGHT_XLAT_GPIO     GPIOC
#define LIGHT_XLAT_PIN      GPIO_Pin_4
#define SYS_UART_RX_GPIO    GPIOC
#define SYS_UART_RX_PIN     GPIO_Pin_10
#define SYS_UART_TX_GPIO    GPIOC
#define SYS_UART_TX_PIN     GPIO_Pin_11
#define SYS_PD_N_GPIO       GPIOC
#define SYS_PD_N_PIN        GPIO_Pin_14

#define OSC_IN_GPIO         GPIOD
#define OSC_IN_PIN          GPIO_Pin_0
#define OSC_OUT_GPIO        GPIOD
#define OSC_OUT_PIN         GPIO_Pin_1
#define SYS_SYNC_N_GPIO     GPIOD
#define SYS_SYNC_N_PIN      GPIO_Pin_2

#endif /* IODEF_H */



