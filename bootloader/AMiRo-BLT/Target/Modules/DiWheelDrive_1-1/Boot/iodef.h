#ifndef IODEF_H
#define IODEF_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#define WKUP_GPIO           GPIOA
#define WKUP_PIN            GPIO_Pin_0
#define LED_GPIO            GPIOA
#define LED_PIN             GPIO_Pin_1
#define DRIVE_PWM1A_GPIO    GPIOA
#define DRIVE_PWM1A_PIN     GPIO_Pin_2
#define DRIVE_PWM1B_GPIO    GPIOA
#define DRIVE_PWM1B_PIN     GPIO_Pin_3
#define MOTION_SCLK_GPIO    GPIOA
#define MOTION_SCLK_PIN     GPIO_Pin_5
#define MOTION_MISO_GPIO    GPIOA
#define MOTION_MISO_PIN     GPIO_Pin_6
#define MOTION_MOSI_GPIO    GPIOA
#define MOTION_MOSI_PIN     GPIO_Pin_7
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
#define DRIVE_PWM2B_GPIO    GPIOA
#define DRIVE_PWM2B_PIN     GPIO_Pin_15

#define DRIVE_SENSE2_GPIO   GPIOB
#define DRIVE_SENSE2_PIN    GPIO_Pin_1
#define POWER_EN_GPIO       GPIOB
#define POWER_EN_PIN        GPIO_Pin_2
#define DRIVE_PWM2A_GPIO    GPIOB
#define DRIVE_PWM2A_PIN     GPIO_Pin_3
#define COMPASS_DRDY_GPIO   GPIOB
#define COMPASS_DRDY_PIN    GPIO_Pin_5
#define DRIVE_ENC1A_GPIO    GPIOB
#define DRIVE_ENC1A_PIN     GPIO_Pin_6
#define DRIVE_ENC1B_GPIO    GPIOB
#define DRIVE_ENC1B_PIN     GPIO_Pin_7
#define COMPASS_SCL_GPIO    GPIOB
#define COMPASS_SCL_PIN     GPIO_Pin_8
#define COMPASS_SDA_GPIO    GPIOB
#define COMPASS_SDA_PIN     GPIO_Pin_9
#define IR_SCL_GPIO         GPIOB
#define IR_SCL_PIN          GPIO_Pin_10
#define IR_SDA_GPIO         GPIOB
#define IR_SDA_PIN          GPIO_Pin_11
#define IR_INT_GPIO         GPIOB
#define IR_INT_PIN          GPIO_Pin_12
#define GYRP_DRDY_GPIO      GPIOB
#define GYRO_DRDY_PIN       GPIO_Pin_13
#define SYS_UART_UP_GPIO    GPIOB
#define SYS_UART_UP_PIN     GPIO_Pin_14
#define ACCEL_INT_N_GPIO    GPIOB
#define ACCEL_INT_N_PIN     GPIO_Pin_15

#define DRIVE_SENSE1_GPIO   GPIOC
#define DRIVE_SENSE1_PIN    GPIO_Pin_0
#define SYS_SYNC_N_GPIO     GPIOC
#define SYS_SYNC_N_PIN      GPIO_Pin_1
#define PATH_DCSTAT_GPIO    GPIOC
#define PATH_DCSTAT_PIN     GPIO_Pin_3
#define PATH_DCEN_GPIO      GPIOC
#define PATH_DCEN_PIN       GPIO_Pin_5
#define DRIVE_ENC2B_GPIO    GPIOC
#define DRIVE_ENC2B_PIN     GPIO_Pin_6
#define DRIVE_ENC2A_GPIO    GPIOC
#define DRIVE_ENC2A_PIN     GPIO_Pin_7
#define SYS_PD_N_GPIO       GPIOC
#define SYS_PD_N_PIN        GPIO_Pin_8
#define SYS_REG_EN_GPIO     GPIOC
#define SYS_REG_EN_PIN      GPIO_Pin_9
#define SYS_UART_RX_GPIO    GPIOC
#define SYS_UART_RX_PIN     GPIO_Pin_10
#define SYS_UART_TX_GPIO    GPIOC
#define SYS_UART_TX_PIN     GPIO_Pin_11
#define ACCEL_SS_N_GPIO     GPIOC
#define ACCEL_SS_N_PIN      GPIO_Pin_13
#define GYRO_SS_N_GPIO      GPIOC
#define GYRO_SS_N_PIN       GPIO_Pin_14

#define OSC_IN_GPIO         GPIOD
#define OSC_IN_PIN          GPIO_Pin_0
#define OSC_OUT_GPIO        GPIOD
#define OSC_OUT_PIN         GPIO_Pin_1
#define SYS_WARMRST_N_GPIO  GPIOD
#define SYS_WARMRST_N_PIN   GPIO_Pin_2

#endif /* IODEF_H */
