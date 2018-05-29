#ifndef IODEF_H
#define IODEF_H

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define WKUP_GPIO               GPIOA
#define WKUP_PIN                GPIO_Pin_0
#define SYS_UART_TX_GPIO        GPIOA
#define SYS_UART_TX_PIN         GPIO_Pin_2
#define SYS_UART_RX_GPIO        GPIOA
#define SYS_UART_RX_PIN         GPIO_Pin_3
#define SYS_SPI_SS0_N_GPIO      GPIOA
#define SYS_SPI_SS0_N_PIN       GPIO_Pin_4
#define SYS_SPI_SCLK_GPIO       GPIOA
#define SYS_SPI_SCLK_PIN        GPIO_Pin_5
#define SYS_SPI_MISO_GPIO       GPIOA
#define SYS_SPI_MISO_PIN        GPIO_Pin_6
#define SYS_SPI_MOSI_GPIO       GPIOA
#define SYS_SPI_MOSI_PIN        GPIO_Pin_7
#define SYS_REG_EN_GPIO         GPIOA
#define SYS_REG_EN_PIN          GPIO_Pin_8
#define PROG_RX_GPIO            GPIOA
#define PROG_RX_PIN             GPIO_Pin_9
#define PROG_TX_GPIO            GPIOA
#define PROG_TX_PIN             GPIO_Pin_10
#define CAN_RX_GPIO             GPIOA
#define CAN_RX_PIN              GPIO_Pin_11
#define CAN_TX_GPIO             GPIOA
#define CAN_TX_PIN              GPIO_Pin_12
#define SWDIO_GPIO              GPIOA
#define SWDIO_PIN               GPIO_Pin_13
#define SWCLK_GPIO              GPIOA
#define SWCLK_PIN               GPIO_Pin_14
#define SYS_SPI_SS1_N_GPIO      GPIOA
#define SYS_SPI_SS1_N_PIN       GPIO_Pin_15

#define IR_INT1_N_GPIO          GPIOB
#define IR_INT1_N_PIN           GPIO_Pin_0
#define VSYS_SENSE_GPIO         GPIOB
#define VSYS_SENSE_PIN          GPIO_Pin_1
#define POWER_EN_GPIO           GPIOB
#define POWER_EN_PIN            GPIO_Pin_2
#define SYS_UART_DN_GPIO        GPIOB
#define SYS_UART_DN_PIN         GPIO_Pin_3
#define CHARGE_STAT2A_GPIO      GPIOB
#define CHARGE_STAT2A_PIN       GPIO_Pin_4
#define BUZZER_GPIO             GPIOB
#define BUZZER_PIN              GPIO_Pin_5
#define GAUGE_BATLOW2_GPIO      GPIOB
#define GAUGE_BATLOW2_PIN       GPIO_Pin_6
#define GAUGE_BATGD2_N_GPIO     GPIOB
#define GAUGE_BATGD2_N_PIN      GPIO_Pin_7
#define GAUGE_SCL2_GPIO         GPIOB
#define GAUGE_SCL2_PIN          GPIO_Pin_8
#define GAUGE_SDA2_GPIO         GPIOB
#define GAUGE_SDA2_PIN          GPIO_Pin_9
#define GAUGE_SCL1_GPIO         GPIOB
#define GAUGE_SCL1_PIN          GPIO_Pin_10
#define GAUGE_SDA1_GPIO         GPIOB
#define GAUGE_SDA1_PIN          GPIO_Pin_11
#define LED_GPIO                GPIOB
#define LED_PIN                 GPIO_Pin_12
#define BT_RTS_GPIO             GPIOB
#define BT_RTS_PIN              GPIO_Pin_13
#define BT_CTS_GPIO             GPIOB
#define BT_CTS_PIN              GPIO_Pin_14
#define SYS_UART_UP_GPIO        GPIOB
#define SYS_UART_UP_PIN         GPIO_Pin_15

#define CHARGE_STAT1A_GPIO      GPIOC
#define CHARGE_STAT1A_PIN       GPIO_Pin_0
#define GAUGE_BATLOW1_GPIO      GPIOC
#define GAUGE_BATLOW1_PIN       GPIO_Pin_1
#define GAUGE_BATGD1_N_GPIO     GPIOC
#define GAUGE_BATGD1_N_PIN      GPIO_Pin_2
#define CHARGE_EN1_N_GPIO       GPIOC
#define CHARGE_EN1_N_PIN        GPIO_Pin_3
#define IR_INT2_N_GPIO          GPIOC
#define IR_INT2_N_PIN           GPIO_Pin_4
#define TOUCH_INT_N_GPIO        GPIOC
#define TOUCH_INT_N_PIN         GPIO_Pin_5
#define SYS_DONE_GPIO           GPIOC
#define SYS_DONE_PIN            GPIO_Pin_6
#define SYS_PROG_N_GPIO         GPIOC
#define SYS_PROG_N_PIN          GPIO_Pin_7
#define PATH_DC_GPIO            GPIOC
#define PATH_DC_PIN             GPIO_Pin_8
#define SYS_SPI_DIR_GPIO        GPIOC
#define SYS_SPI_DIR_PIN         GPIO_Pin_9
#define BT_RX_GPIO              GPIOC
#define BT_RX_PIN               GPIO_Pin_10
#define BT_TX_GPIO              GPIOC
#define BT_TX_PIN               GPIO_Pin_11
#define SYS_SYNC_N_GPIO         GPIOC
#define SYS_SYNC_N_PIN          GPIO_Pin_12
#define SYS_PD_N_GPIO           GPIOC
#define SYS_PD_N_PIN            GPIO_Pin_13
#define SYS_WARMRST_N_GPIO      GPIOC
#define SYS_WARMRST_N_PIN       GPIO_Pin_14
#define BT_RST_GPIO             GPIOC
#define BT_RST_PIN              GPIO_Pin_15

#define OSC_IN_GPIO             GPIOD
#define OSC_IN_PIN              GPIO_Pin_0
#define OSC_OUT_GPIO            GPIOD
#define OSC_OUT_PIN             GPIO_Pin_1
#define CHARGE_EN2_N_GPIO       GPIOD
#define CHARGE_EN2_N_PIN        GPIO_Pin_2

#endif /* IODEF_H */

