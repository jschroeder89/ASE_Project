#include "helper.h"

#include "blt_conf.h"

/*
 * Initialized the system timer.
 */
void saTimerInit(void) {
  /* reset the timer configuration */
  saTimerReset();

  /* configure the systick frequency as a 1 ms event generator */
  SysTick->LOAD = BOOT_CPU_SYSTEM_SPEED_KHZ - 1;
  /* reset the current counter value */
  SysTick->VAL = 0;
  /* select core clock as source and enable the timer */
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

/*
 * Resets the systick status of the system timer.
 */
void saTimerReset(void) {
  /* set the systick's status and control register back into the default reset value */
  SysTick->CTRL = 0;
}

/*
 * Updates the given timer variable.
 * More specifically, the given variable in incremented if a millisecond event occurred.
 */
void saTimerUpdate(uint32_t* millisecond_counter) {
  /* check if the millisecond event occurred */
  if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0)
  {
    /* increment the millisecond counter */
    ++(*millisecond_counter);
  }

  return;
}

/*
 * Actively polls the standalone timer until the specified time has passed.
 */
void msleep(uint32_t ms)
{
  uint32_t current;
  saTimerUpdate(&current);
  uint32_t end = current + ms;

  while (current < end)
  {
    saTimerUpdate(&current);
  }

  return;
}

/*
 * Actively reads the specified GPIO until it has the specified state.
 */
void waitForSignal(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction state) {
  /* check whether the signal has been set */
  while (GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) != state) {
    continue;
  }
  return;
}

/*
 * Actively reads the specified GPIO until it has the specified state, or the specified time has passed.
 */
uint8_t waitForSignalTimeout(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, BitAction state, uint32_t timeout_ms) {
  uint32_t current_time;
  saTimerUpdate(&current_time);
  uint32_t timeout_time = current_time + timeout_ms;
  while ((GPIO_ReadInputDataBit(GPIOx, GPIO_Pin) != state) &&
         (current_time < timeout_time)) {
    saTimerUpdate(&current_time);
  }
  if (current_time < timeout_time) {
    return 1;
  } else {
    return 0;
  }
}

/*
 * Turns the board LED or or off respectively.
 * If the argument is zero, the LED is switched off.
 * If the argument is not zero, the LED is switched on.
 */
void setLed(uint8_t on) {

#if defined(LED_GPIO) && defined(LED_PIN)
  if (on == 0) {
    GPIO_SetBits(LED_GPIO, LED_PIN);
  } else {
    GPIO_ResetBits(LED_GPIO, LED_PIN);
  }
#endif

  return;
}

/*
 * Makes the LED blink 'SOS' in morese code (... --- ...).
 * If the specified number of loops is zero, the function will loop infinitely.
 */
void blinkSOS(uint32_t loops) {
  /* initialize some variables and constants */
  enum State {BLINK_ERROR_S1,
              BLINK_ERROR_O,
              BLINK_ERROR_S2,
              BLINK_ERROR_BREAK
             } state = BLINK_ERROR_S1;
  uint8_t led = 0;
  uint32_t loop = 0;
  const uint32_t sigS = 50;
  const uint32_t sigL = 200;
  const uint32_t sigB = 100;
  const uint32_t letterBreakTime = 200;
  const uint32_t wordBreakTime = 1000;
  uint32_t stateStartTime = 0;
  saTimerUpdate(&stateStartTime);
  uint32_t currentTime = stateStartTime;

  /* either loop the specified number, or infinitely */
  while (loop < loops || loops == 0) {
    /* make the LED blink "SOS" (morse code: ... --- ...)*/
    led = 0;
    saTimerUpdate(&currentTime);
    switch (state) {
      case BLINK_ERROR_S1:
      case BLINK_ERROR_S2:
      {
        if (currentTime < stateStartTime + sigS) {
          led = 1;
        } else if (currentTime < stateStartTime + sigS+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigS+sigB+sigS) {
          led = 1;
        } else if (currentTime < stateStartTime + sigS+sigB+sigS+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigS+sigB+sigS+sigB+sigS) {
          led = 1;
        } else if (currentTime < stateStartTime + sigS+sigB+sigS+sigB+sigS+letterBreakTime) {
          led = 0;
        } else {
          if (state == BLINK_ERROR_S1) {
            state = BLINK_ERROR_O;
          } else {
            state = BLINK_ERROR_BREAK;
            ++loop;
          }
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_ERROR_O:
      {
        if (currentTime < stateStartTime + sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB+sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB+sigL+letterBreakTime) {
          led = 0;
        } else {
          state = BLINK_ERROR_S2;
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_ERROR_BREAK:
      {
        if (currentTime >= stateStartTime + wordBreakTime) {
          state = BLINK_ERROR_S1;
          stateStartTime = currentTime;
        }
        break;
      }
    }

    setLed(led);
  }

  return;
}

/*
 * Shortcut to make the LED blink SOS infinitely.
 */
inline void blinkSOSinf() {
  blinkSOS(0);
  return;
}

/*
 * Makes the LED blink 'OK' in morese code (... -.-).
 * If the specified number of loops is zero, the function will loop infinitely.
 */
void blinkOK(uint32_t loops) {
  /* initialize some variables and constants */
  enum State {BLINK_SUCCESS_O,
              BLINK_SUCCESS_K,
             BLINK_SUCCESS_BREAK
             } state = BLINK_SUCCESS_O;
  uint8_t led = 0;
  uint32_t loop = 0;
  const uint32_t sigS = 50;
  const uint32_t sigL = 200;
  const uint32_t sigB = 100;
  const uint32_t letterBreakTime = 200;
  const uint32_t wordBreakTime = 1000;
  uint32_t stateStartTime = 0;
  saTimerUpdate(&stateStartTime);
  uint32_t currentTime = stateStartTime;

  /* either loop the specified number, or infinitely */
  while (loop < loops || loops == 0)
  {
    /* make the LED blink "OK" (morse code: --- -.-)*/
    led = 0;
    saTimerUpdate(&currentTime);
    switch (state) {
      case BLINK_SUCCESS_O:
      {
        if (currentTime < stateStartTime + sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB+sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigL+sigB+sigL+letterBreakTime) {
          led = 0;
        } else {
          state = BLINK_SUCCESS_K;
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_SUCCESS_K:
      {
        if (currentTime < stateStartTime + sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigS) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigS+sigB) {
          led = 0;
        } else if (currentTime < stateStartTime + sigL+sigB+sigS+sigB+sigL) {
          led = 1;
        } else if (currentTime < stateStartTime + sigL+sigB+sigS+sigB+sigL+letterBreakTime) {
          led = 0;
        } else {
          state = BLINK_SUCCESS_BREAK;
          ++loop;
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_SUCCESS_BREAK:
      {
        if (currentTime >= stateStartTime + wordBreakTime) {
          state = BLINK_SUCCESS_O;
          stateStartTime = currentTime;
        }
        break;
      }
    }

    setLed(led);
  }

  return;
}

/*
 * Shortcut to make the LED blink OK infinitely.
 */
inline void blinkOKinf() {
  blinkOK(0);
  return;
}

/*
 * Makes the LED visualize the specified data.
 * Starting with the MSB of the first of the 'n' bytes, zeros are visualized as short flash and ones as long flash.
 * If the specified number of loops is zero, the function will loop infinitely.
 */
void visualizeData(uint8_t* data, uint32_t bytes, uint32_t loops) {
  /* initialize some variables and constants */
  enum State {BLINK_DATA_BIT,
              BLINK_DATA_BYTE_BREAK,
              BLINK_DATA_LOOP_BREAK
             } state = BLINK_DATA_BIT;
  uint8_t led = 0;
  uint8_t mask = 0x80;
  uint32_t byte = 0;
  uint32_t loop = 0;
  const uint32_t sigS = 50;
  const uint32_t sigL = 200;
  const uint32_t interBitBreak = 500;
  const uint32_t interByteBreak = 1000;
  const uint32_t interLoopBreak = 2500;
  uint32_t flash_dur = 0;
  uint32_t stateStartTime = 0;
  saTimerUpdate(&stateStartTime);
  uint32_t currentTime = stateStartTime;

  /* return immediately if the number of bytes is zero */
  if (bytes == 0) {
    return;
  }

  /* either loop the specified number, or infinetly */
  while (loop < loops || loops == 0) {
    led = 0;
    saTimerUpdate(&currentTime);
    switch (state) {
      case BLINK_DATA_BIT:
      {
        if (data[byte] & mask) {
          flash_dur = sigL;
        } else {
          flash_dur = sigS;
        }
        if (currentTime < stateStartTime + flash_dur) {
          led = 1;
        } else if (currentTime < stateStartTime + flash_dur+interBitBreak) {
          led = 0;
        } else {
          mask = mask >> 1;
          if (mask > 0) {
            state = BLINK_DATA_BIT;
          } else if (byte < bytes-1) {
            state = BLINK_DATA_BYTE_BREAK;
          } else {
            state = BLINK_DATA_LOOP_BREAK;
            ++loop;
          }
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_DATA_BYTE_BREAK:
      {
        if (currentTime >= stateStartTime + interByteBreak) {
          mask = 0x80;
          state = BLINK_DATA_BIT;
          ++byte;
          stateStartTime = currentTime;
        }
        break;
      }
      case BLINK_DATA_LOOP_BREAK:
      {
        if (currentTime >= stateStartTime + interLoopBreak) {
          mask = 0x80;
          state = BLINK_DATA_BIT;
          byte = 0;
          stateStartTime = currentTime;
        }
        break;
      }
    }

    setLed(led);
  }

  return;
}

/*
 * Makes the LED visualize the specified byte.
 * Starting with the MSB, zeros are visualized as short flash and ones as long flash.
 * If the specified number of loops is zero, the function will loop infinitely.
 */
void visualizeByte(uint8_t byte, uint32_t loops) {
  visualizeData(&byte, 1, loops);
  return;
}

