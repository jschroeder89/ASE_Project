/************************************************************************************//**
* \file         Source\ARMCM4_STM32\bluetoothUart.c
* \brief        Bootloader BLUETOOTH UART communication interface source file.
* \ingroup      Target_ARMCM4_STM32
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
#include "stm32f4xx.h"                           /* STM32 registers                    */
#include "stm32f4xx_conf.h"                      /* STM32 peripheral drivers           */


#if (BOOT_COM_BLUETOOTH_UART_ENABLE > 0)
/****************************************************************************************
* Macro definitions
****************************************************************************************/
/* map the configured BLUETOOTH UART channel index to the STM32's BLUETOOTH USART peripheral */
#if (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 0)
/** \brief Set BLUETOOTH UART base address to USART1. */
#define BLUETOOTH_USART_CHANNEL   USART1
#elif (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 1)
/** \brief Set BLUETOOTH UART base address to USART2. */
#define BLUETOOTH_USART_CHANNEL   USART2
#elif (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 2)
/** \brief Set BLUETOOTH UART base address to USART3. */
#define BLUETOOTH_USART_CHANNEL   USART3
#elif (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 3)
/** \brief Set BLUETOOTH UART base address to USART4. */
#define BLUETOOTH_USART_CHANNEL   USART4
#elif (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 4)
/** \brief Set BLUETOOTH UART base address to USART5. */
#define BLUETOOTH_USART_CHANNEL   USART5
#elif (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 5)
/** \brief Set BLUETOOTH UART base address to USART6. */
#define BLUETOOTH_USART_CHANNEL   USART6
#endif

/* Commands of bluetooth communication */
/** \brief Define bluetooth start of frame */
#define BLUETOOTH_START_FRAME   0xBF
/** \brief Define bluetooth link address to iWRAP control */
#define BLUETOOTH_LINK          0x00
/** \brief Define bluetooth frame flags */
#define BLUETOOTH_FLAGS         0x00
/** \brief Define nLINK = {LINK} XOR 0xFF */
#define BLUETOOTH_NLINK         0xFF

/* States of bluetooth communication */
/** \brief bluetooth waiting for start frame signal */
#define BLUETOOTH_STATUS_START    0
/** \brief bluetooth waiting for link address */
#define BLUETOOTH_STATUS_LINK     1
/** \brief bluetooth waiting for frame flags */
#define BLUETOOTH_STATUS_FLAGS    2
/** \brief bluetooth waiting for data length */
#define BLUETOOTH_STATUS_LENGTH   3
/** \brief bluetooth waiting for data */
#define BLUETOOTH_STATUS_DATA     4
/** \brief bluetooth waiting for frame end (nLINK) */
#define BLUETOOTH_STATUS_END      5
static blt_int8u bluetoothStatus = BLUETOOTH_STATUS_START;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static blt_bool BluetoothUartReceiveByte(blt_int8u *data);
static blt_bool BluetoothUartTransmitByte(blt_int8u data);


/************************************************************************************//**
** \brief     Initializes the BLUETOOTH UART communication interface.
** \return    none.
**
****************************************************************************************/
void BluetoothUartInit(void)
{
  USART_InitTypeDef USART_InitStructure;

  /* the current implementation supports USART1 - USART6. throw an assertion error in 
   * case a different BLUETOOTH UART channel is configured.  
   */
  ASSERT_CT((BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 0) ||
            (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 1) ||
            (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 2) ||
            (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 3) ||
            (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 4) ||
            (BOOT_COM_BLUETOOTH_UART_CHANNEL_INDEX == 5)); 
  /* initialize the BLUETOOTH UART for the specified communication speed */
  USART_InitStructure.USART_BaudRate = BOOT_COM_UART_BAUDRATE;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(BLUETOOTH_USART_CHANNEL, &USART_InitStructure);
  /* enable BLUETOOTH UART */
  USART_Cmd(BLUETOOTH_USART_CHANNEL, ENABLE);

  blt_int32u counter = 0;
  while (counter < 20000000) {
    counter++;
  }

/*  BluetoothUartTransmitByte(BLUETOOTH_START_FRAME);
  BluetoothUartTransmitByte(0xFF);
  BluetoothUartTransmitByte(BLUETOOTH_FLAGS);
  BluetoothUartTransmitByte(18);
  BluetoothUartTransmitByte('S');
  BluetoothUartTransmitByte('E');
  BluetoothUartTransmitByte('T');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('P');
  BluetoothUartTransmitByte('R');
  BluetoothUartTransmitByte('O');
  BluetoothUartTransmitByte('F');
  BluetoothUartTransmitByte('I');
  BluetoothUartTransmitByte('L');
  BluetoothUartTransmitByte('E');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('S');
  BluetoothUartTransmitByte('P');
  BluetoothUartTransmitByte('P');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('O');
  BluetoothUartTransmitByte('N');
  BluetoothUartTransmitByte(0x00);*/

  BluetoothUartTransmitByte(BLUETOOTH_START_FRAME);
  BluetoothUartTransmitByte(0xFF);
  BluetoothUartTransmitByte(BLUETOOTH_FLAGS);
  BluetoothUartTransmitByte(22);
  BluetoothUartTransmitByte('S');
  BluetoothUartTransmitByte('E');
  BluetoothUartTransmitByte('T');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('B');
  BluetoothUartTransmitByte('T');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('S');
  BluetoothUartTransmitByte('N');
  BluetoothUartTransmitByte('I');
  BluetoothUartTransmitByte('F');
  BluetoothUartTransmitByte('F');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('4');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('2');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('1');
  BluetoothUartTransmitByte(' ');
  BluetoothUartTransmitByte('8');
  BluetoothUartTransmitByte('\r');
  BluetoothUartTransmitByte('\n');
  BluetoothUartTransmitByte(0x00);

  BluetoothUartTransmitByte(BLUETOOTH_START_FRAME);
  BluetoothUartTransmitByte(0xFF);
  BluetoothUartTransmitByte(BLUETOOTH_FLAGS);
  BluetoothUartTransmitByte(5);
  BluetoothUartTransmitByte('R');
  BluetoothUartTransmitByte('E');
  BluetoothUartTransmitByte('S');
  BluetoothUartTransmitByte('E');
  BluetoothUartTransmitByte('T');
  BluetoothUartTransmitByte(0x00);
} /*** end of BluetoothUartInit ***/


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data Pointer to byte array with data that it to be transmitted.
** \param     len  Number of bytes that are to be transmitted.
** \return    none.
**
****************************************************************************************/
void BluetoothUartTransmitPacket(blt_int8u *data, blt_int8u len)
{
  blt_int16u data_index;
  blt_bool result; 

  /* verify validity of the len-paramenter */
  ASSERT_RT(len <= BOOT_COM_UART_TX_MAX_DATA);
  
  /* Send bluetooth header first */
  result = BluetoothUartTransmitByte(BLUETOOTH_START_FRAME);
  ASSERT_RT(result == BLT_TRUE);
  result = BluetoothUartTransmitByte(BLUETOOTH_LINK);
  ASSERT_RT(result == BLT_TRUE);
  result = BluetoothUartTransmitByte(BLUETOOTH_FLAGS);
  ASSERT_RT(result == BLT_TRUE);
  result = BluetoothUartTransmitByte(len+1);
  ASSERT_RT(result == BLT_TRUE);
  /* end of bluetooth header */

  /* first transmit the length of the packet */ 
  result = BluetoothUartTransmitByte(len);
  ASSERT_RT(result == BLT_TRUE);

  /* transmit all the packet bytes one-by-one */
  for (data_index = 0; data_index < len; data_index++)
  {
    /* keep the watchdog happy */
    CopService();
    /* write byte */
    result = BluetoothUartTransmitByte(data[data_index]);
    ASSERT_RT(result == BLT_TRUE);  
  }

  /* Send bluetooth closer */
  result = BluetoothUartTransmitByte(BLUETOOTH_NLINK);
  ASSERT_RT(result == BLT_TRUE);
  /* end of bluetooth closer */

} /*** end of BluetoothUartTransmitPacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \return    BLT_TRUE if a packet was received, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_int8u BluetoothUartReceivePacket(blt_int8u *data)
{
  static blt_int8u xcpCtoReqPacket[BOOT_COM_UART_RX_MAX_DATA+1];  /* one extra for length */
  static blt_int8u bluetoothDataLength;
  static blt_int8u xcpUartDataLength;
  static blt_int8u xcpCtoRxLength;
  static blt_int8u bluetoothLinkID;
  static blt_int8u byteData;
  static blt_bool  xcpCtoRxInProgress = BLT_FALSE;

  /* store the message length when received */
    if (BluetoothUartReceiveByte(&byteData) == BLT_TRUE)
    {
 
      /* Check bluetooth status */
      switch (bluetoothStatus) {

        case BLUETOOTH_STATUS_START:
          if (byteData == BLUETOOTH_START_FRAME) {
            bluetoothStatus++;
          }
          break;

        case BLUETOOTH_STATUS_LINK:
          //if (xcpCtoReqPacket[0] == BLUETOOTH_LINK) {
            bluetoothLinkID = byteData;
            bluetoothStatus++;
          //} else {
          //  bluetoothStatus = BLUETOOTH_STATUS_START;
          //}
          break;

        case BLUETOOTH_STATUS_FLAGS:
          if (byteData == BLUETOOTH_FLAGS) {
            bluetoothStatus++;
          } else {
            bluetoothStatus = BLUETOOTH_STATUS_START;
          }
          break;

        case BLUETOOTH_STATUS_LENGTH:
          bluetoothStatus++;
          bluetoothDataLength = byteData;
          break;

        case BLUETOOTH_STATUS_DATA:
          if (--bluetoothDataLength == 0) {
            bluetoothStatus++;
          }
          if (bluetoothLinkID == 0xFF) {
          } else {
            /* start of cto packet received? */
            if (xcpCtoRxInProgress == BLT_FALSE)
            {
              xcpCtoReqPacket[0] = byteData;
              /* save message length */
              xcpUartDataLength = xcpCtoReqPacket[0];
              if (xcpCtoReqPacket[0] > 0)
              {
                /* indicate that a cto packet is being received */
                xcpCtoRxInProgress = BLT_TRUE;
                /* reset packet data count */
                xcpCtoRxLength = 0;
              }
            }
            else
            {
              /* store the next packet byte */
              /* increment the packet data count */
              xcpCtoReqPacket[++xcpCtoRxLength] = byteData;
 
              /* check to see if the entire packet was received */
              if (xcpCtoRxLength == xcpCtoReqPacket[0])
              {
                /* copy the packet data */
                CpuMemCopy((blt_int32u)data, (blt_int32u)&xcpCtoReqPacket[1], xcpCtoRxLength);        
                /* done with cto packet reception */
                xcpCtoRxInProgress = BLT_FALSE;

                /* packet reception complete */
                return xcpUartDataLength;
              }
            }
          }
          break;

        default:
          bluetoothStatus = BLUETOOTH_STATUS_START;
          if (byteData == ~bluetoothLinkID) {
        	/* copy the packet data */
        	//CpuMemCopy((blt_int32u)data, (blt_int32u)&xcpCtoReqPacket[1], xcpCtoRxLength);
                //return xcpCtoRxLength;
          }
          break;
      }
    }
  /* packet reception not yet complete */
  return 0;
} /*** end of BluetoothUartReceivePacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface byte if one is present.
** \param     data Pointer to byte where the data is to be stored.
** \return    BLT_TRUE if a byte was received, BLT_FALSE otherwise.
**
****************************************************************************************/
static blt_bool BluetoothUartReceiveByte(blt_int8u *data)
{
  /* check flag to see if a byte was received */
  if (USART_GetFlagStatus(BLUETOOTH_USART_CHANNEL, USART_FLAG_RXNE) == SET)
  {
    /* retrieve and store the newly received byte */
    *data = (unsigned char)USART_ReceiveData(BLUETOOTH_USART_CHANNEL);
    //UartTransmitPacket(data, 1);
    //UartTransmitPacket("\n", 1);
    /* all done */
    return BLT_TRUE;
  }
  /* still here to no new byte received */
  return BLT_FALSE;
} /*** end of BluetoothUartReceiveByte ***/


/************************************************************************************//**
** \brief     Transmits a communication interface byte.
** \param     data Value of byte that is to be transmitted.
** \return    BLT_TRUE if the byte was transmitted, BLT_FALSE otherwise.
**
****************************************************************************************/
static blt_bool BluetoothUartTransmitByte(blt_int8u data)
{
  /* check if tx holding register can accept new data */
  if (USART_GetFlagStatus(BLUETOOTH_USART_CHANNEL, USART_FLAG_TXE) == RESET)
  {
    /* BLUETOOTH UART not ready. should not happen */
    return BLT_FALSE;
  }
  /* write byte to transmit holding register */
  USART_SendData(BLUETOOTH_USART_CHANNEL, data);
  /* wait for tx holding register to be empty */
  while (USART_GetFlagStatus(BLUETOOTH_USART_CHANNEL, USART_FLAG_TXE) == RESET) 
  { 
    ;
  }
  /* byte transmitted */
  return BLT_TRUE;
} /*** end of BluetoothUartTransmitByte ***/
#endif /* BOOT_COM_BLUETOOTH_UART_ENABLE > 0 */


/*********************************** end of bluetoothUart.c *************************************/
