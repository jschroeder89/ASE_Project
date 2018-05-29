/************************************************************************************//**
* \file         Source\gateway.h
* \brief        Bootloader gateway interface header file. Needs the communication module!
* \ingroup      Core
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2011  by Feaser    http://www.feaser.com    All rights reserved
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
#ifndef GATE_H
#define GATE_H

#if (BOOT_GATE_ENABLE > 0)

#if (BOOT_COM_ENABLE <= 0)
/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Defines the maximum number of bytes for transport layer reception
 *         depending on the activates interface(s).
 */
#define BOOT_COM_RX_MAX_DATA            (1)
/* update in case CAN interface uses more */
#if (BOOT_COM_CAN_RX_MAX_DATA > BOOT_COM_RX_MAX_DATA)
#undef BOOT_COM_RX_MAX_DATA
#define BOOT_COM_RX_MAX_DATA            (BOOT_COM_CAN_RX_MAX_DATA)
#endif
/* update in case UART interface uses more */
#if (BOOT_COM_UART_RX_MAX_DATA > BOOT_COM_RX_MAX_DATA)
#undef BOOT_COM_RX_MAX_DATA
#define BOOT_COM_RX_MAX_DATA            (BOOT_COM_UART_RX_MAX_DATA)
#endif
/* update in case USB interface uses more */
#if (BOOT_COM_USB_RX_MAX_DATA > BOOT_COM_RX_MAX_DATA)
#undef BOOT_COM_RX_MAX_DATA
#define BOOT_COM_RX_MAX_DATA            (BOOT_COM_USB_RX_MAX_DATA)
#endif
/* update in case NET interface uses more */
#if (BOOT_COM_NET_RX_MAX_DATA > BOOT_COM_RX_MAX_DATA)
#undef BOOT_COM_RX_MAX_DATA
#define BOOT_COM_RX_MAX_DATA            (BOOT_COM_NET_RX_MAX_DATA)
#endif

/** \brief Defines the maximum number of bytes for transport layer transmission
 *         depending on the activates interface(s).
 */
#define BOOT_COM_TX_MAX_DATA            (1)
/* update in case CAN interface uses more */
#if (BOOT_COM_CAN_TX_MAX_DATA > BOOT_COM_TX_MAX_DATA)
#undef BOOT_COM_TX_MAX_DATA
#define BOOT_COM_TX_MAX_DATA            (BOOT_COM_CAN_TX_MAX_DATA)
#endif
/* update in case UART interface uses more */
#if (BOOT_COM_UART_TX_MAX_DATA > BOOT_COM_TX_MAX_DATA)
#undef BOOT_COM_TX_MAX_DATA
#define BOOT_COM_TX_MAX_DATA            (BOOT_COM_UART_TX_MAX_DATA)
#endif
/* update in case USB interface uses more */
#if (BOOT_COM_USB_TX_MAX_DATA > BOOT_COM_TX_MAX_DATA)
#undef BOOT_COM_TX_MAX_DATA
#define BOOT_COM_TX_MAX_DATA            (BOOT_COM_USB_TX_MAX_DATA)
#endif
/* update in case NET interface uses more */
#if (BOOT_COM_NET_TX_MAX_DATA > BOOT_COM_TX_MAX_DATA)
#undef BOOT_COM_TX_MAX_DATA
#define BOOT_COM_TX_MAX_DATA            (BOOT_COM_NET_TX_MAX_DATA)
#endif

#endif /* BOOT_COM_ENABLE <= 0 */


/****************************************************************************************
* Plausibility
****************************************************************************************/
#if (BOOT_COM_TX_MAX_DATA < 1)
#undef BOOT_COM_TX_MAX_DATA
#define BOOT_COM_TX_MAX_DATA   (8)
#endif

#if (BOOT_COM_TX_MAX_DATA > 256)
#error  "COM.H, BOOT_COM_TX_MAX_DATA cannot be larger than 256."
#endif

#if (BOOT_COM_RX_MAX_DATA < 1)
#undef BOOT_COM_RX_MAX_DATA
#define BOOT_COM_RX_MAX_DATA   (8)
#endif

#if (BOOT_COM_RX_MAX_DATA > 65536)
#error  "COM.H, BOOT_COM_RX_MAX_DATA cannot be larger than 65536."
#endif


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Enumeration for the different gateway interfaces. */
typedef enum
{
  GATE_IF_UART,                                   /**< UART interface                   */
  GATE_IF_CAN,                                    /**< CAN interface                    */
  GATE_IF_USB,                                    /**< USB interface                    */
  GATE_IF_NET,                                    /**< NET interface                    */
  GATE_IF_OTHER                                   /**< Other interface                  */
} tGateInterfaceId;


/****************************************************************************************
* Function prototypes
****************************************************************************************/
void            GateInit(void);
void            GateTask(void);
void            GateFree(void);
blt_int16u      GateGetActiveInterfaceMaxRxLen(void);
blt_int16u      GateGetActiveInterfaceMaxTxLen(void);
void            GateTransmitPacket(blt_int8u *data, blt_int16u len);
#if (BOOT_GATE_CAN_ENABLE > 0)
void            GateTransmitPacketDirect(blt_int8u *data, blt_int8u len, blt_int32u deviceID);
#endif
blt_bool        GateIsConnected(void);
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
void            BuildData(blt_int8u *debugData, blt_int8u *data, blt_int8u len);
#endif

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
blt_int8u debugDataRCan[8];
blt_int8u debugDataTCan[8];
#endif

#endif /* BOOT_GATE_ENABLE > 0 */

#endif /* GATE_H */
/*********************************** end of gateway.h **************************************/
