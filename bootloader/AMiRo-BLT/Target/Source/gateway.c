/************************************************************************************//**
* \file         Source\gateway.c
* \brief        Bootloader gateway interface source file. Needs the communication module!
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

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */
#if (BOOT_GATE_CAN_ENABLE > 0)
  #include "can.h"                                    /* can driver module             */
#endif
#if (BOOT_GATE_UART_ENABLE > 0)
  #include "uart.h"                                   /* uart driver module            */
#endif
#if (BOOT_GATE_USB_ENABLE > 0)
  #include "usb.h"                                    /* usb driver module             */
#endif
#if (BOOT_GATE_NET_ENABLE > 0)
  #include "net.h"                                    /* tcp/ip driver module          */
#endif


#if (BOOT_GATE_ENABLE > 0)
/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Holds the gateway interface of the currently active interface. */
static tGateInterfaceId gateActiveInterface = GATE_IF_OTHER;
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
static void InitDebugData(void);
#endif

blt_bool forwarded;

/************************************************************************************//**
** \brief     Initializes the gateway module. The hardware needed for the
**            communication should be initialized in the communication module!
** \return    none
**
****************************************************************************************/
void GateInit(void)
{
  forwarded = BLT_FALSE;
#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
  InitDebugData();
#endif
#if (BOOT_GATE_CAN_ENABLE > 0)
  #if (BOOT_COM_CAN_ENABLE <= 0)
  /* initialize the CAN controller */
  CanInit();
  #endif
  /* set it as active */
  gateActiveInterface = GATE_IF_CAN;
#endif
#if (BOOT_GATE_UART_ENABLE > 0)
  #if (BOOT_COM_UART_ENABLE <= 0)
  /* initialize the UART interface */
  UartInit();
  #endif
  /* set it as active */
  gateActiveInterface = GATE_IF_UART;
#endif
#if (BOOT_GATE_USB_ENABLE > 0)
  #if (BOOT_COM_USB_ENABLE <= 0)
  /* initialize the USB interface */
  UsbInit();
  #endif
  /* set it as active */
  gateActiveInterface = GATE_IF_USB;
#endif
#if (BOOT_GATE_NET_ENABLE > 0)
  #if (BOOT_COM_NET_ENABLE <= 0)
  /* initialize the TCP/IP interface */
  NetInit();
  #endif
  /* set it as active */
  gateActiveInterface = GATE_IF_NET;
#endif
} /*** end of GateInit ***/

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
/************************************************************************************//**
** \brief     Initializes the debugging data.
** \return    none
**
****************************************************************************************/
static void InitDebugData(void) {
  debugDataRCan[0] = 'r';
  debugDataRCan[1] = 'C';
  debugDataRCan[2] = 'A';
  debugDataRCan[3] = 'N';
  debugDataRCan[4] = ' ';
  debugDataRCan[5] = ':';
  debugDataRCan[6] = ' ';
  debugDataRCan[7] = 10;
  debugDataTCan[0] = 't';
  debugDataTCan[1] = 'C';
  debugDataTCan[2] = 'A';
  debugDataTCan[3] = 'N';
  debugDataTCan[4] = ' ';
  debugDataTCan[5] = ':';
  debugDataTCan[6] = ' ';
  debugDataTCan[7] = 10;
} /*** end of InitDebugData ***/
#endif


/************************************************************************************//**
** \brief     Updates the gateway module by checking if new data was received
**            and submitting the request to process newly received data.
** \return    none
**
****************************************************************************************/
void GateTask(void)
{
  blt_int16s messageLength;
  /* make xcpCtoReqPacket static for runtime efficiency */
  static unsigned char xcpCtoReqPacket[BOOT_COM_RX_MAX_DATA];

#if (BOOT_GATE_CAN_ENABLE > 0)
  messageLength = (blt_int16s)CanReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    gateActiveInterface = GATE_IF_CAN;
    /* process packet */
    if (forwarded == BLT_TRUE) {
      XcpGatewayPacketReceived(&xcpCtoReqPacket[0], messageLength);
    } else {
      XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_TRUE);
    }
    forwarded = BLT_FALSE;
  }
#endif
#if (BOOT_GATE_UART_ENABLE > 0)
  messageLength = (blt_int16s)UartReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    gateActiveInterface = GATE_IF_UART;
    /* process packet */
    if (forwarded == BLT_TRUE) {
      XcpGatewayPacketReceived(&xcpCtoReqPacket[0], messageLength);
    } else {
      XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_TRUE);
    }
    forwarded = BLT_FALSE;
  }

#endif
#if (BOOT_GATE_USB_ENABLE > 0)
  messageLength = (blt_int16s)UsbReceivePacket(&xcpCtoReqPacket[0]);
  if (messageLength > 0)
  {
    /* make this the active interface */
    gateActiveInterface = GATE_IF_USB;
    /* process packet */
    if (forwarded == BLT_TRUE) {
      XcpGatewayPacketReceived(&xcpCtoReqPacket[0], messageLength);
    } else {
      XcpPacketReceived(&xcpCtoReqPacket[0], messageLength, BLT_TRUE);
    }
    forwarded = BLT_FALSE;
  }
#endif
} /*** end of GateTask ***/


/************************************************************************************//**
** \brief     Releases the gateway module.
** \return    none
**
****************************************************************************************/
void GateFree(void)
{
#if (BOOT_GATE_USB_ENABLE > 0)
  /* disconnect the usb device from the usb host */
  UsbFree();
#endif
} /*** end of GateFree ***/


/************************************************************************************//**
** \brief     Transmits the packet using the xcp transport layer.
** \param     data Pointer to the byte buffer with packet data.
** \param     len  Number of data bytes that need to be transmitted.
** \return    none
**
****************************************************************************************/
void GateTransmitPacket(blt_int8u *data, blt_int16u len)
{
#if (BOOT_GATE_CAN_ENABLE > 0)
  /* transmit the packet. note that len is limited to 8 in the plausibility check,
   * so cast is okay.
   */
  if (gateActiveInterface == GATE_IF_CAN)
  {
    CanTransmitPacket(data, (blt_int8u)len, 0);
  }
#endif
#if (BOOT_GATE_UART_ENABLE > 0)
  /* transmit the packet. note that len is limited to 255 in the plausibility check,
   * so cast is okay.
   */
  if (gateActiveInterface == GATE_IF_UART)
  {
    UartTransmitPacket(data, (blt_int8u)len);
  }
#endif
#if (BOOT_GATE_USB_ENABLE > 0)
  /* transmit the packet */
  if (gateActiveInterface == GATE_IF_USB)
  {
    UsbTransmitPacket(data, len);
  }
#endif
#if (BOOT_GATE_NET_ENABLE > 0)
  if (gateActiveInterface == GATE_IF_NET)
  {
    /* transmit the packet */
    NetTransmitPacket(data, len);
  }
#endif

  /* send signal that the packet was transmitted */
  XcpPacketTransmitted();
} /*** end of GateTransmitPacket ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be received on the specified
**            gateway interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u GateGetActiveInterfaceMaxRxLen(void)
{
  blt_int16u result;

  /* filter on gateway interface identifier */
  switch (gateActiveInterface)
  {
    case GATE_IF_UART:
      result = BOOT_COM_UART_RX_MAX_DATA;
      break;

    case GATE_IF_CAN:
      result = BOOT_COM_CAN_RX_MAX_DATA;
      break;

    case GATE_IF_USB:
      result = BOOT_COM_USB_RX_MAX_DATA;
      break;

    case GATE_IF_NET:
      result = BOOT_COM_NET_RX_MAX_DATA;
      break;

    default:
      result = BOOT_COM_RX_MAX_DATA;
      break;
  }

  return result;
} /*** end of GateGetActiveInterfaceMaxRxLen ***/


/************************************************************************************//**
** \brief     Obtains the maximum number of bytes that can be transmitted on the
**            specified gateway interface.
** \return    Maximum number of bytes that can be received.
**
****************************************************************************************/
blt_int16u GateGetActiveInterfaceMaxTxLen(void)
{
  blt_int16u result;

  /* filter on gateway interface identifier */
  switch (gateActiveInterface)
  {
    case GATE_IF_UART:
      result = BOOT_COM_UART_TX_MAX_DATA;
      break;

    case GATE_IF_CAN:
      result = BOOT_COM_CAN_TX_MAX_DATA;
      break;

    case GATE_IF_USB:
      result = BOOT_COM_USB_TX_MAX_DATA;
      break;

    case GATE_IF_NET:
      result = BOOT_COM_NET_TX_MAX_DATA;
      break;

    default:
      result = BOOT_COM_TX_MAX_DATA;
      break;
  }

  return result;
} /*** end of GateGetActiveInterfaceMaxTxLen ***/


/************************************************************************************//**
** \brief     This function obtains the XCP connection state.
** \return    BLT_TRUE when an XCP connection is established, BLT_FALSE otherwise.
**
****************************************************************************************/
blt_bool GateIsConnected(void)
{
  return XcpIsConnected();
} /*** end of GateIsConnected ***/


#endif /* BOOT_GATE_ENABLE > 0 */





#if (BOOT_GATE_CAN_ENABLE > 0)
/************************************************************************************//**
** \brief     This function transmitts the packet direct with CAN.
** \param     data     Pointer to the byte buffer with packet data.
** \param     len      Number of data bytes that need to be transmitted.
** \param     deviceID ID of the device the data has to be sent to.
** \return    none.
**
****************************************************************************************/
void GateTransmitPacketDirect(blt_int8u *data, blt_int8u len, blt_int32u deviceID) {
  forwarded = BLT_TRUE;
  CanTransmitPacket(data, len, deviceID);
  XcpPacketTransmitted();
} /*** end of GateTransmitPacketDirect ***/

#if (BOOT_DEBUGGING_UART2_ENABLE > 0)
/************************************************************************************//**
** \brief     This function transmitts debugging data with UART2.
** \param     debugData Pointer to the byte buffer with additional debugging data.
** \param     data      Pointer to the byte buffer with packet data.
** \param     len       Number of packet data bytes that need to be transmitted.
** \return    none.
**
****************************************************************************************/
void BuildData(blt_int8u *debugData, blt_int8u *data, blt_int8u len) {
  UartSendDebuggingPacket(debugData, 7);
  UartSendDebuggingPacket(data, len);
  UartSendDebuggingPacket(&debugData[7], 1);
} /*** end of BuildData ***/

#endif /* BOOT_DEBUGGING_UART2_ENABLE > 0 */

#endif /* BOOT_GATE_CAN_ENABLE > 0 */

/*********************************** end of gateway.c **************************************/
