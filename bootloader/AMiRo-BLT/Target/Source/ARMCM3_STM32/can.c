/************************************************************************************//**
* \file         Source\ARMCM3_STM32\can.c
* \brief        Bootloader CAN communication interface source file.
* \ingroup      Target_ARMCM3_STM32
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

#define CAN_DEBUG       (0)

/****************************************************************************************
* Include files
****************************************************************************************/
#include "boot.h"                                /* bootloader generic header          */

#if (CAN_DEBUG > 0)
#include <helper.h>
#endif


#if (BOOT_COM_CAN_ENABLE > 0 || BOOT_GATE_CAN_ENABLE > 0)
/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief CAN transmission mailbox layout. */
typedef struct
{
  volatile blt_int32u TIR;
  volatile blt_int32u TDTR;
  volatile blt_int32u TDLR;
  volatile blt_int32u TDHR;
} tCanTxMailBox;

/** \brief CAN reception FIFO mailbox layout. */
typedef struct
{
  volatile blt_int32u RIR;
  volatile blt_int32u RDTR;
  volatile blt_int32u RDLR;
  volatile blt_int32u RDHR;
} tCanRxFIFOMailBox;

/** \brief CAN filter register layout. */
typedef struct
{
  volatile blt_int32u FR1;
  volatile blt_int32u FR2;
} tCanFilter;

/** \brief CAN controller register layout. */
typedef struct
{
  volatile blt_int32u MCR;
  volatile blt_int32u MSR;
  volatile blt_int32u TSR;
  volatile blt_int32u RF0R;
  volatile blt_int32u RF1R;
  volatile blt_int32u IER;
  volatile blt_int32u ESR;
  volatile blt_int32u BTR;
  blt_int32u          RESERVED0[88];
  tCanTxMailBox       sTxMailBox[3];
  tCanRxFIFOMailBox   sFIFOMailBox[2];
  blt_int32u          RESERVED1[12];
  volatile blt_int32u FMR;
  volatile blt_int32u FM1R;
  blt_int32u          RESERVED2;
  volatile blt_int32u FS1R;
  blt_int32u          RESERVED3;
  volatile blt_int32u FFA1R;
  blt_int32u          RESERVED4;
  volatile blt_int32u FA1R;
  blt_int32u          RESERVED5[8];
  tCanFilter          sFilterRegister[14];
} tCanRegs;


/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Reset request bit. */
#define CAN_BIT_RESET    ((blt_int32u)0x00008000)
/** \brief Initialization request bit. */
#define CAN_BIT_INRQ     ((blt_int32u)0x00000001)
/** \brief Initialization acknowledge bit. */
#define CAN_BIT_INAK     ((blt_int32u)0x00000001)
/** \brief Sleep mode request bit. */
#define CAN_BIT_SLEEP    ((blt_int32u)0x00000002)
/** \brief Filter 0 selection bit. */
#define CAN_BIT_FILTER0  ((blt_int32u)0x00000001)
/** \brief Filter init mode bit. */
#define CAN_BIT_FINIT    ((blt_int32u)0x00000001)
/** \brief Transmit mailbox 0 empty bit. */
#define CAN_BIT_TME0     ((blt_int32u)0x04000000)
/** \brief Transmit mailbox request bit. */
#define CAN_BIT_TXRQ     ((blt_int32u)0x00000001)
/** \brief Release FIFO 0 mailbox bit. */
#define CAN_BIT_RFOM0    ((blt_int32u)0x00000020)

#if (BOOT_GATE_CAN_ENABLE > 0)
blt_bool commandSend;
#endif /* BOOT_GATE_CAN_ENABLE > 0 */


/****************************************************************************************
* Register definitions
****************************************************************************************/
/** \brief Macro for accessing CAN controller registers. */
#define CANx             ((tCanRegs *) (blt_int32u)0x40006400)


/****************************************************************************************
* Type definitions
****************************************************************************************/
/** \brief Structure type for grouping CAN bus timing related information. */
typedef struct t_can_bus_timing
{
  blt_int8u tseg1;                                    /**< CAN time segment 1          */
  blt_int8u tseg2;                                    /**< CAN time segment 2          */
} tCanBusTiming;


/****************************************************************************************
* Local constant declarations
****************************************************************************************/
/** \brief CAN bittiming table for dynamically calculating the bittiming settings.
 *  \details According to the CAN protocol 1 bit-time can be made up of between 8..25
 *           time quanta (TQ). The total TQ in a bit is SYNC + TSEG1 + TSEG2 with SYNC
 *           always being 1. The sample point is (SYNC + TSEG1) / (SYNC + TSEG1 + SEG2) *
 *           100%. This array contains possible and valid time quanta configurations with
 *           a sample point between 68..78%.
 */
static const tCanBusTiming canTiming[] =
{                       /*  TQ | TSEG1 | TSEG2 | SP  */
                        /* ------------------------- */
    {  5, 2 },          /*   8 |   5   |   2   | 75% */
    {  6, 2 },          /*   9 |   6   |   2   | 78% */
    {  6, 3 },          /*  10 |   6   |   3   | 70% */
    {  7, 3 },          /*  11 |   7   |   3   | 73% */
    {  8, 3 },          /*  12 |   8   |   3   | 75% */
    {  9, 3 },          /*  13 |   9   |   3   | 77% */
    {  9, 4 },          /*  14 |   9   |   4   | 71% */
    { 10, 4 },          /*  15 |  10   |   4   | 73% */
    { 11, 4 },          /*  16 |  11   |   4   | 75% */
    { 12, 4 },          /*  17 |  12   |   4   | 76% */
    { 12, 5 },          /*  18 |  12   |   5   | 72% */
    { 13, 5 },          /*  19 |  13   |   5   | 74% */
    { 14, 5 },          /*  20 |  14   |   5   | 75% */
    { 15, 5 },          /*  21 |  15   |   5   | 76% */
    { 15, 6 },          /*  22 |  15   |   6   | 73% */
    { 16, 6 },          /*  23 |  16   |   6   | 74% */
    { 16, 7 },          /*  24 |  16   |   7   | 71% */
    { 16, 8 }           /*  25 |  16   |   8   | 68% */
};


/************************************************************************************//**
** \brief     Search algorithm to match the desired baudrate to a possible bus
**            timing configuration.
** \param     baud The desired baudrate in kbps. Valid values are 10..1000.
** \param     prescaler Pointer to where the value for the prescaler will be stored.
** \param     tseg1 Pointer to where the value for TSEG2 will be stored.
** \param     tseg2 Pointer to where the value for TSEG2 will be stored.
** \return    BLT_TRUE if the CAN bustiming register values were found, BLT_FALSE
**            otherwise.
**
****************************************************************************************/
static blt_bool CanGetSpeedConfig(blt_int16u baud, blt_int16u *prescaler,
                                  blt_int8u *tseg1, blt_int8u *tseg2)
{
  blt_int8u  cnt;

  /* loop through all possible time quanta configurations to find a match */
  for (cnt=0; cnt < sizeof(canTiming)/sizeof(canTiming[0]); cnt++)
  {
    if (((BOOT_CPU_SYSTEM_SPEED_KHZ/2) % (baud*(canTiming[cnt].tseg1+canTiming[cnt].tseg2+1))) == 0)
    {
      /* compute the prescaler that goes with this TQ configuration */
      *prescaler = (BOOT_CPU_SYSTEM_SPEED_KHZ/2)/(baud*(canTiming[cnt].tseg1+canTiming[cnt].tseg2+1));

      /* make sure the prescaler is valid */
      if ( (*prescaler > 0) && (*prescaler <= 1024) )
      {
        /* store the bustiming configuration */
        *tseg1 = canTiming[cnt].tseg1;
        *tseg2 = canTiming[cnt].tseg2;
        /* found a good bus timing configuration */
        return BLT_TRUE;
      }
    }
  }
  /* could not find a good bus timing configuration */
  return BLT_FALSE;
} /*** end of CanGetSpeedConfig ***/


/************************************************************************************//**
** \brief     Initializes the CAN controller and synchronizes it to the CAN bus.
** \return    none.
**
****************************************************************************************/
void CanInit(void)
{
  blt_int16u prescaler;
  blt_int8u  tseg1, tseg2;
  blt_bool   result;

#if (BOOT_GATE_CAN_ENABLE > 0)
  commandSend = BLT_FALSE;
#endif /* BOOT_GATE_CAN_ENABLE > 0 */

  /* the current implementation supports CAN1. throw an assertion error in case a
   * different CAN channel is configured.
   */
  ASSERT_CT(BOOT_COM_CAN_CHANNEL_INDEX == 0);
  /* obtain bittiming configuration information */
  result = CanGetSpeedConfig(BOOT_COM_CAN_BAUDRATE/1000, &prescaler, &tseg1, &tseg2);
  ASSERT_RT(result == BLT_TRUE);
  /* disable all can interrupt. this driver works in polling mode */
  CANx->IER = (blt_int32u)0;
  /* set request to reset the can controller */
  CANx->MCR |= CAN_BIT_RESET ;
  /* wait for acknowledge that the can controller was reset */
  while ((CANx->MCR & CAN_BIT_RESET) != 0)
  {
    /* keep the watchdog happy */
    CopService();
  }
  /* exit from sleep mode, which is the default mode after reset */
  CANx->MCR &= ~CAN_BIT_SLEEP;
  /* set request to enter initialisation mode */
  CANx->MCR |= CAN_BIT_INRQ ;
  /* wait for acknowledge that initialization mode was entered */
  while ((CANx->MSR & CAN_BIT_INAK) == 0)
  {
    /* keep the watchdog happy */
    CopService();
  }
  /* configure the bittming */
  CANx->BTR = (blt_int32u)((blt_int32u)(tseg1 - 1) << 16) | \
              (blt_int32u)((blt_int32u)(tseg2 - 1) << 20) | \
              (blt_int32u)(prescaler - 1);
  /* set request to leave initialisation mode */
  CANx->MCR &= ~CAN_BIT_INRQ;
  /* wait for acknowledge that initialization mode was exited */
  while ((CANx->MSR & CAN_BIT_INAK) != 0)
  {
    /* keep the watchdog happy */
    CopService();
  }
  /* enter initialisation mode for the acceptance filter */
  CANx->FMR |= CAN_BIT_FINIT;
  /* deactivate filter 0 */
  CANx->FA1R &= ~CAN_BIT_FILTER0;
  /* 32-bit scale for the filter */
  CANx->FS1R |= CAN_BIT_FILTER0;
  /* open up the acceptance filter to receive all messages */
  CANx->sFilterRegister[0].FR1 = 0;
  CANx->sFilterRegister[0].FR2 = 0;
  /* select id/mask mode for the filter */
  CANx->FM1R &= ~CAN_BIT_FILTER0;
  /* FIFO 0 assignation for the filter */
  CANx->FFA1R &= ~CAN_BIT_FILTER0;
  /* filter activation */
  CANx->FA1R |= CAN_BIT_FILTER0;
  /* leave initialisation mode for the acceptance filter */
  CANx->FMR &= ~CAN_BIT_FINIT;
} /*** end of CanInit ***/


/************************************************************************************//**
** \brief     Transmits a packet formatted for the communication interface.
** \param     data     Pointer to byte array with data that it to be transmitted.
** \param     len      Number of bytes that are to be transmitted.
** \param     deviceID ID of the device the data has to be sent to.
** \return    none.
**
****************************************************************************************/
void CanTransmitPacket(blt_int8u *data, blt_int8u len, blt_int32u deviceID)
{
  /* make sure that transmit mailbox 0 is available */
  ASSERT_RT((CANx->TSR&CAN_BIT_TME0) == CAN_BIT_TME0);

  blt_int32u address;
#if (BOOT_GATE_CAN_ENABLE > 0)
  if (deviceID == 0) {
#endif /* BOOT_GATE_CAN_ENABLE > 0 */
    address = (blt_int32u)BOOT_COM_CAN_TX_MSG_ID;
#if (BOOT_GATE_CAN_ENABLE > 0)
    commandSend = BLT_FALSE;
  } else {
    address = (blt_int32u)BOOT_COM_CAN_RX_MSG_ID;
    commandSend = BLT_TRUE;
  }
#endif /* BOOT_GATE_CAN_ENABLE > 0 */

  /* init variables */
  blt_int8u canData[8];
  blt_int8u restLen = len;
  blt_int8u canIdx = 0;
  blt_int32u ackMsgId;
  blt_int8u ackMsgLen;

  /* send the given package in 8 byte packages */
  while (restLen > 0) {
    /* build the message identifier */
    CANx->sTxMailBox[0].TIR &= CAN_BIT_TXRQ;
    CANx->sTxMailBox[0].TIR |= (address << 21);

    /* if this is the first transmission of this packet */
    if (restLen == len) {

      /* store the message date length code (DLC) */
      CANx->sTxMailBox[0].TDTR = (restLen > 4) ? 8 : restLen+1+4;

      /* store the device ID */
      canData[0] = (blt_int8u)(0xFF & (deviceID >> 0));
      canData[1] = (blt_int8u)(0xFF & (deviceID >> 8));
      canData[2] = (blt_int8u)(0xFF & (deviceID >> 16));
      canData[3] = (blt_int8u)(0xFF & (deviceID >> 24));

      /* store the remaining packet length */
      canData[4] = restLen;

      canIdx = 5;
    }
    /* if this is a succeeding transmission of this packet */
    else {

      /* store the message date length code (DLC) */
      CANx->sTxMailBox[0].TDTR = (restLen > 7) ? 8 : restLen+1;

      /* store the remaining packet length */
      canData[0] = restLen;

      canIdx = 1;
    }

    /* store the packet payload */
    while (restLen > 0 && canIdx < 8) {
      canData[canIdx] = data[len-restLen];
      canIdx++;
      restLen--;
    }
    /* fill rest with nulls */
    while (canIdx < 8) {
      canData[canIdx] = 0;
      canIdx++;
    }

    /* store the message data bytes */
    CANx->sTxMailBox[0].TDLR = (((blt_int32u)canData[3] << 24) | \
                                ((blt_int32u)canData[2] << 16) | \
                                ((blt_int32u)canData[1] <<  8) | \
                                ((blt_int32u)canData[0]));
    CANx->sTxMailBox[0].TDHR = (((blt_int32u)canData[7] << 24) | \
                                ((blt_int32u)canData[6] << 16) | \
                                ((blt_int32u)canData[5] <<  8) | \
                                ((blt_int32u)canData[4]));

    /* request the start of message transmission */
    CANx->sTxMailBox[0].TIR |= CAN_BIT_TXRQ;
    /* wait for transmit completion */
    while ((CANx->TSR&CAN_BIT_TME0) == 0)
    {
      /* keep the watchdog happy */
      CopService();
    }

    /* wait for the acknowledgement transmission */
    while ((CANx->RF0R&(blt_int32u)0x00000003) == 0) {
      /* keep the watchdog happy */
      CopService();
    }

    /* read out the message identifier, length and payload */
    ackMsgId = (blt_int32u)0x000007FF & (CANx->sFIFOMailBox[0].RIR >> 21);
    ackMsgLen = (blt_int8u)0x0F & CANx->sFIFOMailBox[0].RDTR;
    canData[0] = (blt_int8u)0xFF & CANx->sFIFOMailBox[0].RDLR;
    canData[1] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 8);
    canData[2] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 16);
    canData[3] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 24);
    canData[4] = (blt_int8u)0xFF & CANx->sFIFOMailBox[0].RDHR;
    canData[5] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 8);
    canData[6] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 16);
    canData[7] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 24);

#if (CAN_DEBUG > 0)
    /* verification:
     * - ID must be "address & BOOT_COM_CAN_MSG_ACK"
     * - length must be 1
     * - canData[0] must b equal to restLen
     */
    if (ackMsgId != (address | (blt_int32u)BOOT_COM_CAN_MSG_ACK) ||
        ackMsgLen != 1 ||
        canData[0] != restLen) {
      /* some error occurred */
      blinkSOS(1);
      msleep(500);
      visualizeData((blt_int8u*)&ackMsgId, 2, 1);
      msleep(500);
      visualizeByte(ackMsgLen, 1);
      msleep(500);
      visualizeByte(canData[0], 1);
      msleep(500);
      visualizeByte(restLen, 1);
      blinkSOSinf();
    }
#endif /* CAN_DEBUG > 0 */

    /* release FIFO0 */
    CANx->RF0R |= CAN_BIT_RFOM0;

    /* modify address so that receivers can filter */
    address |= (blt_int32u)BOOT_COM_CAN_MSG_SUBSEQUENT;
  }
} /*** end of CanTransmitPacket ***/


/************************************************************************************//**
** \brief     Receives a communication interface packet if one is present.
** \param     data Pointer to byte array where the data is to be stored.
** \return    Length of message (if the message is invalid, the length will be 0).
**
****************************************************************************************/
blt_int8u CanReceivePacket(blt_int8u *data)
{
  blt_int32u rxMsgId;
  blt_bool   result = BLT_FALSE;
  blt_int8u  length = 0;

  static blt_int8u readData[BOOT_COM_RX_MAX_DATA];
  static blt_int8u receivedLen = 0;
  static blt_int8u lastLen = 0;
  static blt_int8u toReceive = 0;
  blt_int8u canData[8];
  blt_int8u restLen;
  blt_int8u canLength;
  blt_int8u idx;

  blt_int32u compID;
#if (BOOT_GATE_CAN_ENABLE > 0)
  if (commandSend == BLT_TRUE) {
    compID = (blt_int32u)BOOT_COM_CAN_TX_MSG_ID;
  } else {
#endif /* BOOT_GATE_CAN_ENABLE > 0 */
    compID = (blt_int32u)BOOT_COM_CAN_RX_MSG_ID;
#if (BOOT_GATE_CAN_ENABLE > 0)
  }
#endif /* BOOT_GATE_CAN_ENABLE > 0 */

  /* check if a new message was received  or is more to come */
  while ((CANx->RF0R&(blt_int32u)0x00000003) > 0 || receivedLen < toReceive)
  {
    /* wait for a transmission if required */
    while (receivedLen < toReceive && (CANx->RF0R&(blt_int32u)0x00000003) == 0) {
      /* keep the watchdog happy */
      CopService();
    }

    /* read out the message identifier */
    rxMsgId = (blt_int32u)0x000007FF & (CANx->sFIFOMailBox[0].RIR >> 21);
    /* is this the packet identifier */
    if (rxMsgId == compID)
    {
      result = BLT_TRUE;

      /* save length */
      canLength = (blt_int8u)0x0F & CANx->sFIFOMailBox[0].RDTR;
      /* store the received packet data */
      canData[0] = (blt_int8u)0xFF & CANx->sFIFOMailBox[0].RDLR;
      canData[1] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 8);
      canData[2] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 16);
      canData[3] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDLR >> 24);
      canData[4] = (blt_int8u)0xFF & CANx->sFIFOMailBox[0].RDHR;
      canData[5] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 8);
      canData[6] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 16);
      canData[7] = (blt_int8u)0xFF & (CANx->sFIFOMailBox[0].RDHR >> 24);

#if (CAN_DEBUG > 0)
      if ((receivedLen == 0 && (rxMsgId & (blt_int32u)BOOT_COM_CAN_MSG_SUBSEQUENT) != 0) ||
          (receivedLen > 0 && (rxMsgId & (blt_int32u)BOOT_COM_CAN_MSG_SUBSEQUENT) == 0)) {
        blinkSOS(4);
        msleep(500);
        visualizeByte(toReceive, 1);
        msleep(500);
        visualizeByte(receivedLen, 1);
        msleep(500);
        visualizeData((blt_int8u*)&rxMsgId, 2, 1);
        msleep(500);
        blinkSOSinf();
      }
#endif /* CAN_DEBUG > 0 */

      /* if this is the first transmission of this packet */
      if (receivedLen == 0) {
        /* abort if the message was meant for someone else */
        blt_int32u deviceID = (((blt_int32u)canData[3]) << 24) | \
                              (((blt_int32u)canData[2]) << 16) | \
                              (((blt_int32u)canData[1]) <<  8) | \
                              (((blt_int32u)canData[0]));
#if (BOOT_GATE_ENABLE > 0)
        if ((commandSend == BLT_TRUE && deviceID == 0) ||
            (commandSend != BLT_TRUE && (deviceID == (blt_int32u)BOOT_COM_DEVICE_ID) || deviceID == (blt_int32u)BOOT_COM_DEVICE_LEGACY_ID)) {
#else
        if (deviceID == (blt_int32u)BOOT_COM_DEVICE_ID ||
            deviceID == (blt_int32u)BOOT_COM_DEVICE_LEGACY_ID) {
#endif
          /* store length of the packet */
          toReceive = canData[4];
          lastLen = canData[4];
          idx = 5;

          /* modify the listening address for filtering of subsequent transmissions */
          compID |= (blt_int32u)BOOT_COM_CAN_MSG_SUBSEQUENT;
        } else {
          /* release FIFO0 */
          CANx->RF0R |= CAN_BIT_RFOM0;
          break;
        }
      }
      /* if this is a subsequent transmission of a packet */
      else {
        /* store rest length and check if possible */
        restLen = canData[0];
#if (CAN_DEBUG > 0)
        if (restLen != toReceive-receivedLen ||
            canLength > restLen+1 ||
            lastLen-restLen != ((lastLen==toReceive) ? 3 : 7)) {
          /* transmission has been lost */
          blinkSOS(2);
          msleep(500);
          visualizeData((blt_int8u*)&rxMsgId, 4, 1);
          msleep(500);
          visualizeByte(toReceive, 1);
          msleep(500);
          visualizeByte(receivedLen, 1);
          msleep(500);
          visualizeByte(lastLen, 1);
          msleep(500);
          visualizeByte(restLen, 1);
          msleep(500);
          visualizeByte(canLength, 1);
          msleep(500);
          visualizeData(readData, receivedLen, 1);
          blinkSOSinf();
        }
#endif /* CAN_DEBUG > 0 */
        lastLen = restLen;
        idx = 1;
      }

      /* store the payload */
      for (; idx < canLength; idx++) {
        readData[receivedLen] = canData[idx];
        receivedLen++;
      }

      /* release FIFO0 */
      CANx->RF0R |= CAN_BIT_RFOM0;

      /* send acknowledgement */
      CANx->sTxMailBox[0].TIR &= CAN_BIT_TXRQ;
      CANx->sTxMailBox[0].TIR |= ((rxMsgId | (blt_int32u)BOOT_COM_CAN_MSG_ACK) << 21);
      CANx->sTxMailBox[0].TDTR = 1;
      CANx->sTxMailBox[0].TDLR = (((blt_int32u)0 << 24) | \
                                  ((blt_int32u)0 << 16) | \
                                  ((blt_int32u)0 <<  8) | \
                                  ((blt_int32u)(toReceive-receivedLen)));
      CANx->sTxMailBox[0].TDHR = (((blt_int32u)0 << 24) | \
                                  ((blt_int32u)0 << 16) | \
                                  ((blt_int32u)0 <<  8) | \
                                  ((blt_int32u)0));

      /* request the start of message transmission */
      CANx->sTxMailBox[0].TIR |= CAN_BIT_TXRQ;
      /* wait for transmit completion */
      while ((CANx->TSR&CAN_BIT_TME0) == 0)
      {
        /* keep the watchdog happy */
        CopService();
      }

      /* check if full package has been received */
      if (receivedLen == toReceive) {
#if (BOOT_GATE_CAN_ENABLE > 0)
        commandSend = BLT_FALSE;
#endif /* BOOT_GATE_CAN_ENABLE > 0 */
        for (idx = 0; idx < toReceive; idx++) {
          data[idx] = readData[idx];
        }
        length = toReceive;
        /* reset static variables */
        receivedLen = 0;
        toReceive = 0;
        break;
      } else {
#if (CAN_DEBUG > 0)
        if (receivedLen > toReceive) {
          /* something strange happened */
          blinkSOS(3);
          msleep(500);
          visualizeByte(toReceive, 1);
          msleep(500);
          visualizeByte(receivedLen, 1);
          blinkSOSinf();
        }
#endif /* CAN_DEBUG > 0 */
        length = 0;
      }
    } else {
      /* release FIFO0 */
      CANx->RF0R |= CAN_BIT_RFOM0;
    }
  }

  return length;
} /*** end of CanReceivePacket ***/
#endif /* BOOT_COM_CAN_ENABLE > 0  || BOOT_GATE_CAN_ENABLE > 0 */


/*********************************** end of can.c **************************************/
