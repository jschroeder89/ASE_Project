/************************************************************************************//**
* \file         main.c
* \brief        SerialBoot command line demonstration program for OpenBLT.
* \ingroup      SerialBoot
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2014  by Feaser    http://www.feaser.com    All rights reserved
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
#include <assert.h>                                   /* assertion module              */
#include <sb_types.h>                                 /* C types                       */
#include <stdio.h>                                    /* standard I/O library          */
#include <string.h>                                   /* string library                */
#include "xcpmaster.h"                                /* XCP master protocol module    */
#include "srecord.h"                                  /* S-record file handling        */
#include "timeutil.h"                                 /* time utility module           */
#include "stdlib.h"                                   /* ascii to integral conversion  */


/****************************************************************************************
* Function prototypes
****************************************************************************************/
static void     DisplayProgramInfo(void);
static void     DisplayProgramUsage(void);
static void     DisplayProgramInput(void);
static void     printErrors(void);
static sb_uint8 ParseCommandLine(sb_int32 argc, sb_char *argv[]);
static sb_int32 startFlashMode(void);
static sb_int32 flashProgram(void);
static sb_int32 startUserProgram(void);
static sb_int32 originalMainPart(void);
static sb_int32 openSerialPort(void);
static sb_int32 openSerialPortConnect(sb_uint8 start);
static sb_int32 closeSerialPort(void);
static sb_int32 buildConnection(sb_uint8 start);
static sb_int32 closeConnection(sb_file *hSrecord);
static sb_int32 closeConnectionWithReset(sb_file *hSrecord);
static sb_int32 prepareProgrammingSession(sb_file *hSrecord, tSrecordParseResults *fileParseResults);
static sb_int32 programCode(sb_file *hSrecord, tSrecordParseResults *fileParseResults, tSrecordLineParseResults *lineParseResults);

/****************************************************************************************
* Macro definitions
****************************************************************************************/
/** \brief Program return code if all went ok. */
#define PROG_RESULT_OK    (0)

/** \brief Program return code if an error occurred. */
#define PROG_RESULT_ERROR (1)

/* mode definitions of the program status */
#define PROGSTATE_ORIGIN           0
#define PROGSTATE_STARTFLASHMODE   1
#define PROGSTATE_RESET            2
#define PROGSTATE_FLASHPROGRAM     3
#define PROGSTATE_SETFLAGS         4

/* error definitions */
#define ERROR_NO                   0
#define ERROR_FILE                 1
#define ERROR_DEVICE               2
#define ERROR_UNKNOWN              255

/* maximal space for flashing programs */
#define MAX_COUNT_FLASHING_PROGRAMS 5


/****************************************************************************************
* Local data declarations
****************************************************************************************/
/** \brief Name of the serial device, such as COM4 or /dev/ttyUSB0. */
static sb_char serialDeviceName[32];

/** \brief Id of the bluetooth device, such as 01:23:45:67:89:AB. */
static sb_char bluetoothID[18];

/** \brief Serial communication speed in bits per second. */
static sb_uint32 serialBaudrate;

/** \brief Name of the S-record file. */
static sb_char *srecordFileName;

/* program status */
static sb_uint8 prog_state = 1;

/* id of flashing target */
static sb_uint32 flashingTargetID = 0;

/* list of program names */
static sb_char *srecordFileNames[MAX_COUNT_FLASHING_PROGRAMS];

/* list of target ids */
static sb_uint32 flashingTargetIDs[MAX_COUNT_FLASHING_PROGRAMS];

/* list of errors */
static sb_uint8 errors[MAX_COUNT_FLASHING_PROGRAMS];

/* last error */
static sb_uint8 errorDetected = ERROR_NO;

/* program counter */
static sb_uint8 countFlashingPrograms = 0;

/* RTS register value */
static sb_uint8 registerRTSvalue = 0;

/* Type of communication is about UART, not Bluetooth */
static sb_uint8 comIsUart = SB_TRUE;


/************************************************************************************//**
** \brief     Program entry point.
** \param     argc Number of program parameters.
** \param     argv array to program parameter strings.
** \return    0 on success, > 0 on error.
**
****************************************************************************************/
sb_int32 main(sb_int32 argc, sb_char *argv[])
{
  /* disable buffering for the standard output to make sure printf does not wait until
   * a newline character is detected before outputting text on the console.
   */
  setbuf(stdout, SB_NULL);

  /* inform user about the program */
  DisplayProgramInfo();

  /* start out by making sure program was started with the correct parameters */
  if (ParseCommandLine(argc, argv) == SB_FALSE)
  {
    /* parameters invalid. inform user about how this program works */
    DisplayProgramUsage();
    return PROG_RESULT_ERROR;
  }

  DisplayProgramInput();

  if (openSerialPortConnect(0) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* initialize errors */
  sb_uint8 errorIdx;
  for (errorIdx=0; errorIdx<MAX_COUNT_FLASHING_PROGRAMS; errorIdx++) {
    errors[errorIdx] = ERROR_NO;
  }

  flashingTargetID = 0;
  if (startFlashMode() == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  sb_uint8 flashIdx;
  for (flashIdx=0; flashIdx<countFlashingPrograms; flashIdx++) {
    errorDetected = ERROR_NO;
    flashingTargetID = flashingTargetIDs[flashIdx];
    srecordFileName = srecordFileNames[flashIdx];
    printf("\nFlash %i: Flashing %s on device 0x%08X\n\n", flashIdx+1, srecordFileName, flashingTargetID);

    if (flashProgram() == PROG_RESULT_ERROR) {
      closeSerialPort();
      openSerialPortConnect(0);
    }
    errors[flashIdx] = errorDetected;
  }

  flashingTargetID = 0;
  if (startUserProgram() == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  printErrors();


  return PROG_RESULT_OK;
} /*** end of main ***/


/************************************************************************************//**
** \brief     Outputs information to the user about this program.
** \return    none.
**
****************************************************************************************/
static void DisplayProgramInfo(void)
{
  printf("---------------------------------------------------------------------------\n");
  printf("SerialBoot version 1.00. Performs firmware updates via the serial port     \n");
  printf("for a microcontroller based system that runs the OpenBLT bootloader.       \n");
  printf("                                                                           \n");
  printf("Copyright (c) by Feaser  http://www.feaser.com                             \n");
  printf("---------------------------------------------------------------------------\n");
  printf("This tool was modified for the 'Autonomous Mini Robot' - AMiRo.            \n");
  printf("Copyright (c) 2016..2018  Marvin Barther, Thomas Schoepping, and Stefan    \n");
  printf("                          Herbrechtsmeier                                  \n");
  printf("This is free software; see the source for copying conditions. There is NO  \n");
  printf("warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n");
  printf("The development of this software was supported by the Excellence Cluster   \n");
  printf("EXC 227 Cognitive Interaction Technology. The Excellence Cluster EXC 227 is\n");
  printf("a grant of the Deutsche Forschungsgemeinschaft (DFG) in the context of the \n");
  printf("German Excellence Initiative.                                              \n");
  printf("---------------------------------------------------------------------------\n");
} /*** end of DisplayProgramInfo ***/


/************************************************************************************//**
** \brief     Outputs information to the user about how to use this program.
** \return    none.
**
****************************************************************************************/
static void DisplayProgramUsage(void)
{
  printf("Usage: SerialBoot -d<device> -b<baudrate> <s-record file>/(-T<flash-id> <s-record file>)\n\n");
  printf("Example 1:  SerialBoot -h\n");
#ifdef PLATFORM_WIN32
  printf("Example 2:  SerialBoot -dCOM4 -b57600 -T3 firmware.srec\n");
  printf("Example 3:  SerialBoot -dCOM4 -b57600 -T0 firm1.srec -T3 firm2.srec\n");
  printf("Example 4:  SerialBoot -dCOM4 -b57600 firmware.srec\n\n");
  printf("WARNING: SerialBoot is not really functional with windows! Please use\n");
  printf("         the Linux version.\n\n");
  printf("  -> In all four cases it connects to COM4 and configures a communication\n");
#else
  printf("Example 2:  SerialBoot -d/dev/ttyUSB0 -b57600 -T3 firmware.srec\n");
  printf("Example 3:  SerialBoot -d/dev/ttyUSB0 -b57600 -T0 firm1.srec -T3 firm2.srec\n");
  printf("Example 4:  SerialBoot -d/dev/ttyUSB0 -b57600 firmware.srec\n\n");
  printf("  -> In all four cases it connects to ttyUSB0 and configures a communication\n");
#endif
  printf("     speed of 57600 bits/second. The bootloaders will be set into flashing\n");
  printf("     mode and will be reset after flashing the last binary file.\n");
  printf("  -> In the first example this help will be opened\n");
  printf("  -> In the second example the binary file 'firmware.srec' will be flashed on\n");
  printf("     device 0x03.\n");
  printf("  -> In the third example the binary file 'firm1.srec' will be flashed on\n");
  printf("     device 0x00 and the other binary file 'firm2.srec' on device 0x03.\n");
  printf("  -> In the last example the standard configuration is shown. The binary file\n");
  printf("     'firmware.srec' will be flashed on device 0x00. It equals the\n");
  printf("     configuration '-T0 firmware.srec'.\n");
  printf("  -> Don't forget that you always have to give the parameters -d and -b.\n");
  printf("  -> There is also the parameter -a for giving an address for bluetooth\n");
  printf("     communication, but this part is not functional yet.\n");
  printf("--------------------------------------------------------------------------------\n");
} /*** end of DisplayProgramUsage ***/



static void DisplayProgramInput(void) {
#ifdef PLATFORM_WIN32
  printf("WARNING: SerialBoot is designed for Linux. While using Windows the correct\n");
  printf("         functionality can not be guaranteed! Please use Linux for using best\n");
  printf("         performance and savety.\n");
  printf("\n--------------------------------------------------------------------------------\n\n");
#endif
  printf("\nOpen device at %s @ %u bits/s and RTS=%i\n\n", serialDeviceName, serialBaudrate, registerRTSvalue);
  printf("Programming will start immediately in following order:\n");
  sb_uint8 idx;
  for (idx=0; idx<countFlashingPrograms; idx++) {
    printf("  %i) device 0x%08X: %s\n", idx+1, flashingTargetIDs[idx], srecordFileNames[idx]);
  }
}


/************************************************************************************//**
** \brief     Parses the command line arguments. A fixed amount of arguments is expected.
**            The program should be called as:
**              SerialBoot -d[device] -b[baudrate] [s-record file]
** \param     argc Number of program parameters.
** \param     argv array to program parameter strings.
** \return    SB_TRUE on success, SB_FALSE otherwise.
**
****************************************************************************************/
static sb_uint8 ParseCommandLine(sb_int32 argc, sb_char *argv[])
{
  sb_uint8 paramIdx;
  sb_uint8 paramAfound = SB_FALSE;
  sb_uint8 paramDfound = SB_FALSE;
  sb_uint8 paramBfound = SB_FALSE;
  sb_uint8 paramTfound = SB_FALSE;
  sb_uint8 paramRTSfound = SB_FALSE;
  sb_uint8 paramXfound = SB_FALSE;
  sb_uint8 srecordfound = SB_FALSE;

  /* make sure the right amount of arguments are given */
  if (argc < 4 || argc > 14)
  {
    return SB_FALSE;
  }

  /* loop through all the command lina parameters, just skip the 1st one because this
   * is the name of the program, which we are not interested in.
   */
  for (paramIdx=1; paramIdx<argc; paramIdx++)
  {
    /* is this the device name? */
    if ( (argv[paramIdx][0] == '-') && (argv[paramIdx][1] == 'd') && (paramDfound == SB_FALSE) && (paramTfound == SB_FALSE) && (paramAfound == SB_FALSE))
    {
      /* copy the device name and set flag that this parameter was found */
      strcpy(serialDeviceName, &argv[paramIdx][2]);
      paramDfound = SB_TRUE;
      comIsUart = SB_TRUE;
    }
    /* is this the bluetooth id? */
/*
    else if ( (argv[paramIdx][0] == '-') && (argv[paramIdx][1] == 'a') && (paramDfound == SB_FALSE) && (paramTfound == SB_FALSE) && (paramAfound == SB_FALSE))
    {
      if ((argv[paramIdx][4] != ':') || (argv[paramIdx][7] != ':') || (argv[paramIdx][10] != ':') || (argv[paramIdx][13] != ':') || (argv[paramIdx][16] != ':')) {
        printf("The bluetooth ID %s has not the format xx:xx:xx:xx:xx:xx\n", &argv[paramIdx][2]);
        return SB_FALSE;
      }
      // copy the bluetooth id and set flag that this parameter was found
      strcpy(serialDeviceName, &argv[paramIdx][2]);
      paramAfound = SB_TRUE;
      comIsUart = SB_FALSE;
      // TODO accept bluetooth communication
      printf("Bluetooth communication is not available yet!");
      return SB_FALSE;
    }
*/
    /* is this the baudrate? */
    else if ( (argv[paramIdx][0] == '-') && (argv[paramIdx][1] == 'b') && (paramBfound == SB_FALSE) && (paramTfound == SB_FALSE) )
    {
      /* extract the baudrate and set flag that this parameter was found */
      sscanf(&argv[paramIdx][2], "%u", &serialBaudrate);
      paramBfound = SB_TRUE;
    }
    /* is this the RTS flag? */
    else if ( (argv[paramIdx][0] == '-') && (argv[paramIdx][1] == 'R') && (argv[paramIdx][2] == 'T') && (argv[paramIdx][3] == 'S') && (paramRTSfound == SB_FALSE) && (paramTfound == SB_FALSE) )
    {
      /* extract RTS flag [0,1] and set that this parameter was found */
      sb_char setValue[2];
      strcpy(setValue, &argv[paramIdx][4]);
      if (setValue[0] == 0x0) {
        registerRTSvalue = 0;
      } else {
        registerRTSvalue = ((int)setValue[0])-48;
      }
      if (registerRTSvalue > 1 || registerRTSvalue < 0 || setValue[1] != 0x0) {
        registerRTSvalue = 0;
        printf("Not allowed value for RTS register. RTS=[0,1]\n");
        return SB_FALSE;
      }
      paramRTSfound = SB_TRUE;
    }
    /* is this the target number? */
    else if ( (argv[paramIdx][0] == '-') && (argv[paramIdx][1] == 'T') && (paramTfound == SB_FALSE) && (paramXfound == SB_FALSE) && (paramDfound == SB_TRUE) && (paramBfound == SB_TRUE) )
    {
      if (countFlashingPrograms >= MAX_COUNT_FLASHING_PROGRAMS) {
        printf("too many programs to flash (-T)\n");
        return SB_FALSE;
      }
      /* prog status has to be set in flashing mode */
      paramTfound = SB_TRUE;
      prog_state = PROGSTATE_FLASHPROGRAM;
      sb_char choosenID[128];
      strcpy(choosenID, &argv[paramIdx][2]);
      /* Transform ascii numbers in string to an integer */
      flashingTargetID = 0;
      if (choosenID[0] == '0' && (choosenID[1] < '0' || choosenID[1] > '9')) {  // if the ID is something like "0x..." or "0o..."
        switch (choosenID[1]) {
          case 'x':
          case 'X':
            flashingTargetID = strtoul(&choosenID[2], NULL, 16);
            break;
          case 'o':
          case 'O':
            flashingTargetID = strtoul(&choosenID[2], NULL, 8);
            break;
          case 'b':
          case 'B':
            flashingTargetID = strtoul(&choosenID[2], NULL, 2);
            break;
          case 'd':
          case 'D':
            flashingTargetID = strtoul(&choosenID[2], NULL, 10);
            break;
          default:
            printf("could not interprete target ID\n");
            return SB_FALSE;
            break;
        }
      } else {
        flashingTargetID = strtoul(choosenID, NULL, 10);
      }
      if (flashingTargetID == 0) {
        printf("target ID invalid (0x%08X)\n", flashingTargetID);
        return SB_FALSE;
      }
      flashingTargetIDs[countFlashingPrograms] = flashingTargetID;
    }
    /* is this a srec file (a parameter without "-" in the beginning) */
    else if (argv[paramIdx][0] != '-')
    {
      if (countFlashingPrograms >= MAX_COUNT_FLASHING_PROGRAMS) {
        printf("too many programs to flash (without -T)\n");
        return SB_FALSE;
      }
      /* srecord file has been found */
      if (paramTfound == SB_FALSE) {
        paramTfound = SB_TRUE;
        flashingTargetIDs[countFlashingPrograms] = 0;
        prog_state = PROGSTATE_ORIGIN;
      }
      srecordFileNames[countFlashingPrograms] = &(argv[paramIdx][0]);
      countFlashingPrograms++;
      paramTfound = SB_FALSE;
      srecordfound = SB_TRUE;
    }
  }

  /* verify if all parameters were found */
  if ( ( ((paramDfound == SB_FALSE) || (paramBfound == SB_FALSE)) && (paramAfound == SB_FALSE)) || (srecordfound == SB_FALSE) )
  {
    printf("-a, -d, -b or srecord file missing, remember that you can only choose -a or -d\n");
    return SB_FALSE;
  }

  /* still here so the parsing was successful */
  return SB_TRUE;
} /*** end of ParseCommandLine ***/





static void printErrors(void) {
  sb_uint8 detected = 0;
  sb_uint8 errorIdx;
  for (errorIdx=0; errorIdx<MAX_COUNT_FLASHING_PROGRAMS; errorIdx++) {
    if (errors[errorIdx] != ERROR_NO) {
      detected++;
    }
  }
  if (detected == 0) {
    printf("\nEvery firmware successfully updated.\nSerialBoot finished without any errors.\n\n");
  } else {
    if (detected < countFlashingPrograms) {
      printf("\n%i ERRORs detected!\nOnly in the following %i cases SerialBoot updated the firmware:\n", detected, countFlashingPrograms-detected);
      for (errorIdx=0; errorIdx<countFlashingPrograms; errorIdx++) {
        if (errors[errorIdx] == ERROR_NO) {
          printf(" * Flash %i: %s on device 0x%08X\n", errorIdx+1, srecordFileNames[errorIdx], flashingTargetIDs[errorIdx]);
        }
      }
      printf("In the following %i cases SerialBoot could not update the firmware:\n", detected);
    } else {
      printf("\nERRORs detected!\nEvery firmware update by SerialBoot failed:\n");
    }
    for (errorIdx=0; errorIdx<countFlashingPrograms; errorIdx++) {
      if (errors[errorIdx] != ERROR_NO) {
        printf(" * ERROR in Flash %i: ", errorIdx+1);
        switch (errors[errorIdx]) {
          case ERROR_FILE:
            printf("The file %s couldn't be read!\n", srecordFileNames[errorIdx]); break;
          case ERROR_DEVICE:
            printf("No connection to device 0x%08X!\n", flashingTargetIDs[errorIdx]); break;
          default:
            printf("Couldn't specify the error. Please check output above!\n"); break;
        }
      }
    }
    printf("Please check the errors before trying to update again.\n\n");
  }
}





static sb_int32 startFlashMode(void) {

  /* -------------------- start flashing mode ---------------------------------------- */
  printf("Set main device on flashing mode\n");

  /* -------------------- open connection -------------------------------------------- */
  if (buildConnection(1) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  //printf("close...");

  /* -------------------- close connection ------------------------------------------- */
  if(closeConnection(NULL) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* all done */
  printf("Main device is now in flashing mode!\n\n");

  return PROG_RESULT_OK;
}



static sb_int32 flashProgram(void) {
  sb_file hSrecord;
  tSrecordParseResults fileParseResults;
  tSrecordLineParseResults lineParseResults;

  /* -------------------- start the firmware update procedure ------------------------ */
  printf("Starting firmware update for \"%s\" on device 0x%08X\n", srecordFileName, flashingTargetID);
  printf("Using %s @ %u bits/s\n", serialDeviceName, serialBaudrate);

  /* -------------------- prepare programming session -------------------------------- */
  if (prepareProgrammingSession(&hSrecord, &fileParseResults) == PROG_RESULT_ERROR) {
    errorDetected = ERROR_FILE;
    return PROG_RESULT_ERROR;
  }

  /* -------------------- open connection -------------------------------------------- */
  if (buildConnection(0) == PROG_RESULT_ERROR) {
    errorDetected = ERROR_DEVICE;
    return PROG_RESULT_ERROR;
  }

  /* -------------------- programming code ------------------------------------------- */
  if (programCode(&hSrecord, &fileParseResults, &lineParseResults) == PROG_RESULT_ERROR) {
    errorDetected = ERROR_UNKNOWN;
    return PROG_RESULT_ERROR;
  }

  /* -------------------- close connection ------------------------------------------- */
  if (closeConnection(&hSrecord) == PROG_RESULT_ERROR) {
    errorDetected = ERROR_UNKNOWN;
    return PROG_RESULT_ERROR;
  }

  /* -------------------- close the S-record file ------------------------------------ */
  SrecordClose(hSrecord);
  printf("Closed S-record file \"%s\"\n", srecordFileName);

  /* all done */
  printf("Firmware successfully updated!\n\n");

  return PROG_RESULT_OK;
}






static sb_int32 startUserProgram(void) {
  /* -------------------- start the user program ------------------------------------- */
  printf("Resetting all using %s @ %u bits/s\n", serialDeviceName, serialBaudrate);

  /* -------------------- open connection -------------------------------------------- */
  if (buildConnection(0) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* -------------------- close connection and reset --------------------------------- */
  if (closeConnectionWithReset(NULL) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* all done */
  printf("User programs have been started!\n\n");

  return PROG_RESULT_OK;
}




static sb_int32 originalMainPart(void) {
  sb_file hSrecord;
  tSrecordParseResults fileParseResults;
  tSrecordLineParseResults lineParseResults;

  /* -------------------- start the firmware update procedure ------------------------ */
  printf("Starting firmware update for \"%s\" using %s @ %u bits/s\n", srecordFileName, serialDeviceName, serialBaudrate);

  /* -------------------- prepare programming session -------------------------------- */
  if (prepareProgrammingSession(&hSrecord, &fileParseResults) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* -------------------- open serial port and open connection ----------------------- */
  if (openSerialPortConnect(0) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* -------------------- open connection -------------------------------------------- */
//  if (buildConnection(0) == PROG_RESULT_ERROR) {
//    return PROG_RESULT_ERROR;
//  }

  /* -------------------- programming code ------------------------------------------- */
  if (programCode(&hSrecord, &fileParseResults, &lineParseResults) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* -------------------- close connection and reset --------------------------------- */
  if (closeConnectionWithReset(&hSrecord) == PROG_RESULT_ERROR) {
    return PROG_RESULT_ERROR;
  }

  /* -------------------- close the S-record file ------------------------------------ */
  SrecordClose(hSrecord);
  printf("Closed S-record file \"%s\"\n", srecordFileName);

  /* all done */
  printf("Firmware successfully updated!\n\n");

  return PROG_RESULT_OK;
}





static sb_int32 openSerialPort(void) {
  /* -------------------- Open the serial port --------------------------------------- */
//  printf("Opening serial port %s...", serialDeviceName);
  if (XcpMasterInit(serialDeviceName, serialBaudrate, comIsUart) == SB_FALSE)
  {
    printf("ERROR\n");
    return PROG_RESULT_ERROR;
  }
//  printf("OK\n");
  return PROG_RESULT_OK;
}





static sb_int32 closeSerialPort(void) {
  /* -------------------- close the serial port -------------------------------------- */
  XcpMasterDeinit();
  return PROG_RESULT_OK;
}





static sb_int32 openSerialPortConnect(sb_uint8 start) {
  /* -------------------- Connect to XCP slave --------------------------------------- */
//  printf("Connecting to bootloader...");
  sb_uint8 connections = 0;
  if (XcpMasterInit(serialDeviceName, serialBaudrate, comIsUart) == SB_FALSE) {
    printf("Could not open serial port\n");
  } else {
    if (XcpMasterConnect(0) == SB_FALSE) {
      connections++;
      /* no response. prompt the user to reset the system */
      if (flashingTargetID == 0 && start > 0) {
        printf("Please connect main device to serial port %s...", serialDeviceName);
      } else  {
        printf("Try to connect to device...");
      }
    }
  }
  /* now keep retrying until we get a response */
  sb_uint16 timeoutIdx = 0;
  sb_uint16 timeoutEnd = 60;
  sb_uint8 noConnectionYet = SB_TRUE;
  while (noConnectionYet == SB_TRUE) {
    if (XcpMasterInit(serialDeviceName, serialBaudrate, comIsUart) == SB_FALSE) {
      if (timeoutIdx == timeoutEnd) {
        printf("\nTIMEOUT in building connection. Please start serial port of the main device ...");
      }
    } else {
      if (XcpMasterConnect(0) == SB_FALSE) {
        if (timeoutIdx == timeoutEnd) {
          printf("\nTIMEOUT in reset. Please reset the main device ...");
        }
      } else {
        noConnectionYet = SB_FALSE;
      }
    }
    /* delay a bit to not pump up the CPU load */
    TimeUtilDelayMs(20);
    timeoutIdx++;
  }
  if (connections > 0) {
    printf("OK\n");
  }

  return PROG_RESULT_OK;
}




static sb_int32 buildConnection(sb_uint8 start) {
  /* -------------------- Connect to XCP slave --------------------------------------- */
//  printf("Connecting to bootloader...");
  sb_uint8 connections = 0;
  if (XcpMasterConnect(flashingTargetID) == SB_FALSE) {
    connections++;
    /* no response. prompt the user to reset the system */
    if (flashingTargetID == 0 && start > 0) {
      printf("Please connect main device to serial port %s...", serialDeviceName);
    } else if (flashingTargetID == 0) {
      printf("Try to connect to device...");
    } else {
      printf("Try to connect to device 0x%08X...", flashingTargetID);
    }
  }
  /* now keep retrying until we get a response */
  sb_uint16 timeoutIdx = 0;
  sb_uint16 timeoutEnd = 50;
  while (XcpMasterConnect(flashingTargetID) == SB_FALSE) {
    if (timeoutIdx == timeoutEnd) {
      printf("\nTIMEOUT: No connection possible.\n\n");
      return PROG_RESULT_ERROR;
    }
    /* delay a bit to not pump up the CPU load */
    TimeUtilDelayMs(20);
    timeoutIdx++;
  }
  if (connections > 0) {
    printf("OK\n");
  }

  return PROG_RESULT_OK;
}




static sb_int32 closeConnection(sb_file *hSrecord) {
  /* -------------------- Disconnect from XCP slave and perform software reset ------- */
//  printf("Performing software reset...");
  if (XcpMasterDisconnect() == SB_FALSE)
  {
    printf("ERROR\n");
    if (hSrecord != NULL) {
      SrecordClose(*hSrecord);
    }
    return PROG_RESULT_ERROR;
  }
//  printf("OK\n");

  return PROG_RESULT_OK;
}




static sb_int32 closeConnectionWithReset(sb_file *hSrecord) {
  /* -------------------- Disconnect from XCP slave ---------------------------------- */
  printf("Resetting...");
  if (XcpMasterProgramReset() == SB_FALSE)
  {
    printf("ERROR\n");
    XcpMasterDisconnect();
    if (hSrecord != NULL) {
      SrecordClose(*hSrecord);
    }
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  closeSerialPort();
  printf("Closed serial port %s\n", serialDeviceName);

  return PROG_RESULT_OK;
}




static sb_int32 prepareProgrammingSession(sb_file *hSrecord, tSrecordParseResults *fileParseResults) {
  /* -------------------- validating the S-record file ------------------------------- */
  printf("Checking formatting of S-record file \"%s\"...", srecordFileName);
  if (SrecordIsValid(srecordFileName) == SB_FALSE)
  {
    printf("ERROR\n\n");
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  /* -------------------- opening the S-record file ---------------------------------- */
  printf("Opening S-record file \"%s\"...", srecordFileName);
  if ((*hSrecord = SrecordOpen(srecordFileName)) == SB_NULL)
  {
    printf("ERROR\n\n");
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  /* -------------------- parsing the S-record file ---------------------------------- */
  printf("Parsing S-record file \"%s\"...", srecordFileName);
  SrecordParse(*hSrecord, fileParseResults);
  printf("OK\n");
  printf("-> Lowest memory address:  0x%08x\n", fileParseResults->address_low);
  printf("-> Highest memory address: 0x%08x\n", fileParseResults->address_high);
  printf("-> Total data bytes: %u\n", fileParseResults->data_bytes_total);

  return PROG_RESULT_OK;
}




static sb_int32 programCode(sb_file *hSrecord, tSrecordParseResults *fileParseResults, tSrecordLineParseResults *lineParseResults) {
  /* -------------------- Prepare the programming session ---------------------------- */
  printf("Initializing programming session...");
  if (XcpMasterStartProgrammingSession() == SB_FALSE)
  {
    printf("ERROR\n");
    XcpMasterDisconnect();
    SrecordClose(*hSrecord);
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  /* -------------------- Erase memory ----------------------------------------------- */
  printf("Erasing %u bytes starting at 0x%08x...", fileParseResults->data_bytes_total, fileParseResults->address_low);
  if (XcpMasterClearMemory(fileParseResults->address_low, (fileParseResults->address_high - fileParseResults->address_low)) == SB_FALSE)
  {
    printf("ERROR\n");
    XcpMasterDisconnect();
    SrecordClose(*hSrecord);
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  /* -------------------- Program data ----------------------------------------------- */
  printf("Programming data. Please wait...");
  /* loop through all S-records with program data */
  while (SrecordParseNextDataLine(*hSrecord, lineParseResults) == SB_TRUE)
  {
    if (XcpMasterProgramData(lineParseResults->address, lineParseResults->length, lineParseResults->data) == SB_FALSE)
    {
      printf("ERROR\n");
      XcpMasterDisconnect();
      SrecordClose(*hSrecord);
      return PROG_RESULT_ERROR;
    }
  }
  printf("OK\n");

  /* -------------------- Stop the programming session ------------------------------- */
  printf("Finishing programming session...");
  if (XcpMasterStopProgrammingSession() == SB_FALSE)
  {
    printf("ERROR\n");
    XcpMasterDisconnect();
    SrecordClose(*hSrecord);
    return PROG_RESULT_ERROR;
  }
  printf("OK\n");

  return PROG_RESULT_OK;
}



/*********************************** end of main.c *************************************/
