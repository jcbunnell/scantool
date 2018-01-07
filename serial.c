#include <windows.h>
#ifdef WIN_PRINTF
#include <strsafe.h>
#else // WIN_PRINTF
#include <stdio.h>
#endif // WIN_PRINTF
#include <string.h>
#include <ctype.h>
#include "globals.h"
#include "serial.h"
#include "topwork.h"

extern COMPORT comport;
HANDLE CommHandle;
int serial_time_out;

int open_comport()
{
   DCB dcb;
   char temp_str[16];
   COMMTIMEOUTS timeouts;

   if (comport.status == READY)    // if the comport is open,
   {
      close_comport();    // close it
   }

#ifdef WIN_PRINTF
   StringCchPrintf(temp_str, sizeof(temp_str), "COM%i", comport.number);
#else // WIN_PRINTF
   sprintf(temp_str, "COM%i", comport.number);
#endif // WIN_PRINTF
   CommHandle = CreateFile(temp_str, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
   if (CommHandle == INVALID_HANDLE_VALUE)
   {
#ifdef WIN_PRINTF
      printf("Unable to open %s\n", temp_str);
#endif   // WIN_PRINTF
      comport.status = NOT_OPEN; //port was not open
      return -1; // return error
   }

   GetCommState(CommHandle, &dcb);
   dcb.BaudRate = comport.baud_rate;
   dcb.ByteSize = 8;
   dcb.StopBits = ONESTOPBIT;
   dcb.fParity = FALSE;
   dcb.Parity = NOPARITY;
   dcb.fOutxCtsFlow = FALSE;
   dcb.fOutxDsrFlow = FALSE;
   dcb.fOutX = FALSE;
   dcb.fInX = FALSE;
   dcb.fDtrControl = DTR_CONTROL_ENABLE;
   dcb.fRtsControl = RTS_CONTROL_ENABLE;
   dcb.fDsrSensitivity = FALSE;
   dcb.fErrorChar = FALSE;
   dcb.fAbortOnError = FALSE;
   SetCommState(CommHandle, &dcb);

   timeouts.ReadIntervalTimeout = MAXWORD;
   timeouts.ReadTotalTimeoutMultiplier = 0;
   timeouts.ReadTotalTimeoutConstant = 0;
   timeouts.WriteTotalTimeoutMultiplier = 0;
   timeouts.WriteTotalTimeoutConstant = 0;
   SetCommTimeouts(CommHandle, &timeouts);

   serial_time_out = FALSE;
   comport.status = READY;

   return 0; // everything is okay
}


void close_comport()
{
   if (comport.status == READY)    // if the comport is open, close it
   {
      PurgeComm(CommHandle, PURGE_TXCLEAR|PURGE_RXCLEAR);
      CloseHandle(CommHandle);
   }
   comport.status = NOT_OPEN;
}


void send_command(const char *command)
{
   char tx_buf[32];
   DWORD bytes_written;

#ifdef WIN_PRINTF
   StringCchPrintf(tx_buf, sizeof(tx_buf), "%s\r", command);  // Append CR to the command
#else // WIN_PRINTF
   sprintf(tx_buf, "%s\r", command);  // Append CR to the command
#endif // WIN_PRINTF

#ifdef LOG_COMMS
   write_comm_log("TX", tx_buf);
#endif

   PurgeComm(CommHandle, PURGE_TXCLEAR|PURGE_RXCLEAR);
   WriteFile(CommHandle, tx_buf, (DWORD) strlen(tx_buf), &bytes_written, 0);
}


int read_comport(char *response, DWORD *numBytes)
{
//   char *prompt_pos = NULL;

   DWORD errors;
   COMSTAT stat;

   *numBytes = 0;
   response[0] = '\0';
   ClearCommError(CommHandle, &errors, &stat);
   if (stat.cbInQue > 0)
   {
      ReadFile(CommHandle, response, stat.cbInQue, numBytes, 0);
   }
   response[*numBytes] = '\0';

   if (*numBytes == 0)  // if the string is empty,
   {
      return EMPTY;
   }
   else                         //otherwise,
   {
#ifdef LOG_COMMS
      write_comm_log("RX", response);
#endif
      return DATA;
   }
}

DWORD compress_response(char *msg, DWORD bufSize)
{
   DWORD cIndex=0;
   DWORD mIndex=0;
   // go until end of input string
   while (mIndex < bufSize &&
          msg[mIndex])
   {
      // if an intervening space, get rid of them
      if (' ' == msg[mIndex])
      {
         // if the character is a space, then skip it
         ++mIndex;
      }
      else
      {
         // if the character is not a space, copy it and
         // advance both indices
         msg[cIndex++] = msg[mIndex++];
      }
   }
   // re-guarantee the null termination
   msg[cIndex++] = '\0';
   return cIndex;
}


// DO NOT TRANSLATE ANY STRINGS IN THIS FUNCTION!
int display_error_message(int error, int retry)
{
#if 1
   (void) error;
   (void) retry;
   return 1;
#else
   char buf[32];

   switch (error)
   {
      case BUS_ERROR:
         return alert("Bus Error: OBDII bus is shorted to Vbatt or Ground.", NULL, NULL, (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case BUS_BUSY:
         return alert("OBD Bus Busy. Try again.", NULL, NULL, (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case BUS_INIT_ERROR:
         return alert("OBD Bus Init Error. Check connection to the vehicle,", "make sure the vehicle is OBD-II compliant,", "and ignition is ON.", (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case UNABLE_TO_CONNECT:
         return alert("Unable to connect to OBD bus.", "Check connection to the vehicle. Make sure", "the vehicle is OBD-II compliant, and ignition is ON.", (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case CAN_ERROR:
         return alert("CAN Error.", "Check connection to the vehicle. Make sure", "the vehicle is OBD-II compliant, and ignition is ON.", (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case DATA_ERROR:
      case DATA_ERROR2:
         return alert("Data Error: there has been a loss of data.", "You may have a bad connection to the vehicle,", "check the cable and try again.", (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case BUFFER_FULL:
         return alert("Hardware data buffer overflow.", NULL, NULL, (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      case SERIAL_ERROR:
      case UNKNOWN_CMD:
      case RUBBISH:
         return alert("Serial Link Error: please check connection", "between computer and scan tool.", NULL, (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);

      default:
         sprintf(buf, "Unknown error occured: %i", error);
         return alert(buf, NULL, NULL, (retry) ? "Retry" : "OK", (retry) ? "Cancel" : NULL, 0, 0);
   }
#endif
}


const char *get_protocol_string(int interface_type, int protocol_id)
{
   // response to an AT DP command
   switch (interface_type)
   {
      case INTERFACE_ELM320:
         return "SAE J1850 PWM (41.6 kBit/s)";
      case INTERFACE_ELM322:
         return "SAE J1850 VPW (10.4 kBit/s)";
      case INTERFACE_ELM323:
         return "ISO 9141-2 / ISO 14230-4 (KWP2000)";
      case INTERFACE_ELM327:
         switch (protocol_id)
         {
            case 0:
               return "N/A";
            case 1:
               return "SAE J1850 PWM (41.6 kBit/s)";
            case 2:
               return "SAE J1850 VPW (10.4 kBit/s)";
            case 3:
               return "ISO 9141-2";
            case 4:
               return "ISO 14230-4 KWP2000 (5-baud init)";
            case 5:
               return "ISO 14230-4 KWP2000 (fast init)";
            case 6:
               return "ISO 15765-4 CAN (11-bit ID, 500 kBit/s)";
            case 7:
               return "ISO 15765-4 CAN (29-bit ID, 500 kBit/s)";
            case 8:
               return "ISO 15765-4 CAN (11-bit ID, 250 kBit/s)";
            case 9:
               return "ISO 15765-4 CAN (29-bit ID, 250 kBit/s)";
         }
   }

   return "unknown";
}


int sendAndWaitForResponse(char *buf, size_t bufSize, char *cmdbuf, DWORD *numBytes, DWORD sleepTimeMs)
{
   int response;

   send_command(cmdbuf);
   Sleep(sleepTimeMs);
   // This also gives us the array of supported commands
   memset(buf, 0, bufSize);
   response = read_comport(buf, numBytes);
   *numBytes = compress_response(buf, *numBytes);
   return response;
}
