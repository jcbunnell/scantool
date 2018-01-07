/*
 * ScanTool.net version 1.08 (Jan 12)
 *    + fixed problem with ECUs that pad the response with 0's (i.e., '41 05 7C 00 00 00')
 *    + removed references to "early Beta" from non-supported function descriptions (Apr 8)
 * ScanTool.net version 1.07, beta
 * started on January 2, 2003
 *    + added the rest of the sensors defined in SAE J1979 (APR2002)
 *    + cleaned up the code:
 *        = set_window_close_button is deprecated in the current version of Allegro (4.1.11 (WIP))
 *        = removed redundant formulas in sensors.c
 *    + added the rest of "OBD requirements" to sensors.c
 *    + added COM ports 5-8 to options.c
 *    + updated codes.dat with latest generic P and U codes, and removed B and C codes
 *    + system information dialog supports a wider range of platforms
 *
 *    TODO:
 *    - pending DTCs
 *    - freeze frames
 */

#ifdef WINDDK
#include <windows.h>
#include <strsafe.h>
#endif // WINDDK
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "serial.h"
#include "sensors.h"
#include "trouble_code_reader.h"
#include "topwork.h"

COMPORT comport;
#ifdef WIN_GUI
HWND ghMainWnd = NULL;
#endif // WIN_GUI
int stopWork=0;

#ifdef LOG_COMMS
char comm_log_file_name[20];
void write_comm_log(const char *marker, const char *data)
{
    FILE *logfile = NULL;

    if (0 == fopen_s(&logfile, comm_log_file_name, "a"))
    {
        fprintf(logfile, "[%s]%s[/%s]\n", marker, data, marker);
        fclose(logfile);
    }
}
#endif

static char BCDToByte(char hi, char lo)
{
    if (isdigit(hi))
    {
        hi -= '0';
    }
    else
    {
        hi = hi - 'A' + 10;
    }
    if (isdigit(lo))
    {
        lo -= '0';
    }
    else
    {
        lo = lo - 'A' + 10;
    }
    return (hi << 4) + lo;
}

static long getVinInfo(char *simBuffer, size_t simBufSize)
{
    int response;
    char cmdbuf[8];
    char inbuf[128];
    char vin[64];
    char *ptr;
    char *next;
    char *vptr;
    DWORD numBytes = 0;
    long modelYear = 0;

    if (NULL == simBuffer)
    {
        memset(inbuf, 0, sizeof(inbuf));
        // first and foremost, ask for the VIN information
        StringCchCopy(cmdbuf, sizeof(cmdbuf), "0902");
        response = sendAndWaitForResponse(inbuf, sizeof(inbuf), cmdbuf, &numBytes, CMD_TO_RESPONSE_VIN_SLEEP_MS);
        ptr = inbuf;
    }
    else
    {
        response = DATA;
        ptr = simBuffer;
        numBytes = (long) simBufSize;
    }
    // this gets a little tricky
    // the format for this command returns a length in units
    // then multiple lines with a line index prefix followed by a colon
    // 014
    // 0: 49 02 01 31 44 33
    // 1: 48 56 31 33 54 30 39
    // 2: 53 37 31 38 30 35 37
    memset(vin, 0, sizeof(vin));

    if (DATA == response &&
        numBytes)
    {
        // find the leading signature for the VIN return
        next = strstr(ptr, "0:490201");
        if (next)
        {
            numBytes -= (long)(next - ptr);
            ptr = next;
            numBytes -= 8;
            ptr += 8;
            vptr = vin;
            while (numBytes &&
                   *ptr &&
                   RECORD_DELIMITER != *ptr &&
                   LINE_DELIMITER != *ptr)
            {
                *vptr = BCDToByte(ptr[0], ptr[1]);
                ptr += 2;
                numBytes -= 2;
                ++vptr;
            }
            next = strstr(ptr, "1:");
            if (next)
            {
                numBytes -= (long)(next - ptr);
                ptr = next;
                ptr += 2;
                numBytes -= 2;
                while (numBytes &&
                       *ptr &&
                       RECORD_DELIMITER != *ptr &&
                       LINE_DELIMITER != *ptr)
                {
                    *vptr = BCDToByte(ptr[0], ptr[1]);
                    ptr += 2;
                    numBytes -= 2;
                    ++vptr;
                }
                next = strstr(ptr, "2:");
                if (next)
                {
                    numBytes -= (long)(next - ptr);
                    ptr = next;
                    numBytes -= 2;
                    ptr += 2;
                    while (numBytes &&
                           *ptr &&
                           RECORD_DELIMITER != *ptr &&
                           LINE_DELIMITER != *ptr)
                    {
                        *vptr = BCDToByte(ptr[0], ptr[1]);
                        ptr += 2;
                        numBytes -= 2;
                        ++vptr;
                    }
                }
            }
        }   // end IF the VIN id string is found
    }

    // assume the year is 1980-2009
    switch (vin[9])
    {
        case 'A':
        case 'B':
        case 'C':
        case 'D':
        case 'E':
        case 'F':
        case 'G':
        case 'H':
            modelYear = 1980 + vin[9] - 'A';
            break;
        case 'J':
        case 'K':
        case 'L':
        case 'M':
        case 'N':
            modelYear = 1988 + vin[9] - 'J';
            break;
        case 'P':
            modelYear = 1996;
            break;
        case 'R':
        case 'S':
        case 'T':
            modelYear = 1994 + vin[9] - 'R';
            break;
        case 'V':
        case 'W':
        case 'X':
        case 'Y':
            modelYear = 1997 + vin[9] - 'V';
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            modelYear = 2001 + vin[9] - '1';
            break;
        default:
            modelYear = 1980;
            break;
    }

    if (isalpha(vin[6]))
    {
        // make the adjustment to the new year range.
        modelYear += 2010 - 1980;
    }
    StringCchPrintf(inbuf, sizeof(inbuf), "%lu", modelYear);

#ifdef WIN_GUI
    SetDlgItemText(ghMainWnd, IDC_VEHICLEVINVALUE, vin);
    SetDlgItemText(ghMainWnd, IDC_MODELYEARVALUE, inbuf);
#else   /* WIN_GUI */
    printf("Vehicle VIN: %s  Model year: %s\n", vin, inbuf);
#endif  /* WIN_GUI */
    return numBytes;
}

void workInit(char *simBuffer, size_t simBufSize, int comPortNumber)
{
    initializeUnknownList();
    if (simBuffer)
    {
        comport.status = READY;
        simBufSize = compress_response(simBuffer, (long) simBufSize);
    }
    else
    {
#ifdef LOG_COMMS
        char temp_buf[256];
        StringCchCopy(comm_log_file_name, sizeof(comm_log_file_name), "comm_log.txt");
        remove(comm_log_file_name);
        write_comm_log("START_TIME", temp_buf);
#endif
        comport.status = NOT_OPEN;
        comport.number = comPortNumber;
        comport.baud_rate = 9600;

        /* try opening comport (comport.status will be set) */
        open_comport();
    }

    if (READY == comport.status)
    {
        simBufSize = getVinInfo(simBuffer, simBufSize);
    }
    destroyUnknownList();
}

void process_all_codes(char *simBuffer)
{
    int bank = 0;
    int response;
    char cmdbuf[16];
    char inbuf[128];
    char *ptr;
    unsigned long enabledCodes[3];
    unsigned long index;
    DWORD numBytes = 0;

    // the input buffer pointer being NULL means we are handling live data
    if (NULL == simBuffer)
    {
        if (READY == comport.status)
        {
            do
            {
                index = (bank * 0x20) + 1;    // set the index to the starting pid for that bank
                StringCchPrintf(cmdbuf, sizeof(cmdbuf), "%02X%X0", MODE_CURRENT_DATA, bank*2);
                // generate current mode commands
                response = sendAndWaitForResponse(inbuf, sizeof(inbuf), cmdbuf, &numBytes, CMD_TO_RESPONSE_SLEEP_MS);
                if (DATA == response)
                {
                    cmdbuf[0] = '4';  // replace command with response byte and find
                    ptr = strstr(inbuf, cmdbuf);
                    if (ptr)
                    {
                        unsigned long codes;
                        ptr += 4;   // skip the 4yxx response
                        codes = enabledCodes[bank] = strtoul(ptr, NULL, DATA_RADIX);
                        // continue until there are no more codes to process
                        while (codes && (0 == stopWork))
                        {
                            // check uppermost bits for what is enabled
                            if (codes & 0x80000000)
                            {
                                // query each of the interfaces supported
                                StringCchPrintf(cmdbuf, sizeof(cmdbuf), "%02X%02X", MODE_CURRENT_DATA, index);
                                response = sendAndWaitForResponse(inbuf, sizeof(inbuf), cmdbuf, &numBytes, CMD_TO_RESPONSE_SLEEP_MS);
                                cmdbuf[0] = '4';  // replace command with response byte and find
                                ptr = strstr(inbuf, cmdbuf);
                                if (ptr)
                                {
                                    // check to see if we are handling this code
                                    if (codeIsDisplayed(index))
                                    {
                                        process_and_display_data(ptr, simBuffer);
                                    }
                                    else
                                    {
                                        printf("PID %02X reported and not handled\n", (int) index);
                                    }
                                }
                                else
                                {
                                    printf("Hmmm. PID %02X reported as supported, but no response to query\n", (int) index);
                                }
                            }
                            ++index;        // account for numeric index
                            codes <<= 1;    // shift next bit up
                        }

                    }
                }
                else
                {
                    break;
                }
                ++bank;
            } while ((0 == stopWork) && (bank < MAX_BANKS_OF_20));
        }
    }
    else
    {
        // simulated data is being used.
        // that means we scan for return values and then pump them into the display routines
        int codeIndex;
        for (codeIndex=1;codeIndex < 0x7F; ++codeIndex)
        {
            /* do not process the indices reports */
            if (codeIndex != 0x20 && codeIndex != 0x40)
            {
                StringCchPrintf(cmdbuf, sizeof(cmdbuf), "41%02X", codeIndex);
                ptr = strstr(simBuffer, cmdbuf);
                if (ptr)
                {
                    process_and_display_data(ptr, simBuffer);
                }
            }
        }
    }
}

