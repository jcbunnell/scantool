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
#endif // WINDDK
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "topwork.h"
#include "serial.h"
#include "sensors.h"
#include "trouble_code_reader.h"


int main(int argc, char *argv[])
{
    char vin[64];
    char modelYear[8];
    char *fname = NULL;
    int index = 1;
    int comPortNumber=7;
    char *simData = NULL;
    size_t simSize = 0;
    while (argc > index)
    {
        FILE *inFile = NULL;
        char *parm = argv[index];
        if ('-' == *parm)
        {
            ++parm;    // skip dash
            if ('i' == *parm)
            {
                ++parm;    // skip i
                // if the next character is a NULL, then the user put a space between the -i and the name
                // otherwise, what follows is a filename and fname already points to it.
                if (0 == *parm)
                {
                    ++index;
                    // if there is a next parameter, it is the file name
                    if (argc > index)
                    {
                        fname = argv[index];
                    }
                }
                else
                {
                    ++parm;
                    if ('=' == *parm)
                    {
                        ++parm;
                        fname = parm;
                    }
                }
            }
            else if ('c' == *parm)
            {
                ++parm;
                if (0 == *parm)
                {
                    ++index;
                    // if there is a next parameter, it is the file name
                    if (argc > index)
                    {
                        parm = argv[index];
                    }
                    else
                    {
                        // if not, oops
                        parm = NULL;
                    }
                }
                else
                {
                    if ('=' == *parm)
                    {
                        ++parm;
                    }
                }
                if (NULL == parm ||
                    1 != sscanf(parm, "%d", &comPortNumber))
                {
                    comPortNumber = 7;
                }
            }
        }

        if (fname)
        {
            // see if the file name is valid - read only
            inFile = fopen(fname, "r");
            if (inFile)
            {
                // go to end of file
                fseek(inFile, 0, SEEK_END);
                // find file size, plus one for NULL terminator
                simSize = ftell(inFile) + 1;
                // reset pointer to start of file
                fseek(inFile, 0, SEEK_SET);
                if (simSize > 1)
                {
                    // if there is a nonzero size, the allocate
                    simData = (char *)malloc(simSize);
                    if (simData)
                    {
                        // if allocation good, then read in the data
                        simSize = fread(simData, 1, simSize, inFile);
                        // and NULL terminate
                        simData[simSize] = 0;
                    }
                    else
                    {
                        // malloc failure, reset the size
                        simSize = 0;
                    }
                }
                else
                {
                    simSize = 0;
                }

                fclose(inFile);
            }
        }
        ++index;
    }

    printf("Starting with com port %d\n", comPortNumber);
    workInit(simData, simSize, comPortNumber, vin, sizeof(vin), modelYear, sizeof(modelYear)); // initialize everything
    printf("Vehicle VIN: %s  Model year: %s\n", vin, modelYear);

    process_all_codes(simData);

    getStoredDiagnosticCodes();

    if (NULL == simData)
    {
        close_comport();
    }

    return 0;
}

