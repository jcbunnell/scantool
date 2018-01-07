#include <windows.h>
#ifdef WIN_PRINTF
#include <strsafe.h>
#else // WIN_PRINTF
#endif // WIN_PRINTF
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "globals.h"
#include "serial.h"
#include "trouble_code_reader.h"
#include "scan_rc.h"
#ifndef WIN_PRINTF
#include "topwork.h"
#include "resource.h"
#endif  /* USER_PRINTF */

#define MAX(x,y)     (((x) > (y)) ? (x) : (y))

typedef enum
{
    MSG_USER,
    MSG_READ_CODES,
    MSG_CLEAR_CODES,
    MSG_READY,
    MSG_START,
    MSG_IDLE,
    MSG_END
} ST_MSGS;

#define D_O_K   ERROR_SUCCESS

#define CRQ_NONE       0
#define NUM_OF_CODES   1
#define READ_CODES     2
#define READ_PENDING   3
#define CLEAR_CODES    4

#define NUM_OF_RETRIES   3

typedef struct TROUBLE_CODE
{
    char code[7];
    char *description;
    char *pending;
    struct TROUBLE_CODE *next;
} TROUBLE_CODE;

static char mfr_code_description[] = "Manufacturer-specific code.  Please refer to your vehicle's service manual for more information";
static char mfr_pending_code_description[] = "[Pending]\nManufacturer-specific code.  Please refer to your vehicle's service manual for more information";
static char code_no_description[] = "";
static char pending_code_no_description[] = "[Pending]";

static int num_of_codes_reported = 0;
static int current_code_index;
static int mil_is_on; // MIL is ON or OFF

#define CODE_LEN    5   /* Pxxxx or Uxxxx */

static TROUBLE_CODE *pcode_list = NULL;
static TROUBLE_CODE *ucode_list = NULL;
static TROUBLE_CODE *trouble_codes = NULL;

static void add_trouble_code(const TROUBLE_CODE *);
static TROUBLE_CODE *get_trouble_code(int index);
static int get_number_of_codes();
static void clear_trouble_codes();

// function definitions:
static void trouble_codes_simulator(int show);
static void swap_codes(TROUBLE_CODE *, TROUBLE_CODE *);
static void handle_errors(int error, int operation);
static TROUBLE_CODE *find_code(TROUBLE_CODE *);

static char *pCodes=NULL;
static DWORD sizePCodes=0;
static char *uCodes=NULL;
static DWORD sizeUCodes=0;

extern COMPORT comport;

int loadResource(WORD enumId,
                 char **ppData,
                 DWORD *pSizeData)
{
    LPVOID  resMem; /*  Our memory handle to the locked resource. */
    HRSRC   resHandle; /*  The resource handle as found by lookup. */
    HGLOBAL resData; /*  The resource data handle as loaded. */
    HMODULE module = NULL;
    DWORD mSize;
    resHandle = FindResourceEx(module,
                               "BINARY",
                               MAKEINTRESOURCE(enumId),
                               MAKELANGID(LANG_NEUTRAL, SUBLANG_NEUTRAL));
    if (!resHandle)
    {
#ifdef WIN_PRINTF
        printf("Unable to find %d in %s resource (error=%d)", enumId,"BINARY", GetLastError());
#else   /* WIN_PRINTF */
#endif  /* WIN_PRINTF */
        return 0;
    }

    mSize = SizeofResource(module, resHandle);
    *pSizeData = mSize + 3;
    if (!*pSizeData)
    {
#ifdef WIN_PRINTF
        printf("%d resource has zero bytes.", enumId);
#else   /* WIN_PRINTF */
#endif  /* WIN_PRINTF */
        return 0;
    }

    resData = LoadResource(module, resHandle);
    if (!resData)
    {
#ifdef WIN_PRINTF
        printf("Unable to load %d resource", enumId);
#else   /* WIN_PRINTF */
#endif  /* WIN_PRINTF */
        return 0;
    }

    resMem = LockResource(resData);
    if (!resMem)
    {
#ifdef WIN_PRINTF
        printf("Unable to lock %d resource", enumId);
#else   /* WIN_PRINTF */
#endif  /* WIN_PRINTF */
        return 0;
    }

    if (*ppData)
    {
        free(*ppData);
    }
    *ppData = (char *)malloc(*pSizeData);
    if (*ppData)
    {
        char *ptr = mSize + *ppData;
        // copy the data over and make sure there is a terminating CRLF and NULL
        memcpy(*ppData, resMem, mSize);
        *ptr++ = RECORD_DELIMITER;
        *ptr++ = LINE_DELIMITER;
        *ptr = 0x00;
        *pSizeData -= 1;
    }
    return 1;
}

static void convertResourceToTC(char *pStart, DWORD sizeRes)
{
    char *ptr = pStart;
    DWORD sizeLeft = sizeRes;
    TROUBLE_CODE tCode;
    while ((CODE_LEN < sizeLeft) && *ptr)
    {
        memset(&tCode, 0, sizeof(TROUBLE_CODE));
        memcpy(tCode.code, ptr, CODE_LEN);
        ptr += CODE_LEN;
        sizeLeft -= CODE_LEN;

        // skip over the code elements
        while (*ptr != ' ' &&
               *ptr != FIELD_DELIMITER &&
               *ptr != RECORD_DELIMITER &&
               *ptr != LINE_DELIMITER)
        {
            ++ptr;
            --sizeLeft;
        }
        // skip over the intervening spaces
        while (*ptr == ' ' ||
               *ptr == FIELD_DELIMITER)
        {
            ++ptr;
            --sizeLeft;
        }
        tCode.description = ptr;
        // find the end of line
        while (*ptr != '\0' &&
               *ptr != RECORD_DELIMITER &&
               *ptr != LINE_DELIMITER)
        {
            ++ptr;
            --sizeLeft;
        }
        *ptr = 0;
        ++ptr;
        --sizeLeft;
        // skip the end of line
        while (*ptr != '\0' &&
               (*ptr == RECORD_DELIMITER ||
                *ptr == LINE_DELIMITER))
        {
            ++ptr;
            --sizeLeft;
        }
        add_trouble_code(&tCode);
    }
}

int import_trouble_codes()
{
    loadResource(IDR_BINARY_PCODES, &pCodes, &sizePCodes);
    loadResource(IDR_BINARY_UCODES, &uCodes, &sizeUCodes);

    // convert the list, using trouble_codes as the anchor
    convertResourceToTC(pCodes, sizePCodes);
    // at the end, move the pointer to our pointer
    pcode_list = trouble_codes;
    trouble_codes = NULL;

    // convert the list, using trouble_codes as the anchor
    convertResourceToTC(uCodes, sizeUCodes);
    // at the end, move the pointer to our pointer
    ucode_list = trouble_codes;
    trouble_codes = NULL;
    return 1;
}

void ready_trouble_codes()
{
    clear_trouble_codes();
    num_of_codes_reported = 0;
    mil_is_on = FALSE;
}


int parse_dtcs(const char *response, int pending)
{
    char code_letter[] = "PCBU";
    int dtc_count = 0;
    size_t k;
    TROUBLE_CODE temp_trouble_code;
    size_t respLen = strlen(response);
    ULONG respConvert;

    for (k = 0; k < respLen; k += 4)    // read codes
    {
        memset(&temp_trouble_code, 0, sizeof(TROUBLE_CODE));
        temp_trouble_code.code[0] = ' '; // make first position a blank space
        // begin to copy from response to temp_trouble_code.code beginning with position #1
        memcpy(temp_trouble_code.code + 1, response+k, CODE_LEN - 1);

        if (strcmp(temp_trouble_code.code, " 0000") == 0) // if there's no trouble code,
        {
            break;      // break out of the for() loop
        }

        // begin with position #1 (skip blank space), convert to hex, extract first two bits
        // use the result as an index into the code_letter array to get the corresponding code letter
        respConvert = strtol(temp_trouble_code.code + 1, NULL, 16);
        temp_trouble_code.code[0] = code_letter[respConvert >> 14];
        temp_trouble_code.code[1] = (char)((respConvert >> 12) & 0x03);
        temp_trouble_code.code[1] += 0x30; // convert to ASCII
        if (pending)
        {
            temp_trouble_code.pending = "[Pending]";
        }
        add_trouble_code(&temp_trouble_code);
        dtc_count++;
    }

    return dtc_count;
}


static int find_valid_response(char *buf, char *response, const char *filter, char **stop)
{
    char *in_ptr = response;
    char *out_ptr = buf;

    buf[0] = 0;

    while (*in_ptr)
    {
        // check if the current pointer matches the filter
        if (strncmp(in_ptr, filter, strlen(filter)) == 0)
        {
            // copy the matching filter and everything after until a delimiter is found
            while (*in_ptr &&
                   *in_ptr != SPECIAL_DELIMITER &&
                   *in_ptr != LINE_DELIMITER &&
                   *in_ptr != RECORD_DELIMITER) // copy valid response into buf
            {
                *out_ptr = *in_ptr;
                in_ptr++;
                out_ptr++;
            }
            *out_ptr = 0;  // terminate string
            while (*in_ptr == SPECIAL_DELIMITER ||
                   *in_ptr == LINE_DELIMITER ||
                   *in_ptr == RECORD_DELIMITER)
            {
                in_ptr++;
            }
            break;
        }
        else
        {
            // skip to the next delimiter
            while (*in_ptr &&
                   *in_ptr != SPECIAL_DELIMITER &&
                   *in_ptr != LINE_DELIMITER &&
                   *in_ptr != RECORD_DELIMITER)
            {
                in_ptr++;
            }
            if (*in_ptr == SPECIAL_DELIMITER ||
                *in_ptr == LINE_DELIMITER ||
                *in_ptr == RECORD_DELIMITER)  // skip the delimiter
            {
                in_ptr++;
            }
        }
    }

    if (stop)
    {
        *stop = in_ptr;
    }

    return(strlen(buf) > 0) ? TRUE : FALSE;
}


/* NOTE:
 *  ELM327 multi-message CAN responses are parsed using the following assumptions:
 *   - max 8 ECUs (per ISO15765-4 standard)
 *   - max 51 DTCs per ECU - there is no way to tell which ECU a response belongs to when the message counter wraps (0-F)
 *   - ECUs respond sequentially (i.e. 2nd msg from the 1st ECU to respond will come before 2nd msg from the 2nd ECU to respond)
 *     This has been observed imperically (in fact most of the time 2nd ECU lags several messages behind the 1st ECU),
 *     this should be true for most cases due to arbitration, unless 1st ECU takes a very long time to prepare next message,
 *     which should not happen. There is no other choice at the moment, unless we turn on headers, but that would mean rewriting
 *     whole communication paradigm.
 */

int handle_read_codes(char *vehicle_response, int pending)
{
    int dtc_count = 0;
    char *start = vehicle_response;
    char filter[3];
    char msg[48];
    int can_resp_cnt = 0;
    int can_msg_cnt = 0;
    int can_resp_len[8];
    char *can_resp_buf[8];  // 8 CAN ECUs max
    int buf_len, max_len, trim;
    int i, j;

    // First, look for non-CAN and single-message CAN responses
#ifdef WIN_PRINTF
    StringCchCopyA(filter, sizeof(filter), (pending) ? "47" : "43");
#else // WIN_PRINTF
#endif // WIN_PRINTF
    while (find_valid_response(msg, start, filter, &start))
    {
        if (strlen(msg) == 4)  // skip '4X 00' CAN responses
        {
            continue;
        }
        // if even number of bytes (CAN), skip first 2 bytes, otherwise, skip 1 byte
        i = (((strlen(msg)/2) & 0x01) == 0) ? 4 : 2;
        dtc_count += parse_dtcs(msg + i, pending);
    }

#if 0
    // Look for CAN multi-message responses
    start = vehicle_response;
    while (find_valid_response(msg, start, "", &start))  // step through all responses
    {
        if (strlen(msg) == 3)  // we're looking for 3-byte response length messages
        {
            can_resp_len[can_resp_cnt] = strtol(msg, NULL, 16);  // get total length for the response
            can_resp_buf[can_resp_cnt] = calloc((can_resp_len[can_resp_cnt]-2)*2 + 1, sizeof(char));
            i = (int) ceil((float)(can_resp_len[can_resp_cnt] + 1) / 7);  // calculate number of messages necessary to transmit specified number of bytes
            can_msg_cnt = MAX(can_msg_cnt, i);  // calculate max number of messages for any response
            can_resp_cnt++;
        }
    }
#endif

    for (i = 0; i < can_msg_cnt; i++)
    {
        j = 0;
        start = vehicle_response;
#ifdef WIN_PRINTF
        StringCchPrintf(filter, sizeof(filter), "%X:", i);
#else // WIN_PRINTF
#endif // WIN_PRINTF
        while (find_valid_response(msg, start, filter, &start))
        {
            for (; j < can_resp_cnt; j++)  // find next response that is not full
            {
                buf_len = (int) strlen(can_resp_buf[j]);
                max_len = (can_resp_len[j]-2)*2;
                if (buf_len < max_len)
                {
                    // first response -- skip '0:4XXX', all other -- skip 'X:'
                    // first response -- 6 bytes total, all other -- 7 bytes
                    // if this is last message for a response, trim padding
                    trim = (buf_len + (int)strlen(msg) - 2 >= max_len) ? buf_len + (int)strlen(msg) - 2 - max_len : 0;
#ifdef WIN_PRINTF
                    StringCchCatN(can_resp_buf[j], max_len, msg + ((i == 0) ? 6 : 2), (i == 0) ? 8 : 14 - trim);
#else // WIN_PRINTF
#endif // WIN_PRINTF
                    j++;
                    break;
                }
            }
        }
    }

    for (i = 0; i < can_resp_cnt; i++)
    {
        dtc_count += parse_dtcs(can_resp_buf[i], pending);
        free(can_resp_buf[i]);
    }

    return dtc_count; // return the actual number of codes read
}


void populate_trouble_codes_list(void)
{
    int i, j, min;
    TROUBLE_CODE *trouble_code;
    int codeCount = get_number_of_codes();
    TROUBLE_CODE *code_def;

    if (codeCount == 0)
    {
        return;
    }

    if (codeCount > 1)
    {
        for (i = 0; i < codeCount; i++)    // sort codes in ascending order
        {
            min = i;

            for (j = i+1; j < codeCount; j++)
                if (strcmp(get_trouble_code(j)->code, get_trouble_code(min)->code) < 0)
                    min = j;

            swap_codes(get_trouble_code(i), get_trouble_code(min));
        }
    }

    for (trouble_code = trouble_codes; trouble_code; trouble_code = trouble_code->next)   // search for descriptions and solutions
    {
        // pass the letter (B, C, P, or U) to find_code, which returns the file handle
        // if we reached EOF, or the file does not exist, go to the next DTC
        code_def = find_code(trouble_code);
        if (code_def)
        {
            trouble_code->description = code_def->description;
        }

    } // end of for() loop
}


void handle_errors(int error, int operation)
{
    static int retry_attempts = NUM_OF_RETRIES;

    if (error == BUS_ERROR || error == UNABLE_TO_CONNECT || error == BUS_INIT_ERROR)
    {
        display_error_message(error, FALSE);
        retry_attempts = NUM_OF_RETRIES;
        clear_trouble_codes();
        num_of_codes_reported = 0;
        mil_is_on = FALSE;
    }
    else    // if we received "BUS BUSY", "DATA ERROR", "<DATA ERROR", SERIAL_ERROR, or RUBBISH,
    {
        // try to re-send the request, do nothing if successful and alert user if failed:
        if (retry_attempts > 0) //
        {
            retry_attempts--;
            switch (operation)
            {
            case READ_CODES:
            case READ_PENDING:
            case NUM_OF_CODES:  // if we are currently reading codes,
                break;

            case CLEAR_CODES:   // if we are currently clearing codes,
                break;
            }
        }
        else
        {
            display_error_message(error, FALSE);
            retry_attempts = NUM_OF_RETRIES; // reset the number of retry attempts
            clear_trouble_codes();
            num_of_codes_reported = 0;
            mil_is_on = FALSE;
        }
    }
}


void swap_codes(TROUBLE_CODE *code1, TROUBLE_CODE *code2)
{
    char temp_str[256];
    char *temp_pend;

#ifdef WIN_PRINTF
    temp_pend = code1->pending;
    StringCchCopyA(temp_str, sizeof(temp_str), code1->code);
    code1->pending = code2->pending;
    StringCchCopyA(code1->code, sizeof(code1->code), code2->code);
    code2->pending = temp_pend;
    StringCchCopyA(code2->code, sizeof(code2->code), temp_str);
#else // WIN_PRINTF
    temp_pend = code1->pending;
    strcpy(temp_str, code1->code);
    code1->pending = code2->pending;
    strcpy(code1->code, code2->code);
    code2->pending = temp_pend;
    strcpy(code2->code, temp_str);
#endif // WIN_PRINTF
}


void add_trouble_code(const TROUBLE_CODE * init_code)
{
    TROUBLE_CODE *next = trouble_codes;

    if (init_code)
    {
        trouble_codes = (TROUBLE_CODE *)malloc(sizeof(TROUBLE_CODE));
        if (trouble_codes)
        {
#ifdef WIN_PRINTF
            StringCchCopyA(trouble_codes->code, sizeof(trouble_codes->code), init_code->code);
#else // WIN_PRINTF
#endif // WIN_PRINTF
            trouble_codes->description = init_code->description;
            trouble_codes->pending = init_code->pending;
            trouble_codes->next = next;
        }
        else
        {
            // if the malloc fails, just put the pointer back
            trouble_codes = next;
        }
    }
}


TROUBLE_CODE *get_trouble_code(int index)
{
    int i;
    TROUBLE_CODE *trouble_code = trouble_codes;

    for (i = 0; i < index; i++)
    {
        if (trouble_code->next == NULL)
            return NULL;
        trouble_code = trouble_code->next;
    }

    return trouble_code;
}


int get_number_of_codes()
{
    TROUBLE_CODE *trouble_code = trouble_codes;
    int ret = 0;

    while (trouble_code)
    {
        trouble_code = trouble_code->next;
        ret++;
    }

    return ret;
}


void clear_trouble_codes()
{
    TROUBLE_CODE *next;

    while (trouble_codes)
    {
        next = trouble_codes->next;

        free(trouble_codes);

        trouble_codes = next;
    }
}


TROUBLE_CODE *find_code(TROUBLE_CODE *inCode)
{
    TROUBLE_CODE *ret = NULL;

    if ('P' == inCode->code[0])
    {
        ret = pcode_list;
    }
    else if ('U' == inCode->code[0])
    {
        ret = ucode_list;
    }

    while (ret)
    {
        if (0 == strcmp(inCode->code, ret->code))
        {
            break;
        }
        ret = ret->next;
    }

    return ret;
}

void printTroubleCodes(char *buf, size_t bufSize)
{
    HRESULT hr = S_OK;
    if (trouble_codes)
    {
        TROUBLE_CODE *trouble_code = trouble_codes;

        while (S_OK == hr &&
               trouble_code)
        {
#ifdef WIN_PRINTF
            hr = StringCchCatA(buf, bufSize, trouble_code->code);
#else // WIN_PRINTF
#endif // WIN_PRINTF
            if (S_OK == hr)
            {
#ifdef WIN_PRINTF
                hr = StringCchCatA(buf, bufSize, " ");
#else // WIN_PRINTF
#endif // WIN_PRINTF
                if (S_OK == hr)
                {
                    if (trouble_code->description)
                    {
#ifdef WIN_PRINTF
                        hr = StringCchCatA(buf, bufSize, trouble_code->description);
#else // WIN_PRINTF
#endif // WIN_PRINTF
                    }
                    else
                    {
#ifdef WIN_PRINTF
                        hr = StringCchCatA(buf, bufSize, "Not Found");
#else // WIN_PRINTF
#endif // WIN_PRINTF
                    }
                    if (S_OK == hr)
                    {
#ifdef WIN_PRINTF
                        hr = StringCchCatA(buf, bufSize, "\n");
#else // WIN_PRINTF
#endif // WIN_PRINTF
                    }
                }
            }
            trouble_code = trouble_code->next;
        }
    }
    else
    {
#ifdef WIN_PRINTF
        StringCchCopyA(buf, bufSize, "None\n");
#else // WIN_PRINTF
#endif // WIN_PRINTF
    }
}
