#include "globals.h"
#include "serial.h"
#include "trouble_code_reader.h"
#include "topwork.h"
#ifdef WIN_GUI
#include "resource.h"
#endif  /* WIN_GUI */

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

static int mil_is_on; // MIL is ON or OFF

#define CODE_LEN    5   /* Pxxxx or Uxxxx */
#define MAX_UNKNOWN_SIZE  10

TROUBLE_CODE unknownTCList[MAX_UNKNOWN_SIZE];

static void add_trouble_code(char *, int);
static void clear_trouble_codes(void);

// function definitions:
static void trouble_codes_simulator(int show);

extern COMPORT comport;

void initializeUnknownList(void)
{
    memset(unknownTCList, 0, sizeof(unknownTCList));
}

void destroyUnknownList(void)
{
    int k;
    for (k = 0; k < MAX_UNKNOWN_SIZE; ++k)
    {
        if (unknownTCList[k].code)
        {
            free(unknownTCList[k].code);
            unknownTCList[k].code = NULL;
            unknownTCList[k].foundCount = 0;
        }
    }
}

void ready_trouble_codes(void)
{
    clear_trouble_codes();
    mil_is_on = FALSE;
}

int parse_dtcs(const char *response, int pending)
{
    char code_letter[] = "PCBU";
    int dtc_count = 0;
    unsigned long k;
    char temp_trouble_code[CODE_LEN + 1];
    unsigned long respLen = (unsigned long)strlen(response);
    ULONG respConvert;

    for (k = 0; k < respLen; k += 4)    // read codes
    {
        memset(temp_trouble_code, 0, sizeof(temp_trouble_code));
        temp_trouble_code[0] = ' '; // make first position a blank space
                                    // begin to copy from response to temp_trouble_code.code beginning with position #1
        memcpy(temp_trouble_code + 1, response + k, CODE_LEN - 1);

        if (strcmp(temp_trouble_code, " 0000") == 0) // if there's no trouble code,
        {
            break;      // break out of the for() loop
        }

        // begin with position #1 (skip blank space), convert to hex, extract first two bits
        // use the result as an index into the code_letter array to get the corresponding code letter
        respConvert = strtol(temp_trouble_code + 1, NULL, 16);
        temp_trouble_code[0] = code_letter[respConvert >> 14];
        temp_trouble_code[1] = (char)((respConvert >> 12) & 0x03);
        temp_trouble_code[1] += 0x30; // convert to ASCII
        add_trouble_code(temp_trouble_code, pending);
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

    return (strlen(buf) > 0) ? TRUE : FALSE;
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
    int buf_len;
    int max_len;
    int trim;
    int i;

    // First, look for non-CAN and single-message CAN responses
    StringCchCopy(filter, sizeof(filter), (pending) ? "47" : "43");
    while (find_valid_response(msg, start, filter, &start))
    {
        if (strlen(msg) == 4)  // skip '4X 00' CAN responses
        {
            continue;
        }
        // if even number of bytes (CAN), skip first 2 bytes, otherwise, skip 1 byte
        i = (((strlen(msg) / 2) & 0x01) == 0) ? 4 : 2;
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
        int j = 0;
        start = vehicle_response;
#ifdef WIN_VS6
        sprintf(filter, "%X:", i);
#else // WIN_VS6
        StringCchPrintf(filter, sizeof(filter), "%X:", i);
#endif // WIN_VS6
        while (find_valid_response(msg, start, filter, &start))
        {
            for (; j < can_resp_cnt; j++)  // find next response that is not full
            {
                buf_len = (int)strlen(can_resp_buf[j]);
                max_len = (can_resp_len[j] - 2) * 2;
                if (buf_len < max_len)
                {
                    // first response -- skip '0:4XXX', all other -- skip 'X:'
                    // first response -- 6 bytes total, all other -- 7 bytes
                    // if this is last message for a response, trim padding
                    trim = (buf_len + (int)strlen(msg) - 2 >= max_len) ? buf_len + (int)strlen(msg) - 2 - max_len : 0;
#ifdef WIN_VS6
                    strcat(can_resp_buf[j], msg + ((i == 0) ? 6 : 2));
#else // WIN_VS6
                    StringCchCatN(can_resp_buf[j], max_len, msg + ((i == 0) ? 6 : 2), (i == 0) ? 8 : 14 - trim);
#endif // WIN_VS6
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

void add_trouble_code(char *init_code, int pending)
{
    if (init_code)
    {
        int k = 0;
        while (master_trouble_list[k].code)
        {
            if (0 == strcmp(init_code, master_trouble_list[k].code))
            {
                ++master_trouble_list[k].foundCount;
                if (pending)
                {
                    master_trouble_list[k].pending = "[Pending]";
                }
                break;
            }
            ++k;
        }
        // check if found
        if (master_trouble_list[k].code == NULL)
        {
            k = 0;
            while (k < MAX_UNKNOWN_SIZE &&
                   unknownTCList[k].code &&
                   strcmp(init_code, unknownTCList[k].code))
            {
                ++k;
            }
            if (k < MAX_UNKNOWN_SIZE)
            {
                if (unknownTCList[k].code)
                {
                    ++unknownTCList[k].foundCount;
                }
                else
                {
                    unknownTCList[k].code = (char *)malloc(CODE_LEN + 2);
                    if (unknownTCList[k].code)
                    {
                        memcpy(unknownTCList[k].code, init_code, CODE_LEN + 1);
                        unknownTCList[k].foundCount = 1;
                        if (pending)
                        {
                            unknownTCList[k].pending = "[Pending]";
                        }
                    }
                    else
                    {
                        printf("Error: Allocate for unknown trouble code failed\n");
                    }
                }
            }
            else
            {
                printf("Error: Unknown trouble list exceeded\n");
            }
        }
    }
}

void clear_trouble_codes(void)
{
    int k = 0;
    while (master_trouble_list[k].code)
    {
        master_trouble_list[k].foundCount = 0;
        ++k;
    }
}

void printTroubleCodes(char *buf, unsigned long bufSize)
{
    int numFound = 0;
    int k = 0;
    unsigned long nowLen;
    while (master_trouble_list[k].code)
    {
        if (master_trouble_list[k].foundCount)
        {
            numFound += master_trouble_list[k].foundCount;
            nowLen = (unsigned long)strlen(buf);
#ifdef WIN_VS6
            sprintf(buf + nowLen, "%s(%d) %s\n", master_trouble_list[k].code, master_trouble_list[k].foundCount, master_trouble_list[k].description);
#else // WIN_VS6
            StringCchPrintf(buf + nowLen, bufSize - nowLen, "%s(%d) %s\n", master_trouble_list[k].code, master_trouble_list[k].foundCount, master_trouble_list[k].description);
#endif // WIN_VS6
        }
        ++k;
    }
    k = 0;
    while (k < MAX_UNKNOWN_SIZE && unknownTCList[k].code)
    {
        ++numFound;
        nowLen = (unsigned long)strlen(buf);
#ifdef WIN_VS6
        sprintf(buf + nowLen, "%s Not Found\n", unknownTCList[k].code);
#else // WIN_VS6
        StringCchPrintf(buf + nowLen, bufSize - nowLen, "%s Not Found\n", unknownTCList[k].code);
#endif // WIN_VS6
        ++k;
    }
    if (numFound == 0)
    {
        nowLen = (unsigned long)strlen(buf);
#ifdef WIN_VS6
        sprintf(buf + nowLen, "None\n");
#else // WIN_VS6
        StringCchPrintf(buf + nowLen, bufSize - nowLen, "None\n");
#endif // WIN_VS6
    }
}
