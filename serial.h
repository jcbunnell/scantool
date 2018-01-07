#ifndef SERIAL_H
#define SERIAL_H

   #define COM1   0
   #define COM2   1
   #define COM3   2
   #define COM4   3
   #define COM5   4
   #define COM6   5
   #define COM7   6
   #define COM8   7
   #define BAUD_RATE_9600    BAUD_9600
   #define BAUD_RATE_38400   BAUD_38400

#define CMD_TO_RESPONSE_SLEEP_MS   100
#define CMD_TO_RESPONSE_VIN_SLEEP_MS   256
#define DATA_RADIX  16
#define FIELD_DELIMITER     '\t'
#define RECORD_DELIMITER    0x0D
#define LINE_DELIMITER      0x0A

//read_comport returned data type
#define EMPTY    0
#define DATA     1
#define PROMPT   2

#define SPECIAL_DELIMITER   '\t'

//comport status
#define READY          0
#define NOT_OPEN       1
#define USER_IGNORED   2

//process_response return values
#define HEX_DATA           0
#define BUS_BUSY           1
#define BUS_ERROR          2
#define BUS_INIT_ERROR     3
#define UNABLE_TO_CONNECT  4
#define CAN_ERROR          5
#define DATA_ERROR         6
#define DATA_ERROR2        7
#define ERR_NO_DATA        8
#define BUFFER_FULL        9
#define SERIAL_ERROR       10
#define UNKNOWN_CMD        11
#define RUBBISH            12

#define INTERFACE_ID       13
#define INTERFACE_ELM320   13
#define INTERFACE_ELM322   14
#define INTERFACE_ELM323   15
#define INTERFACE_ELM327   16

// timeouts
#define OBD_REQUEST_TIMEOUT   9900
#define ATZ_TIMEOUT           1500
#define AT_TIMEOUT            130
#define ECU_TIMEOUT           5000

#ifdef __cplusplus
extern "C" {
#endif

extern HANDLE CommHandle;

// function prototypes
DWORD compress_response(char *msg, DWORD bufSize);
int open_comport();
void close_comport();
void send_command(const char *command);
int read_comport(char *response, DWORD *numBytes);
void start_serial_timer(int delay);
int process_response(const char *cmd_sent, char *msg_received);
const char *get_protocol_string(int interface_type, int protocol_id);
int display_error_message(int error, int retry);
int sendAndWaitForResponse(char *buf, size_t bufSize, char *cmdbuf, DWORD *numBytes, DWORD sleepTimeMs);
#ifdef __cplusplus
   }
#endif

typedef enum 
{
    ST_READY
} ST_STATUS_TYPES;

typedef struct COMPORT {
   int number;
   int baud_rate;
   ST_STATUS_TYPES status;    // READY, NOT_OPEN, USER_IGNORED
} COMPORT;

#endif
