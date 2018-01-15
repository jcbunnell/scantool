#ifndef TOPWORK_H
#define TOPWORK_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_BANKS_OF_20 8       // number of banks to query for data
typedef enum _OBD_MODES
{
    MODE_CURRENT_DATA=1,
    MODE_FREEZE_FRAME_DATA=2,
    MODE_STORED_DIAG_TROUBLE_CODES=3,
    MODE_REQUEST_VIN=9
} OBD_MODES;

#ifdef LOG_COMMS
void write_comm_log(const char *marker, const char *data);
#endif

#ifdef WIN_GUI
extern HWND ghMainWnd;
#endif //WIN_GUI
extern int stopWork;

void getStoredDiagnosticCodes();
void process_all_codes(char *simBuffer);
void workInit(char *, unsigned long, int, char *, unsigned long , char *, unsigned long);
#ifdef __cplusplus
   }
#endif

#endif  /* TOPWORK_H */
