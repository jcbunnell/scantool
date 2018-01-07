
#ifdef WINDDK
#include <windows.h>
#include <windowsx.h>
#include <intsafe.h>
#include <strsafe.h>
#endif // WINDDK
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
//#include <commctrl.h>
#include "globals.h"
#include "topwork.h"
#include "serial.h"
#include "sensors.h"
#include "trouble_code_reader.h"

static char *globalSimBuffer=NULL;

typedef struct
{
    char pid[PID_SIZE+1];
    int idc_value;
    int bIsListBox;
    int bIsProgressBar;
    int max_data_value;
    void (*formula)(int raw_data, char *buf, size_t bufSize);
    char label[32];
    char screen_buf[64];
    int enabled;
    int bytes; // number of data bytes expected from vehicle
} SENSOR;

// Sensor formulae:
static void throttle_position_formula(int data, char *buf, size_t bufSize); // Page 1
static void engine_rpm_formula(int data, char *buf, size_t bufSize);
static void vehicle_speed_formula(int data, char *buf, size_t bufSize);
static void engine_load_formula(int data, char *buf, size_t bufSize);
static void timing_advance_formula(int data, char *buf, size_t bufSize);
static void intake_pressure_formula(int data, char *buf, size_t bufSize);
static void air_flow_rate_formula(int data, char *buf, size_t bufSize);
static void fuel_system1_status_formula(int data, char *buf, size_t bufSize);
static void fuel_system2_status_formula(int data, char *buf, size_t bufSize);
static void short_term_fuel_trim_formula(int data, char *buf, size_t bufSize); // Page 2
static void long_term_fuel_trim_formula(int data, char *buf, size_t bufSize);
static void intake_air_temp_formula(int data, char *buf, size_t bufSize);
static void coolant_temp_formula(int data, char *buf, size_t bufSize);
static void fuel_pressure_formula(int data, char *buf, size_t bufSize);
static void secondary_air_status_formula(int data, char *buf, size_t bufSize);
static void pto_status_formula(int data, char *buf, size_t bufSize);
static void o2_sensor_formula(int data, char *buf, size_t bufSize);
void obd_requirements_formula(int data, char *buf, size_t bufSize);
// added 1/2/2003
static void engine_run_time_formula(int data, char *buf, size_t bufSize);
static void mil_distance_formula(int data, char *buf, size_t bufSize);
static void frp_relative_formula(int data, char *buf, size_t bufSize);
static void frp_widerange_formula(int data, char *buf, size_t bufSize);
static void o2_sensor_wrv_formula(int data, char *buf, size_t bufSize);
static void commanded_egr_formula(int data, char *buf, size_t bufSize);
static void egr_error_formula(int data, char *buf, size_t bufSize);
static void evap_pct_formula(int data, char *buf, size_t bufSize);
static void fuel_level_formula(int data, char *buf, size_t bufSize);
static void warm_ups_formula(int data, char *buf, size_t bufSize);
static void clr_distance_formula(int data, char *buf, size_t bufSize);
static void evap_vp_formula(int data, char *buf, size_t bufSize);
static void baro_pressure_formula(int data, char *buf, size_t bufSize);
static void o2_sensor_wrc_formula(int data, char *buf, size_t bufSize);
static void cat_temp_formula(int data, char *buf, size_t bufSize);
static void ecu_voltage_formula(int data, char *buf, size_t bufSize);
static void abs_load_formula(int data, char *buf, size_t bufSize);
static void eq_ratio_formula(int data, char *buf, size_t bufSize);
static void relative_tp_formula(int data, char *buf, size_t bufSize);
static void amb_air_temp_formula(int data, char *buf, size_t bufSize);
static void abs_tp_formula(int data, char *buf, size_t bufSize);
static void tac_pct_formula(int data, char *buf, size_t bufSize);
static void mil_time_formula(int data, char *buf, size_t bufSize);
static void clr_time_formula(int data, char *buf, size_t bufSize);
static void process_trouble_codes(int data, char *buf, size_t bufSize);

static SENSOR sensors[] =
{
    //pid    IDC_x                             List Bar  max formula                       label                              screen_buffer     enabled   bytes
    { "01",  IDC_TROUBLECODESLIST,             1,   0,  0,  process_trouble_codes,         "Trouble codes:",                  "",               1,        4},
    { "03",  IDC_FUELSYSTEM1STATUSVALUE,       0,   0,  0,  fuel_system1_status_formula,   "Fuel System 1 Status:",           "",               1,        2},
    { "03",  0,                                0,   0,  0,  fuel_system2_status_formula,   "Fuel System 2 Status:",           "",               1,        2},
    { "04",  IDC_CALCULATEDLOADVALUE,          0,   0,  0,  engine_load_formula,           "Calculated Load Value:",          "",               1,        1},
    { "05",  IDC_COOLANTTEMPVALUE,             0,   0,  0,  coolant_temp_formula,          "Coolant Temperature:",            "",               1,        1},
    { "06",  0,                                0,   0,  0,  short_term_fuel_trim_formula,  "Short Term Fuel Trim (Bank 1):",  "",               1,        2},
    { "07",  0,                                0,   0,  0,  long_term_fuel_trim_formula,   "Long Term Fuel Trim (Bank 1):",   "",               1,        2},
    { "08",  0,                                0,   0,  0,  short_term_fuel_trim_formula,  "Short Term Fuel Trim (Bank 2):",  "",               1,        2},
    { "09",  0,                                0,   0,  0,  long_term_fuel_trim_formula,   "Long Term Fuel Trim (Bank 2):",   "",               1,        2},
    { "0A",  0,                                0,   0,  0,  fuel_pressure_formula,         "Fuel Pressure (gauge):",          "",               1,        1},
    { "0B",  IDC_INTAKEMANIFOLDPRESSUREVALUE,  0,   0,  0,  intake_pressure_formula,       "Intake Manifold Pressure:",       "",               1,        1},
    { "0C",  IDC_RPMNUMVALUE,                  0,   0,  0,  engine_rpm_formula,            "Engine RPM:",                     "",               1,        2},
    { "0D",  IDC_SPEEDNUMVALUE,                0,   0,  0,  vehicle_speed_formula,         "Vehicle Speed:",                  "",               1,        1},
    { "0E",  0,                                0,   0,  0,  timing_advance_formula,        "Timing Advance (Cyl. #1):",       "",               1,        1},
    { "0F",  IDC_INTAKEAIRTEMPVALUE,           0,   0,  0,  intake_air_temp_formula,       "Intake Air Temperature:",         "",               1,        1},
    { "10",  0,                                0,   0,  0,  air_flow_rate_formula,         "Air Flow Rate (MAF sensor):",     "",               1,        2},
    { "12",  0,                                0,   0,  0,  secondary_air_status_formula,  "Secondary air status:",           "",               1,        1},
    { "14",  IDC_O2SENSOR1BANK1VALUE,          0,   0,  0,  o2_sensor_formula,             "O2 Sensor 1, Bank 1:",            "",               1,        2},
    { "15",  IDC_O2SENSOR2BANK1VALUE,          0,   0,  0,  o2_sensor_formula,             "O2 Sensor 2, Bank 1:",            "",               1,        2},
    { "16",  0,                                0,   0,  0,  o2_sensor_formula,             "O2 Sensor 3, Bank 1:",            "",               1,        2},
    { "17",  0,                                0,   0,  0,  o2_sensor_formula,             "O2 Sensor 4, Bank 1:",            "",               1,        2},
    { "18",  IDC_O2SENSOR1BANK2VALUE,          0,   0,  0,  o2_sensor_formula,             "O2 Sensor 1, Bank 2:",            "",               1,        2},
    { "19",  IDC_O2SENSOR2BANK2VALUE,          0,   0,  0,  o2_sensor_formula,             "O2 Sensor 2, Bank 2:",            "",               1,        2},
    { "1A",  0,                                0,   0,  0,  o2_sensor_formula,             "O2 Sensor 3, Bank 2:",            "",               1,        2},
    { "1B",  0,                                0,   0,  0,  o2_sensor_formula,             "O2 Sensor 4, Bank 2:",            "",               1,        2},
    { "1C",  IDC_OBDCONFORMSTOVALUE,           0,   0,  0,  obd_requirements_formula,      "OBD conforms to:",                "",               1,        1},
    { "1E",  0,                                0,   0,  0,  pto_status_formula,            "Power Take-Off Status:",          "",               1,        1},
    { "1F",  IDC_TIMESINCEENGINESTARTVALUE,    0,   0,  0,  engine_run_time_formula,       "Time Since Engine Start:",        "",               1,        2},
    { "21",  IDC_DISTANCESINCEMILVALUE,        0,   0,  0,  mil_distance_formula,          "Distance since MIL activated:",   "",               1,        2},
    { "22",  0,                                0,   0,  0,  frp_relative_formula,          "FRP rel. to manifold vacuum:",    "",               1,        2},    // fuel rail pressure relative to manifold vacuum
    { "23",  0,                                0,   0,  0,  frp_widerange_formula,         "Fuel Pressure (gauge):",          "",               1,        2},    // fuel rail pressure (gauge), wide range
    { "24",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 1, Bank 1 (WR):",       "",               1,        4},    // o2 sensors (wide range), voltage
    { "25",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 2, Bank 1 (WR):",       "",               1,        4},
    { "26",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 3, Bank 1 (WR):",       "",               1,        4},
    { "27",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 4, Bank 1 (WR):",       "",               1,        4},
    { "28",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 1, Bank 2 (WR):",       "",               1,        4},
    { "29",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 2, Bank 2 (WR):",       "",               1,        4},
    { "2A",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 3, Bank 2 (WR):",       "",               1,        4},
    { "2B",  0,                                0,   0,  0,  o2_sensor_wrv_formula,         "O2 Sensor 4, Bank 2 (WR):",       "",               1,        4},
    { "2C",  0,                                0,   0,  0,  commanded_egr_formula,         "Commanded EGR:",                  "",               1,        1},
    { "2D",  0,                                0,   0,  0,  egr_error_formula,             "EGR Error:",                      "",               1,        1},
    { "2E",  0,                                0,   0,  0,  evap_pct_formula,              "Commanded Evaporative Purge:",    "",               1,        1},
    { "2F",  IDC_FUELNUMVALUE,                 0,   0,  0,  fuel_level_formula,            "Fuel Level Input:",               "",               1,        1},
    { "30",  IDC_WARMUPSSINCECURESETLABEL,     0,   0,  0,  warm_ups_formula,              "Warm-ups since ECU reset:",       "",               1,        1},
    { "31",  IDC_DISTANCEECURESETVALUE,        0,   0,  0,  clr_distance_formula,          "Distance since ECU reset:",       "",               1,        2},
    { "32",  0,                                0,   0,  0,  evap_vp_formula,               "Evap System Vapor Pressure:",     "",               1,        2},
    { "33",  IDC_BAROMETRICPRESSUREVALUE,      0,   0,  0,  baro_pressure_formula,         "Barometric Pressure (absolute):", "",               1,        1},
    { "34",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 1, Bank 1 (WR):",       "",               1,        4},   // o2 sensors (wide range), current
    { "35",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 2, Bank 1 (WR):",       "",               1,        4},
    { "36",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 3, Bank 1 (WR):",       "",               1,        4},
    { "37",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 4, Bank 1 (WR):",       "",               1,        4},
    { "38",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 1, Bank 2 (WR):",       "",               1,        4},
    { "39",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 2, Bank 2 (WR):",       "",               1,        4},
    { "3A",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 3, Bank 2 (WR):",       "",               1,        4},
    { "3B",  0,                                0,   0,  0,  o2_sensor_wrc_formula,         "O2 Sensor 4, Bank 2 (WR):",       "",               1,        4},
    { "3C",  0,                                0,   0,  0,  cat_temp_formula,              "CAT Temperature, B1S1:",          "",               1,        2},
    { "3D",  0,                                0,   0,  0,  cat_temp_formula,              "CAT Temperature, B2S1:",          "",               1,        2},
    { "3E",  0,                                0,   0,  0,  cat_temp_formula,              "CAT Temperature, B1S2:",          "",               1,        2},
    { "3F",  0,                                0,   0,  0,  cat_temp_formula,              "CAT Temperature, B2S2:",          "",               1,        2},
    { "42",  IDC_ECUVOLTAGEVALUE,              0,   0,  0,  ecu_voltage_formula,           "ECU voltage:",                    "",               1,        2},
    { "43",  IDC_ABSENGINELOADVALUE,           0,   0,  0,  abs_load_formula,              "Absolute Engine Load:",           "",               1,        2},
    { "44",  0,                                0,   0,  0,  eq_ratio_formula,              "Commanded Equivalence Ratio:",    "",               1,        2},
    { "45",  IDC_THROTTLENUMVALUE,             0,   0,  0,  relative_tp_formula,           "Relative Throttle Position:",     "",               1,        1},
    { "46",  IDC_AMBIENTAIRTEMPVALUE,          0,   0,  0,  amb_air_temp_formula,          "Ambient Air Temperature:",        "",               1,        1},  // same scaling as $0F
    { "47",  0,                                0,   0,  0,  abs_tp_formula,                "Absolute Throttle Position B:",   "",               1,        1},
    { "48",  0,                                0,   0,  0,  abs_tp_formula,                "Absolute Throttle Position C:",   "",               1,        1},
    { "49",  0,                                0,   0,  0,  abs_tp_formula,                "Accelerator Pedal Position D:",   "",               1,        1},
    { "4A",  0,                                0,   0,  0,  abs_tp_formula,                "Accelerator Pedal Position E:",   "",               1,        1},
    { "4B",  0,                                0,   0,  0,  abs_tp_formula,                "Accelerator Pedal Position F:",   "",               1,        1},
    { "4C",  0,                                0,   0,  0,  tac_pct_formula,               "Comm. Throttle Actuator Cntrl:",  "",               1,        1}, // commanded TAC
    { "4D",  IDC_ENGINERUNMILVALUE,            0,   0,  0,  mil_time_formula,              "Engine running while MIL on:",    "",               1,        2}, // minutes run by the engine while MIL activated
    { "4E",  IDC_TIMESINCEDTCLEAREDVALUE,      0,   0,  0,  clr_time_formula,              "Time since DTCs cleared:",        "",               1,        2},
#ifndef WIN_GUI
    { "0C",  IDC_ENGINERPMVALUE,               0,   1,  16383,  engine_rpm_formula,            "Engine RPM:",                     "",               1,        2},
    { "0D",  IDC_VEHICLESPEEDVALUE,            0,   1,  255,  vehicle_speed_formula,         "Vehicle Speed:",                  "",               1,        1},
    { "2F",  IDC_FUELEVELVALUE,                0,   1,  100,  fuel_level_formula,            "Fuel Level Input:",               "",               1,        1},
    { "45",  IDC_RELTHROTTLEVALUE,             0,   1,  100,  relative_tp_formula,           "Relative Throttle Position:",     "",               1,        1},
#endif  // WIN_GUI
    { "",    0,                                0,   0,  0,     NULL,                          "",                                "",               0,        0}
};

// Options
static int system_of_measurements = IMPERIAL;

int codeIsDisplayed(unsigned long index)
{
    int rc = 0;
    char hexValue[16];
    StringCchPrintf(hexValue, sizeof(hexValue), "%02X", (int) index);

    index = 0;
    // until we get a stop work order or run out of pids in the list
    while ((0 == stopWork) && sensors[index].pid[0])
    {
        // check to see if the pids match
        if (0 == strncmp(hexValue, sensors[index].pid, PID_SIZE))
        {
            if (sensors[index].idc_value && sensors[index].formula)
            {
                rc = 1;
                break;
            }
        }
        ++index;
    }
    return rc;
}


void process_and_display_data(char *buf, char *simBuffer)
{
    // if the buffer holds a response
    if (0 == strncmp(buf, "41", RESPONSE_SIZE))
    {
        int index=0;
        buf += RESPONSE_SIZE;
        while ((0 == stopWork) && sensors[index].pid[0])
        {
            // if there is a formula
            // and it matches the pid, then we have a winner
            if (sensors[index].formula &&
                (0 == strncmp(buf, sensors[index].pid, PID_SIZE)))
            {
                char *valuePtr = buf + PID_SIZE;
                if ((int)strlen(valuePtr) >= sensors[index].bytes)
                {
                    char outbuf[OUTPUT_BUFFER_SIZE];
                    int data = (int) strtoul(valuePtr, NULL, DATA_RADIX);

                    // for the purpose of the simulator, set the global
                    // pointer to the input buffer value to pick up where they left off
                    if (simBuffer)
                    {
                        globalSimBuffer = valuePtr;
                    }

                    // process the data into buffer.
                    // all routines null terminate the buffer
                    sensors[index].formula(data, outbuf, sizeof(outbuf));

                    // for clarity the next time through, reset to NULL
                    if (simBuffer)
                    {
                        globalSimBuffer = NULL;
                    }
#ifdef WIN_GUI
                    if (sensors[index].bIsProgressBar)
                    {
                        float relData;
                        int newData = 0;
                        int numItems = sscanf(outbuf, "%f", &relData);
                        if (1 == numItems &&
                            sensors[index].max_data_value &&
                            ((int)relData <= sensors[index].max_data_value))
                        {
                            newData = (int) (relData * (float) 100.0 / (float)sensors[index].max_data_value);
                        }
                        SendDlgItemMessage(ghMainWnd, sensors[index].idc_value, PBM_SETPOS, newData, 0);
                    }
                    else
                    {
                        if (sensors[index].bIsListBox)
                        {
                            HWND listBox = GetDlgItem(ghMainWnd, sensors[index].idc_value);
                            if (listBox)
                            {
                                char *lineToPrint = outbuf;
                                char *newLine;
                                // reset the current contents
                                (void)ListBox_ResetContent(listBox);
                                // while there are complete lines remaining in the output
                                while (lineToPrint)
                                {
                                    // find the end of current line
                                    newLine = strstr(lineToPrint, "\n");
                                    if (newLine)
                                    {
                                        // if found, null terminate
                                        *newLine = '\0';
                                        // add it to the list
                                        (void)ListBox_AddString(listBox, lineToPrint);
                                        // advance past the CRLF
                                        lineToPrint = newLine + 1;
                                        while (RECORD_DELIMITER == *lineToPrint ||
                                               LINE_DELIMITER == *lineToPrint)
                                        {
                                            ++lineToPrint;
                                        }
                                    }
                                    else
                                    {
                                        lineToPrint = NULL;
                                    }
                                }
                            }
                        }
                        else
                        {
                            SetDlgItemText(ghMainWnd, sensors[index].idc_value, outbuf);
                        }
                    }
#else   /* WIN_GUI */
                    printf("%s %s\n", sensors[index].label, outbuf);
#endif  /* WIN_GUI */
                }
            }
            // continue the search in case a field has two displays
            ++index;
        }
    }
}

// the data value of the trouble code read
// buf and bufSize are for the output buffer
void process_trouble_codes(int data, char *buf, size_t bufSize)
{
    // the first byte is the trouble code count and the high bit is the MIL status light indicator
    // the remaining 3 bytes are tests available and tests incomplete
    // A7 is MIL
    // A6-0 is the trouble code count
    //
    //                      Test available      Test incomplete
    // Misfire                B0                   B4
    // Fuel System            B1                   B5
    // Components             B2                   B6
    // Reserved               B3                   B7
    // Catalyst               C0                   D0
    // Heated Catalyst        C1                   D1
    // Evaporative System     C2                   D2
    // Secondary Air System   C3                   D3
    // A/C Refrigerant        C4                   D4
    // Oxygen Sensor          C5                   D5
    // Oxygen Sensor Heater   C6                   D6
    // EGR System             C7                   D7
    int codeCount;
    int lightOn = 0;
    int k;
    int response;
    char *globalPtr = globalSimBuffer;
    DWORD numBytes = 0;

    ready_trouble_codes();

    if (data & 0x80000000)
    {
        lightOn = 1;
        codeCount &= 0x7FFFFFFF;
    }

    codeCount = (data >> 24) & 0x7F;
    StringCchPrintf(buf, bufSize, "%d : MIL=%s\n",
                    codeCount, lightOn ? "On" : "Off");

    for (k=0; k<codeCount && (0 == stopWork);)
    {
        if (NULL == globalSimBuffer)
        {
            char inbuf[128];
            char cmdbuf[16];
            StringCchPrintf(cmdbuf, sizeof(cmdbuf), "%02X", MODE_STORED_DIAG_TROUBLE_CODES);
            response = sendAndWaitForResponse(inbuf, sizeof(inbuf), cmdbuf, &numBytes, CMD_TO_RESPONSE_SLEEP_MS);
            if (DATA == response)
            {
                k += handle_read_codes(inbuf, FALSE);
            }
            else
            {
                break;
            }
        }
        else
        {
            // find the next response string in the file
            globalPtr = strstr(globalPtr, "43");
            k += handle_read_codes(globalPtr, FALSE);
            ++globalPtr;    // go to next byte so that we don't spin on the same line
        }
    }

    printTroubleCodes(buf, bufSize);
}

void getStoredDiagnosticCodes()
{
    DWORD numBytes = 0;
    int response;
    char inbuf[1024];
    char cmdbuf[16];
    StringCchPrintf(cmdbuf, sizeof(cmdbuf), "%02X", MODE_STORED_DIAG_TROUBLE_CODES);
    response = sendAndWaitForResponse(inbuf, sizeof(inbuf), cmdbuf, &numBytes, CMD_TO_RESPONSE_SLEEP_MS);
    if (DATA == response)
    {
        handle_read_codes(inbuf, FALSE);
    }
}

void engine_rpm_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i r/min", data/4);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%i rpm", data/4);
    }
}


void engine_load_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100/255);
}


void coolant_temp_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i%c C", data-40, 0xB0);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%i%c F", (int)(((float)data-40.0)*9.0/5.0 + 32.0), 0xB0);
    }
}


void fuel_system_status_formula(int data, char *buf, size_t bufSize)
{
    if (data == 0)
    {
        StringCchCopy(buf, bufSize, "unused");
    }
    else if (data == 0x01)
    {
        StringCchCopy(buf, bufSize, "open loop");
    }
    else if (data == 0x02)
    {
        StringCchCopy(buf, bufSize, "closed loop");
    }
    else if (data == 0x04)
    {
        StringCchCopy(buf, bufSize, "open loop (driving conditions)");
    }
    else if (data == 0x08)
    {
        StringCchCopy(buf, bufSize, "open loop (system fault)");
    }
    else if (data == 0x10)
    {
        StringCchCopy(buf, bufSize, "closed loop, O2 sensor fault");
    }
    else
    {
        StringCchPrintf(buf, bufSize, "unknown: 0x%02X", data);
    }
}


void fuel_system1_status_formula(int data, char *buf, size_t bufSize)
{
    fuel_system_status_formula((data >> 8) & 0xFF, buf, bufSize);  // Fuel System 1 status: Data A
}


void fuel_system2_status_formula(int data, char *buf, size_t bufSize)
{
    fuel_system_status_formula(data & 0xFF, buf, bufSize);  // Fuel System 2 status: Data B
}


void vehicle_speed_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i km/h", data);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%i mph", (int)((float)data/1.609));
    }
}


void intake_pressure_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i kPa", data);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.1f inHg", (float)data/3.386389);
    }
}


void timing_advance_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%c", ((float)data-128.0)/2.0, 0xB0);
}


void intake_air_temp_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i%c C", data-40, 0xB0);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%i%c F", (int)(((float)data-40.0)*9.0/5.0 + 32.0), 0xB0);
    }
}


void air_flow_rate_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%.2f g/s", data*0.01);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.1f lb/min", data*0.0013227736);
    }
}


void throttle_position_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100.0/255.0);
}


// **** New formulae added 3/11/2003: ****

// Fuel Pressure (guage): PID 0A
void fuel_pressure_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i kPa", data*3);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%.1f psi", (float)data*3.0*0.145037738);
    }
}


// Fuel Trim statuses: PID 06-09
void short_term_fuel_trim_formula(int data, char *buf, size_t bufSize)
{
    if (data > 0xFF)  // we're only showing bank 1 and 2 FT
    {
        data >>= 8;
    }

    StringCchPrintf(buf, bufSize, (data == 128) ? "0.0%%" : "%+.1f%%", ((float)data - 128.0)*100.0/128.0);
}


void long_term_fuel_trim_formula(int data, char *buf, size_t bufSize)
{
    if (data > 0xFF)  // we're only showing bank 1 and 2 FT
    {
        data >>= 8;
    }

    StringCchPrintf(buf, bufSize, (data == 128) ? "0.0%%" : "%+.1f%%", ((float)data - 128.0)*100.0/128.0);
}


// Commanded secondary air status: PID 12
void secondary_air_status_formula(int data, char *buf, size_t bufSize)
{
    data = data & 0x0700; // mask bits 0, 1, and 2

    if (data == 0x0100)
    {
        StringCchCopy(buf, bufSize, "upstream of 1st cat. conv.");
    }
    else if (data == 0x0200)
    {
        StringCchCopy(buf, bufSize, "downstream of 1st cat. conv.");
    }
    else if (data == 0x0400)
    {
        StringCchCopy(buf, bufSize, "atmosphere / off");
    }
    else
    {
        StringCchCopy(buf, bufSize, "Not supported");
    }
}

// Oxygen sensor voltages & short term fuel trims: PID 14-1B
// Format is bankX_sensor

void o2_sensor_formula(int data, char *buf, size_t bufSize)
{
    if ((data & 0xFF) == 0xFF)  // if the sensor is not used in fuel trim calculation,
    {
        StringCchPrintf(buf, bufSize, "%.3f V", (data >> 8)*0.005);
    }
    else
    {
        StringCchPrintf(buf, bufSize, ((data & 0xFF) == 128) ? "%.3f V @ 0.0%% s.t. fuel trim" : "%.3f V @ %+.1f%% s.t. fuel trim", (data >> 8)*0.005, ((float)(data & 0xFF) - 128.0)*100.0/128.0);
    }
}


//Power Take-Off Status: PID 1E
void pto_status_formula(int data, char *buf, size_t bufSize)
{
    if ((data & 0x01) == 0x01)
    {
        StringCchCopy(buf, bufSize, "active");
    }
    else
    {
        StringCchCopy(buf, bufSize, "not active");
    }
}

// OBD requirement to which vehicle is designed: PID 1C
void obd_requirements_formula(int data, char *buf, size_t bufSize)
{
    switch (data)
    {
    case 0x01:
        StringCchCopy(buf, bufSize, "OBD-II (California ARB)");
        break;
    case 0x02:
        StringCchCopy(buf, bufSize, "OBD (Federal EPA)");
        break;
    case 0x03:
        StringCchCopy(buf, bufSize, "OBD and OBD-II");
        break;
    case 0x04:
        StringCchCopy(buf, bufSize, "OBD-I");
        break;
    case 0x05:
        StringCchCopy(buf, bufSize, "Not OBD compliant");
        break;
    case 0x06:
        StringCchCopy(buf, bufSize, "EOBD (Europe)");
        break;
    case 0x07:
        StringCchCopy(buf, bufSize, "EOBD and OBD-II");
        break;
    case 0x08:
        StringCchCopy(buf, bufSize, "EOBD and OBD");
        break;
    case 0x09:
        StringCchCopy(buf, bufSize, "EOBD, OBD and OBD-II");
        break;
    case 0x0A:
        StringCchCopy(buf, bufSize, "JOBD (Japan)");
        break;
    case 0x0B:
        StringCchCopy(buf, bufSize, "JOBD and OBD-II");
        break;
    case 0x0C:
        StringCchCopy(buf, bufSize, "JOBD and EOBD");
        break;
    case 0x0D:
        StringCchCopy(buf, bufSize, "JOBD, EOBD, and OBD-II");
        break;
    default:
        StringCchPrintf(buf, bufSize, "Unknown: 0x%02X", data);
    }
}

/* Sensors added 1/2/2003: */

void engine_run_time_formula(int data, char *buf, size_t bufSize)
{
    int sec, min, hrs;

    hrs = data / 3600;  // get hours
    min = (data % 3600) / 60;  // get minutes
    sec = data % 60;  // get seconds

    StringCchPrintf(buf, bufSize, "%02i:%02i:%02i", hrs, min, sec);
}


void mil_distance_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i km", data);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%i miles", (int)((float)data/1.609));
    }
}


void frp_relative_formula(int data, char *buf, size_t bufSize)
{
    float kpa, psi;

    kpa = (float)data*(float)0.079;
    psi = kpa*(float)0.145037738;

    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%.3f kPa", kpa);
    }
    else   // if the system is IMPERIAL
    {
        StringCchPrintf(buf, bufSize, "%.1f PSI", psi);
    }
}


void frp_widerange_formula(int data, char *buf, size_t bufSize)
{
    int kpa;
    float psi;

    kpa = data*10;
    psi = (float)kpa*(float)0.145037738;

    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i kPa", kpa);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.1f PSI", psi);
    }
}


void o2_sensor_wrv_formula(int data, char *buf, size_t bufSize)
{
    float eq_ratio, o2_voltage; // equivalence ratio and sensor voltage

    eq_ratio = (float)(data >> 16)*(float)0.0000305;  // data bytes A,B
    o2_voltage = (float)(data & 0xFFFF)*(float)0.000122; // data bytes C,D

    StringCchPrintf(buf, bufSize, "%.3f V, Eq. ratio: %.3f", o2_voltage, eq_ratio);
}


//Commanded EGR status: PID 2C
void commanded_egr_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100.0/255.0);
}

//EGR error: PID 2D
void egr_error_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, (data == 128) ? "0.0%%" : "%+.1f%%", (float)(data-128)/255.0*100.0);
}


void evap_pct_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data/255.0*100.0);
}


void fuel_level_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data/255.0*100.0);
}


void warm_ups_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%i", data);
}


void clr_distance_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i km", data);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%i miles", (int)((float)data/1.609));
    }
}


void evap_vp_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%.2f Pa", (float)data*0.25);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.3f in H2O", (float)data*0.25/249.088908);
    }
}


void baro_pressure_formula(int data, char *buf, size_t bufSize)
{
    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i kPa", data);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.1f inHg", (float)data*0.2953);
    }
}


void o2_sensor_wrc_formula(int data, char *buf, size_t bufSize)
{
    float eq_ratio, o2_ma; // equivalence ratio and sensor current

    eq_ratio = (float)(data >> 16)*(float)0.0000305;  // data bytes A,B
    o2_ma = ((float)(data & 0xFFFF) - 0x8000)*(float)0.00390625; // data bytes C,D

    StringCchPrintf(buf, bufSize, "%.3f mA, Eq. ratio: %.3f", o2_ma, eq_ratio);
}


void cat_temp_formula(int data, char *buf, size_t bufSize)
{
    float c, f;

    c = (float)data*(float)0.1 - (float)40; // degrees Celcius
    f = c*(float)9.0/(float)5.0 + (float)32.0;  // degrees Fahrenheit

    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%.1f%c C", c, 0xB0);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%.1f%c F", f, 0xB0);
    }
}


void ecu_voltage_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.3f V", (float)data*0.001);
}


void abs_load_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100/255);
}


void eq_ratio_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.3f", (float)data*0.0000305);
}


void relative_tp_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100.0/255.0);
}


void amb_air_temp_formula(int data, char *buf, size_t bufSize)
{
    int c, f;

    c = data-40; // degrees Celcius
    f = (int)((float)c*(float)9.0/(float)5.0 + (float)32.0);  // degrees Fahrenheit

    if (system_of_measurements == METRIC)
    {
        StringCchPrintf(buf, bufSize, "%i%c C", c, 0xB0);
    }
    else
    {
        StringCchPrintf(buf, bufSize, "%i%c F", f, 0xB0);
    }
}


void abs_tp_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100.0/255.0);
}


void tac_pct_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%.1f%%", (float)data*100.0/255.0);
}


void mil_time_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%i hrs %i min", data/60, data%60);
}


void clr_time_formula(int data, char *buf, size_t bufSize)
{
    StringCchPrintf(buf, bufSize, "%i hrs %i min", data/60, data%60);
}
