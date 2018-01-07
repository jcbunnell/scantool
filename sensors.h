#ifndef SENSORS_H
#define SENSORS_H

#define PID_SIZE        2
#define RESPONSE_SIZE   2
#define OUTPUT_BUFFER_SIZE  1024

#ifdef WIN_PRINTF
#define IDC_VEHICLEVINVALUE             0
#define IDC_SPEEDNUMVALUE               0
#define IDC_BAROMETRICPRESSUREVALUE     0
#define IDC_TROUBLECODESLIST            0
#define IDC_AMBIENTAIRTEMPVALUE         0
#define IDC_FUELEVELVALUE               0
#define IDC_VEHICLESPEEDVALUE           0
#define IDC_ENGINERPMVALUE              0
#define IDC_RELTHROTTLEVALUE            0
#define IDC_VEHICLEVINLABEL             0
#define IDC_ENGINERPMLABEL              0
#define IDC_VEHICLESPEEDLABEL           0
#define IDC_AMBIENTAIRTEMPLABEL         0
#define IDC_FUELLEVELLABEL              0
#define IDC_THROTTLEPOSNLABEL           0
#define IDC_TROUBLECODESLABEL           0
#define IDC_RPMNUMVALUE                 0
#define IDC_THROTTLENUMVALUE            0
#define IDC_FUELNUMVALUE                0
#define IDC_INTAKEAIRTEMPVALUE          0
#define IDC_INTAKEAIRTEMPLABEL          0
#define IDC_DISTANCEECURESETVALUE       0
#define IDC_DISTANCEECURESETLABEL       0
#define IDC_BAROMETRICPRESSURELABEL     0
#define IDC_ABSENGINELOADVALUE          0
#define IDC_ABSENGINELOADLABEL          0
#define IDC_INTAKEMANIFOLDPRESSURELABEL 0
#define IDC_INTAKEMANIFOLDPRESSUREVALUE 0
#define IDC_COOLANTTEMPVALUE            0
#define IDC_COOLANTTEMPLABEL            0
#define IDC_OBDCONFORMSLABEL            0
#define IDC_OBDCONFORMSTOVALUE          0
#define IDC_ECUVOLTAGELABEL             0
#define IDC_ECUVOLTAGEVALUE             0
#define IDC_CALCULATEDLOADLABEL         0
#define IDC_CALCULATEDLOADVALUE         0
#define IDC_O2SENSOR1BANK1LABEL         0
#define IDC_O2SENSOR1BANK1VALUE         0
#define IDC_O2SENSOR2BANK1LABEL         0
#define IDC_O2SENSOR1BANK2LABEL         0
#define IDC_O2SENSOR2BANK2LABEL         0
#define IDC_O2SENSOR2BANK1VALUE         0
#define IDC_O2SENSOR1BANK2VALUE         0
#define IDC_O2SENSOR2BANK2VALUE         0
#define IDC_DISTANCESINCEMILLABEL       0
#define IDC_DISTANCESINCEMILVALUE       0
#define IDC_ENGINERUNMILLABEL           0
#define IDC_ENGINERUNMILVALUE           0
#define IDC_TIMESINCEDTCCLEAREDLABEL    0
#define IDC_TIMESINCEDTCLEAREDVALUE     0
#define IDC_WARMUPSSINCECURESETVALUE    0
#define IDC_WARMUPSSINCECURESETLABEL    0
#define IDC_FUELSYSTEM1STATUSLABEL      0
#define IDC_FUELSYSTEM1STATUSVALUE      0
#define IDC_TIMESINCEENGINESTARTLABEL   0
#define IDC_TIMESINCEENGINESTARTVALUE   0
#define IDC_MODELYEARLABEL              0
#define IDC_MODELYEARVALUE              0
#endif  /* WIN_PRINTF*/

void obd_requirements_formula(int data, char *buf, size_t bufSize);
void process_and_display_data(char *buf, char *simBuffer);
int codeIsDisplayed(unsigned long index);

#endif
