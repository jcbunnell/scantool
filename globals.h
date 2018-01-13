#ifndef GLOBALS_H
#define GLOBALS_H

#define FALSE               0
#define TRUE                1

// system_of_measurements
#define METRIC     0
#define IMPERIAL    1

#ifndef WINDDK
#ifdef WIN_VS6
//#include <winbase.h>
#else // WIN_VS6
#include <unistd.h>
#define HANDLE  int
#define DWORD   long
#define Sleep(x)   sleep((x)*1000)
#endif // WIN_VS6
#endif // WINDDK

#ifndef StringCchPrintf
#define StringCchPrintf    snprintf
#endif // StringCchPrintf

#ifndef StringCchCopy
#define StringCchCopy(a,b,c) strncpy(a,c,b)
#endif // StringCchCopy

#ifndef StringCchCatN
#define StringCchCatN strncat
#endif // StringCchCatN

#endif  /* GLOBALS_H */
