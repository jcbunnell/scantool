#ifndef GLOBALS_H
#define GLOBALS_H

#define FALSE               0
#define TRUE                1

// system_of_measurements
#define METRIC     0
#define IMPERIAL    1

#ifdef WIN_VS6
#include <windows.h>
#endif // WIN_VS6

#ifdef WINDDK
#include <windows.h>
#include <strsafe.h>
#endif // WINDDK

#ifdef WINWATCOM
#include <windows.h>
#include <strsafe.h>
#endif // WINWATCOM

#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#ifndef StringCchPrintf
#define StringCchPrintf    snprintf
#endif // StringCchPrintf

#ifndef StringCchCopy
#define StringCchCopy(a,b,c) strncpy(a,c,b)
#endif // StringCchCopy

#ifndef StringCchCatN
#define StringCchCatN strncat
#endif // StringCchCatN

#ifndef NULL
#define NULL    0L
#endif

#endif  /* GLOBALS_H */
