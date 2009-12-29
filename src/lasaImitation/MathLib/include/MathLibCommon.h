#ifndef MATHLIBCOMMON_H_
#define MATHLIBCOMMON_H_

#ifdef WIN32
#include "windows.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <iostream>

#define USE_MATHLIB_NAMESPACE

#define MATHLIB_USE_DOUBLE_AS_REAL

#ifdef MATHLIB_USE_DOUBLE_AS_REAL
  typedef double REALTYPE;
#else
  typedef float REALTYPE;
#endif

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif



#endif /*MATHLIBCOMMON_H_*/
