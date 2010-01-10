#ifndef __MATHMACROS_H_
#define __MATHMACROS_H_

/**
 * \file Macros
 * 
 * \ingroup MathLib
 * 
 * \brief Here is a set of useful math Macros
 */

#include "MathLibCommon.h"

/// Constants
#ifndef R_ZERO
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define R_ZERO 0.0
  #else
    #define R_ZERO 0.0f
  #endif
#endif

#ifndef R_ONE
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define R_ONE 1.0
  #else
    #define R_ONE 1.0f
  #endif
#endif


/// The PI constant
#ifndef PI
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define PI 3.14159265358979323846
  #else
    #define PI 3.14159265358979323846f
  #endif
#endif
#ifndef PIf
  #define PIf 3.14159265358979323846f
#endif

#ifndef TWOPI
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define TWOPI (2.0*PI)
  #else
    #define TWOPI (2.0f*PIf)
  #endif
#endif
#ifndef TWOPIf
  #define TWOPIf (2.0f*PIf)
#endif

/// Convert degrees to radians
#ifndef DEG2RAD
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define DEG2RAD(x) ((x)*(PI/180.0))
  #else
    #define DEG2RAD(x) ((x)*(PI/180.0f))
  #endif
#endif

/// Convert radians to degrees
#ifndef RAD2DEG
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define RAD2DEG(x) ((x)*(180.0f/PI))
  #else
    #define RAD2DEG(x) ((x)*(180.0/PI))
  #endif
#endif

/// Reset the seed of the randomizer 
#ifndef RANDOMIZE
#define RANDOMIZE   (srand(time(NULL)));
#endif

/// Give a random number uniformliy distributed between 0 and 1 
#ifndef RND
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define RND(x) ((((double)rand())/((double)(RAND_MAX+1.0)))*(x))
  #else
    #define RND(x) ((((float)rand())/((float)(RAND_MAX+1.0)))*(x))
  #endif
#endif

/// The miminum between two values
#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

/// The maximum between two values
#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif

/// Return a value, trunked between a minimum and a maximum 
#ifndef TRUNC
#define TRUNC(x,mn,mx) (MIN(MAX((x),(mn)),(mx)))
#endif

/// Trunc the variable between -pi and pi
#ifndef PTRUNC
#define PTRUNC(x) {while((x)<-PI) (x)+=TWOPI; while((x)>PI) (x)-=TWOPI;}
#endif

/// Return the sign of a variable.  
#ifndef SIGN
#define SIGN(x) (((x)<0.0f)?(-R_ONE):(R_ONE))
#endif

/// Return a variable signed according to the sign of another   
#ifndef SIGN2
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define SIGN2(a,b) ((b) >= 0.0f ? fabs(a) : -fabs(a))
  #else
    #define SIGN2(a,b) ((b) >= 0.0f ? fabsf(a) : -fabsf(a))
  #endif
#endif

/// Round a variable
#ifndef ROUND
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define ROUND(x) (floor((x)+0.5))
  #else
    #define ROUND(x) float(floor((x)+0.5f))
  #endif
#endif

/// AN arbitrary mimimal error bound
#ifndef EPSILON
  #ifdef MATHLIB_USE_DOUBLE_AS_REAL
    #define EPSILON   (1e-12)
  #else
    #define EPSILON   (1e-6f)
  #endif
#endif

/// A wise way to get the hypothenuse
inline REALTYPE hypot_s(REALTYPE a, REALTYPE b){
  REALTYPE r;
#ifdef MATHLIB_USE_DOUBLE_AS_REAL
  if (fabs(a) > fabs(b)) {
      r = b / a;
      r = fabs(a) * sqrt(1.0   + r * r);
  } else if (b != 0.0) {
      r = a / b;
      r = fabs(b) * sqrt(1.0 + r * r);
  } else {
      r = 0.0;
  }
  return r;
#else
  if (fabsf(a) > fabsf(b)) {
      r = b / a;
      r = fabsf(a) * sqrtf(1.0f + r * r);
  } else if (b != 0.0f) {
      r = a / b;
      r = fabsf(b) * sqrtf(1.0f + r * r);
  } else {
      r = 0.0f;
  }
  return r;
#endif
} 

#endif
