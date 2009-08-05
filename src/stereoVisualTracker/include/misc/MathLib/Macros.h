#ifndef __MATHMACROS_H_
#define __MATHMACROS_H_

#include <math.h>
#include <stdlib.h>
#include <time.h>

#ifndef PIf
#define PIf 3.14159265358979323846f
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*(PIf/180.0f))
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*(180.0f/PIf))
#endif

#ifndef RANDOMIZE
#define RANDOMIZE   (srand(time(NULL)));
#endif

#ifndef RND
#define RND(x) (float((x)*((double)rand())/((double)(RAND_MAX+1.0))))
#endif

#ifndef MIN
#define MIN(x,y) (((x)<(y))?(x):(y))
#endif

#ifndef MAX
#define MAX(x,y) (((x)>(y))?(x):(y))
#endif

#ifndef TRUNC
#define TRUNC(x,mn,mx) (MIN(MAX((x),(mn)),(mx)))
#endif

#ifndef SIGN
#define SIGN(x) (((x)<0.0f)?(-1.0f):(1.0f))
#endif

#ifndef SIGN2
#define SIGN2(a,b) ((b) >= 0.0f ? fabs(a) : -fabs(a))
#endif

#ifndef ROUND
#define ROUND(x) (floor((x)+0.5f))
#endif

#ifndef EPSILON
#define EPSILON   (1e-6)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

inline float hypot_s(float a, float b){
  float r;
  if (fabs(a) > fabs(b)) {
      r = b / a;
      r = fabs(a) * sqrtf(1.0f + r * r);
  } else if (b != 0.0f) {
      r = a / b;
      r = fabs(b) * sqrtf(1.0f + r * r);
  } else {
      r = 0.0f;
  }
  return r;
} 

#endif
