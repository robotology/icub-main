// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Eric Sauser, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __MATHMACROS_H_
#define __MATHMACROS_H_
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef PIf
#define PIf 3.14159265358979323846f
#endif

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*(PIf/180.0f))
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*(180.0f/PIf))
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

#ifndef SIGN2
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
