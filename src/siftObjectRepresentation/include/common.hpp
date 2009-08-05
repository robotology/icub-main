// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Authors: 2007 Bruno Damas
 *          2008 Dario Figueira
 *      Institute for Systems and Robotics (IST)
 *      IST, TULisbon, Portugal
 */

#ifndef __Common__B
#define __Common__B

#include <cmath>   // remainder


//typedef float Float;
//typedef double Float;


#ifndef NULL
#define NULL 0
#endif

/*
#ifndef true
#define true 1
#endif
#ifndef false
#define false 0
#endif
*/

#define NOT !


#ifndef SQUARE
#define SQUARE(A) ((A)*(A))
#endif


#ifndef SQR
#define SQR(A) ((A)*(A))
#endif

#define INF_PLUS +10000000000.0
/*
#ifndef MIN
#define MIN(A,B) ((A) > (B) ? (B) : (A))
#endif

#ifndef MAX
#define MAX(A,B) ((A) < (B) ? (B) : (A))
#endif
*/

#define STEP( A )      ( ( A ) >= 0 ? 1 : 0 )
#define RAMP( A )      ( ( A ) >= 0 ? ( A ) : 0 )
#define SIGN( A )      ( ( A ) >= 0 ? 1 : -1 )

#define CLIP_DEG( A )  ( remainder( ( A ), 360.0 ) )
#define CLIP_RAD( A )  ( remainder( ( A ), 2*M_PI ) )
//#define CLIP_RAD(A) (fabs(fmod((A),2*M_PI)) > M_PI ? fmod((A),2*M_PI) - 2*M_PI*SIGN((A)) : fmod((A),2*M_PI))



#endif //__Common__B
