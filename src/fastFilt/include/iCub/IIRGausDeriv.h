// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file IIRGausDeriv.h
 * \brief Coefficients and poles of IIR Gaussian Derivative Filters (order 0, 1 and 2).
 * \see From "Recursive Gaussian Derivative Filters", L.van.Vliet,I.T.Young and P.W.Verbeek, 1998
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */

#ifndef _IIRGAUSDERIV_H_
#define _IIRGAUSDERIV_H_


#include <complex>
using namespace std;

/**
* Coefficients for scale = 2 filters
*/

extern const complex<double> d0_N3_L2[];   /** 3 tap gaussian filter with L2 norm approximation*/ 
extern const complex<double> d0_N4_L2[];   /** 4 tap gaussian filter with L2 norm approximation*/ 
extern const complex<double> d0_N5_L2[];   /** 5 tap gaussian filter with L2 norm approximation*/ 
extern const complex<double> d0_N3_Linf[]; /** 3 tap gaussian filter with Linf norm approximation*/ 
extern const complex<double> d0_N4_Linf[]; /** 4 tap gaussian filter with Linf norm approximation*/
extern const complex<double> d0_N5_Linf[]; /** 5 tap gaussian filter with Linf norm approximation*/
extern const complex<double> d1_N3_Linf[]; /** 3 tap first derivative filter with Linf norm approximation*/
extern const complex<double> d1_N4_Linf[]; /** 4 tap first derivative filter with Linf norm approximation*/
extern const complex<double> d1_N5_Linf[]; /** 5 tap first derivative filter with Linf norm approximation*/
extern const complex<double> d2_N3_Linf[]; /** 3 tap second derivative filter with Linf norm approximation*/
extern const complex<double> d2_N4_Linf[]; /** 4 tap second derivative filter with Linf norm approximation*/
extern const complex<double> d2_N5_Linf[]; /** 5 tap second derivative filter with Linf norm approximation*/

/**
* Compute the poles of the filter for scale s, given the poles at scale 2
* \param taps Number of taps (3, 4, or 5)
* \param scale Required filter scale (values between 1 and 100 are OK).
* \param oldpoles Poles of the scale 2 filter.
* \param newpoles Poles of the computed filter
*/
void calc_poles(int taps, const double scale, const complex<double> oldpoles[], complex<double> newpoles[] );



//compute the coefficients of the filter for scale s given the poles at scale 2
//the output is written in array coeffs - the first element is the gain and 
//the remaining elements correspond to the autoregressive coefficients 
/**
* Compute the coefficients of the filter for scale s, given the poles at scale 2.
* \param taps Number of taps (3, 4, or 5)
* \param poles Poles of the scale 2 filter.
* \param scale Required filter scale (values between 1 and 100 are OK).
* \param coeffs Computed coefficients \f$(b_0, a_1, a_2, a_3)\f$:
* - \f$ b_0 \f$ is the gain
* - \f$ (a_1, a_2, a_3)\f$ are the autoregressive coefficients
* .
*/
void calc_coeffs(int taps, const complex<double> poles[], const double s, float *coeffs);


/**
* Compute the coefficients of the filter, given its poles
* \param taps Number of taps (3, 4, or 5)
* \param poles Poles of the filter.
* \param coeffs Computed coefficients \f$(b_0, a_1, a_2, a_3)\f$:
* - \f$ b_0 \f$ is the gain
* - \f$ (a_1, a_2, a_3)\f$ are the autoregressive coefficients
* .
*/
void calc_coeffs(int taps, const complex<double> poles[], float *coeffs);

#endif
