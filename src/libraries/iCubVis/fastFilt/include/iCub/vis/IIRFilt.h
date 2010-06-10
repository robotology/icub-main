// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file IIRFilt.h
 * \brief Functions for generic, 3 tap, iir filtering.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */

#ifndef _IIRFILT_H_
#define _IIRFILT_H_


/**
 * Functions iir_filt_forward and iir_filt_backward do a 3 tap recursive
 * filtering operation on floating point buffers.
 *
 * Forward filtering:
 * \f[
 *      y(t) = b_0 x(t) - a_1 y(t-1) - a_2 y(t-2) - a_3 y(t-3)
 * \f]
 *
 * \param in Floating point buffer with the input signal.
 * \param stepin Spacing between consecutive input signal samples.
 * \param out Floating point buffer to store the computation results.
 * \param stepout Spacing between consecutive output buffer samples.
 * \param length Number of samples to process (N) 
 * \param coeffs Filter coefficients. A length 4 floating point array containing \f$(b_0, a_1, a_2, a_3)\f$
 * \param i0 Filter boundary conditions. A length 3 floating point array containing:
 * - in the forward filtering case \f$(y_{-1}, y_{-2}, y_{-3})\f$
 * - in the backward filtering case \f$(y_{N}, y_{N+1}, y_{N+2})\f$
 * .
 * \note Several overloads exist for the cases when stepin or stepout parameters are unitary.
 */


void iir_filt_forward(float *in, int stepin, float *out, int stepout, int length,  float *coeffs, float *i0);

void iir_filt_backward(float *in, int stepin, float *out, int stepout, int length,  float *coeffs, float *i0);

void iir_filt_forward(float *in, float *out, int length,  float *coeffs, float *i0);

void iir_filt_backward(float *in, float *out, int length,  float *coeffs, float *i0);

void iir_filt_forward(float *in, float *out, int stepout, int length,  float *coeffs, float *i0);

void iir_filt_backward(float *in, float *out, int stepout, int length,  float *coeffs, float *i0);

void iir_filt_forward(float *in, int stepin, float *out, int length,  float *coeffs, float *i0);

void iir_filt_backward(float *in, int stepin, float *out, int length,  float *coeffs, float *i0);


#endif



