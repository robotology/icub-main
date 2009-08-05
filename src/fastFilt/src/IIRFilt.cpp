// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file IIRFilt.h
 * \brief Functions for generic, 3 tap, iir filtering.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * \see IIRFilt.h
 */


#include "iCub/IIRFilt.h"

void iir_filt_forward(float *in, int stepin, float *out, int stepout, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[0] = b0*in[0] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[1*stepout] = b0*in[stepin] - a1*out[0] - a2*i0[0] - a3*i0[1];
	out[2*stepout] = b0*in[2*stepin] - a1*out[stepout] - a2*out[0] - a3*i0[0];
	for(j = 3; j < length; j++)
		out[j*stepout] = b0*in[j*stepin] - a1*out[(j-1)*stepout] - a2*out[(j-2)*stepout] - a3*out[(j-3)*stepout];
}


void iir_filt_backward(float *in, int stepin, float *out, int stepout, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[(length-1)*stepout] = b0*in[(length-1)*stepin] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[(length-2)*stepout] = b0*in[(length-2)*stepin] - a1*out[(length-1)*stepout] - a2*i0[0] - a3*i0[1]; 
	out[(length-3)*stepout] = b0*in[(length-3)*stepin] - a1*out[(length-2)*stepout] - a2*out[(length-1)*stepout] - a3*i0[0]; 
	for(j = length-4; j >= 0; j--)
		out[j*stepout] = b0*in[j*stepin] - a1*out[(j+1)*stepout] - a2*out[(j+2)*stepout] - a3*out[(j+3)*stepout];
}

void iir_filt_forward(float *in, float *out, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[0] = b0*in[0] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[1] = b0*in[1] - a1*out[0] - a2*i0[0] - a3*i0[1];
	out[2] = b0*in[2] - a1*out[1] - a2*out[0] - a3*i0[0];
	for(j = 3; j < length; j++)
		out[j] = b0*in[j] - a1*out[j-1] - a2*out[j-2] - a3*out[j-3];
}


void iir_filt_backward(float *in, float *out, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[length-1] = b0*in[length-1] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[length-2] = b0*in[length-2] - a1*out[length-1] - a2*i0[0] - a3*i0[1]; 
	out[length-3] = b0*in[length-3] - a1*out[length-2] - a2*out[length-1] - a3*i0[0]; 
	for(j = length-4; j >= 0; j--)
		out[j] = b0*in[j] - a1*out[j+1] - a2*out[j+2] - a3*out[j+3];
}

void iir_filt_forward(float *in, float *out, int stepout, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[0] = b0*in[0] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[1*stepout] = b0*in[1] - a1*out[0] - a2*i0[0] - a3*i0[1];
	out[2*stepout] = b0*in[2] - a1*out[stepout] - a2*out[0] - a3*i0[0];
	for(j = 3; j < length; j++)
		out[j*stepout] = b0*in[j] - a1*out[(j-1)*stepout] - a2*out[(j-2)*stepout] - a3*out[(j-3)*stepout];
}


void iir_filt_backward(float *in, float *out, int stepout, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[(length-1)*stepout] = b0*in[length-1] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[(length-2)*stepout] = b0*in[length-2] - a1*out[(length-1)*stepout] - a2*i0[0] - a3*i0[1]; 
	out[(length-3)*stepout] = b0*in[length-3] - a1*out[(length-2)*stepout] - a2*out[(length-1)*stepout] - a3*i0[0]; 
	for(j = length-4; j >= 0; j--)
		out[j*stepout] = b0*in[j] - a1*out[(j+1)*stepout] - a2*out[(j+2)*stepout] - a3*out[(j+3)*stepout];
}

void iir_filt_forward(float *in, int stepin, float *out, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[0] = b0*in[0] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[1] = b0*in[stepin] - a1*out[0] - a2*i0[0] - a3*i0[1];
	out[2] = b0*in[2*stepin] - a1*out[1] - a2*out[0] - a3*i0[0];
	for(j = 3; j < length; j++)
		out[j] = b0*in[j*stepin] - a1*out[j-1] - a2*out[j-2] - a3*out[j-3];
}


void iir_filt_backward(float *in, int stepin, float *out, int length,  float *coeffs, float *i0)
{
	int j;
	float b0 = coeffs[0];
	float a1 = coeffs[1]; 
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	out[length-1] = b0*in[(length-1)*stepin] - a1*i0[0] - a2*i0[1] - a3*i0[2];
	out[length-2] = b0*in[(length-2)*stepin] - a1*out[length-1] - a2*i0[0] - a3*i0[1]; 
	out[length-3] = b0*in[(length-3)*stepin] - a1*out[length-2] - a2*out[length-1] - a3*i0[0]; 
	for(j = length-4; j >= 0; j--)
		out[j] = b0*in[j*stepin] - a1*out[j+1] - a2*out[j+2] - a3*out[j+3];
}
