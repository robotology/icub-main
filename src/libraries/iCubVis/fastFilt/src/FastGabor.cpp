// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file FastGabor.cpp
 * \brief Implements Gabor Filters by Gaussian Filtering exponentially modulated images.
 * \see "Fast IIR Isotropic 2-D Complex Gabor Filters With Boundary Initialization", A. Bernardino and J. Santos-Victor, 2006.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */

#include <iCub/vis/FastGabor.h>
#include <iCub/vis/IIRFilt.h>
//#include "iCub/IIRGausDeriv.h"

//#include <memory.h>
//#include <malloc.h>

#include <string.h>
#include <stdlib.h>
#include <cmath>

bool FastGabor::GaborFilt(float * input, float * real, float * imag )
{	
	if(!IsAllocated())
		throw "Resources not allocated";

    //modulation
	_image_prod(input, cosine, even);
	_image_prod(input, sine, odd);

    //filtering
    _iir_gaborfilt3(input, even, odd, real, imag, temp, GetCols(), GetLines(), GetStridePix(), filt_coeffs, m_wx, m_wy, 
		m_magnitudex, m_magnitudey, m_phasex, m_phasey, m_resid_ic, m_resid_step, 
		m_resid_cosx, m_resid_sinx, m_resid_cosy, m_resid_siny,
		upper_boundary, lower_boundary);
    
    // demodulation                      
	_image_prod(real, cosine, even );
	_image_prod(imag, sine, odd );	
	_image_prod(real, sine, temp2 );
	_image_prod(imag, cosine, temp );
    _image_sub(temp2, temp, imag);
    _image_add(even, odd, real);


	return true;
}

bool FastGabor::GaborFiltZeroMean(float * input, float * real, float * imag )
{
     GaborFilt(input,real,imag);
     GaussFilt(input, temp);
     RemoveDC(real, temp, real);
     return true;
}

bool FastGabor::ComputeMagnitude(float * real, float * imag, float * magn )
{	
    _image_prod(real,real,even);
	_image_prod(imag,imag,odd);
	_image_add(even,odd,temp);					
	_image_sqrt(temp,magn);
    return true;
}

bool FastGabor::RemoveDC(float * real, float * gauss, float * zeromeanreal)
{
    _const_prod(gauss,temp,(float)m_dc_gain);
	_image_sub(real,temp,zeromeanreal);
    return true;
}


/*********************************************************************************
Constructor():
	Sets all variables to NULL		
**********************************************************************************/
FastGabor::FastGabor(void)
{
	//Initialization of constants and variables
	pi = 3.14159265358979323846;
	sine = NULL;
	cosine = NULL;
	even = NULL;
	odd = NULL;
	temp = NULL;
	temp2 = NULL;
	m_wavelength = 0;
	m_orientation = 0;
	m_dc_gain = 0;
	m_magnitudex = 0;
	m_phasex = 0;
	m_magnitudey = 0;
	m_phasey = 0;
	m_wx = 0;
	m_wy = 0;
	m_bAllocated = false;
	upper_boundary = NULL;
	lower_boundary = NULL;
	m_resid_cosx = NULL;
	m_resid_sinx = NULL;
	m_resid_cosy = NULL;
	m_resid_siny = NULL;
}

/*********************************************************************************
Destructor():
	Releases all allocated resources and exits
**********************************************************************************/
FastGabor::~FastGabor(void)
{
	FreeResources();
}

/*********************************************************************************
IsAllocated():
	Check if memory resources have already been allocated		
**********************************************************************************/
bool FastGabor::IsAllocated()
{
    return (FastGauss::IsAllocated() && m_bAllocated);
}


/*********************************************************************************
GetDcGain():
	returns the zero mean correction term for the Gabor real part
**********************************************************************************/
double FastGabor::GetDcGain()
{
	if(m_dc_gain)
		return m_dc_gain;
	return 0;
}
/*********************************************************************************
AllocateResources():
	Creates internal structures and memory space for the required computations
	lines : number of lines of the image to process
	cols : number of columns of the image to process
	nlevels : number of scales in the gabor filter bank
	scales : vector containing the scale values
	norients : number of orientations in the gabor filter bank
	orients : vector containing the orientation values (in degrees)
	nwav : number of wavelenghths in the gabor filter bank 
	wavs : vector containing the wavelenght values (in pixels)
	sw_table : boolean matrix of "nlevels" lines and "nwavs" columns indicating 
	           which combinations scale/wavelength are to process
**********************************************************************************/
bool FastGabor::AllocateResources(long lines, long cols, double scale, double orient /**degrees*/, double wav )
{
	int m,n;
	double angle;

    if(FastGauss::IsAllocated())
        FastGauss::FreeResources();

    if(IsAllocated())
		FreeResources();

    FastGauss::AllocateResources(lines, cols, scale);
	m_orientation = orient*pi/180.0f;
	m_wavelength = wav;
    m_resid_cosx = (float*)malloc(3*sizeof(float));
    m_resid_sinx = (float*)malloc(3*sizeof(float));
	m_resid_cosy = (float*)malloc(3*sizeof(float));
	m_resid_siny = (float*)malloc(3*sizeof(float));
	temp = (float*)malloc(GetLines()*GetCols()*sizeof(float));
    temp2 = (float*)malloc(GetLines()*GetCols()*sizeof(float));
	even = (float*)malloc(GetLines()*GetCols()*sizeof(float));
	odd = (float*)malloc(GetLines()*GetCols()*sizeof(float));
	sine = (float*)malloc(GetLines()*GetCols()*sizeof(float));
	cosine = (float*)malloc(GetLines()*GetCols()*sizeof(float));
	upper_boundary = (float*)malloc(GetCols()*sizeof(float));
	lower_boundary = (float*)malloc(GetCols()*sizeof(float));
    m_wx = 2*pi/m_wavelength*cos(m_orientation);
	m_wy = 2*pi/m_wavelength*sin(m_orientation);
	
	//initialization of carrier images
	for( m = 0; m < GetLines(); m++)
	{
		for( n = 0; n < GetCols(); n++ )
		{
			angle = n*m_wx - m*m_wy;
			cosine[m*GetStridePix()+n] = (float)cos(angle);
			sine[m*GetStridePix()+n] = (float)sin(angle);
		}
	}
	m_dc_gain = calc_dcvalue(filt_coeffs, m_wx, m_wy);
	_compute_gabor3_horz_resids(filt_poles, filt_coeffs, m_wx, m_resid_cosx, m_resid_sinx);
	calc_frequency_response(filt_coeffs, m_wx, &m_magnitudex, &m_phasex);
	_compute_gabor3_vert_resids(filt_poles, filt_coeffs, m_wy, m_resid_cosy, m_resid_siny);					
	calc_frequency_response(filt_coeffs, m_wy, &m_magnitudey, &m_phasey);
	m_bAllocated = true;
	return true;
}


/*********************************************************************************
FreeResources():
	Deletes internal structures and memory space allocated by AllocateResources
	Careful: Depends on the parameters 'm_i_orientations', 'm_i_scales',
				'm_i_kernels', 'm_i_wavelengths'. Do not change these values
				after calling AllocateResources
**********************************************************************************/
bool FastGabor::FreeResources()
{

	m_bAllocated = false;
	
	if(m_resid_cosx != NULL)
	{
		free(m_resid_cosx);
		m_resid_cosx = NULL;
	}
	if(m_resid_sinx != NULL)
	{
		free(m_resid_sinx);
		m_resid_sinx = NULL;
	}

	if(m_resid_cosy != NULL)
	{
		free(m_resid_cosy);
		m_resid_cosy = NULL;
	}
	if(m_resid_siny != NULL)
	{
        free(m_resid_siny);
		m_resid_siny = NULL;
	}

	if(upper_boundary != NULL)
	{
		free(upper_boundary);
		upper_boundary = NULL;
	}
	if(lower_boundary != NULL)
	{
		free(lower_boundary);
		lower_boundary = NULL;
	}

	if(sine != NULL)
		free(sine);		
	
	if(cosine != NULL)
		free(cosine);		
	
	if(even != NULL) 
		free(even);
	
	if(odd != NULL) 
		free(odd);
		
	if(temp != NULL) 
		free(temp);	
    if(temp2 != NULL) 
		free(temp2);	
	sine = NULL;
	cosine = NULL;
	even = NULL;
	odd = NULL;
	temp = NULL;
    temp2 = NULL;
    return FastGauss::FreeResources();
}



/**********************************************************************
BASIC IMAGE PROCESSING FUNCTIONS
***********************************************************************/
void FastGabor::_image_prod(float * i1, float * i2, float * out)
{
	int i, j;
	for(i = 0; i < m_i_lines; i++)
		for(j = 0; j < m_i_cols; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j]*i2[i*m_i_stridepix+j];
}
void FastGabor::_image_add(float * i1, float * i2, float * out)
{
	int i, j;
	for(i = 0; i < m_i_lines; i++)
		for(j = 0; j < m_i_cols; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j] + i2[i*m_i_stridepix+j];
}
void FastGabor::_image_sub(float * i1, float * i2, float * out)
{
	int i, j;
	for(i = 0; i < m_i_lines; i++)
		for(j = 0; j < m_i_cols; j++)
			out[i*m_i_stridepix+j] = i1[i*m_i_stridepix+j] - i2[i*m_i_stridepix+j];
}
void FastGabor::_const_prod(float * in, float * out, float gain)
{
	int i, j;
	for(i = 0; i < m_i_lines; i++)
		for(j = 0; j < m_i_cols; j++)
			out[i*m_i_stridepix+j] = gain*in[i*m_i_stridepix+j];
}
void FastGabor::_image_sqrt(float * in, float * out)
{
	int i, j;
	for(i = 0; i < m_i_lines; i++)
		for(j = 0; j < m_i_cols; j++)
			out[i*m_i_stridepix+j] = (float)sqrt(in[i*m_i_stridepix+j]);
}



//
//compute the dc value of the 2D filter modulated by a complex exponential
//with horizontal and vertical frequencies wx,wy.
//
double FastGabor::calc_dcvalue(float *coeffs, double wx, double wy)
{
	double t1,t2;
	complex<double> temp1, temp2, temp3, temp4;
	complex<double> pureimag(0.0,1.0);
	complex<double> z1x, z2x, z3x, z4x, z5x, z_1x, z_2x, z_3x, z_4x, z_5x, z1y, z2y, z3y, z4y, z5y, z_1y, z_2y, z_3y, z_4y, z_5y;
	z1x = exp(pureimag*wx); 
	z2x = exp(pureimag*wx*2.0);
	z3x = exp(pureimag*wx*3.0);
	z4x = exp(pureimag*wx*4.0);
	z5x = exp(pureimag*wx*5.0);
	z_1x = exp(-pureimag*wx); 
	z_2x = exp(-pureimag*wx*2.0);
	z_3x = exp(-pureimag*wx*3.0);
	z_4x = exp(-pureimag*wx*4.0);
	z_5x = exp(-pureimag*wx*5.0);
	z1y = exp(pureimag*wy); 
	z2y = exp(pureimag*wy*2.0);
	z3y = exp(pureimag*wy*3.0);
	z4y = exp(pureimag*wy*4.0);
	z5y = exp(pureimag*wy*5.0);
	z_1y = exp(-pureimag*wy); 
	z_2y = exp(-pureimag*wy*2.0);
	z_3y = exp(-pureimag*wy*3.0);
	z_4y = exp(-pureimag*wy*4.0);
	z_5y = exp(-pureimag*wy*5.0);

	double b0, a1, a2, a3, a4, a5;
	b0 = coeffs[0];
	a1 = coeffs[1];
	a2 = coeffs[2];
	a3 = coeffs[3];
	a4 = coeffs[4];
	a5 = coeffs[5];

	temp1 = 1.0 + z1x*a1 + z2x*a2 + z3x*a3 + z4x*a4 + z5x*a5;
	temp2 = 1.0 + z_1x*a1 + z_2x*a2 + z_3x*a3 + z_4x*a4 + z_5x*a5;
	temp3 = 1.0 + z1y*a1 + z2y*a2 + z3y*a3 + z4y*a4 + z5y*a5;
	temp4 = 1.0 + z_1y*a1 + z_2y*a2 + z_3y*a3 + z_4y*a4 + z_5y*a5;
	t1 = real(temp1*temp2*temp3*temp4);
	t2 = pow(b0,4.0);
	if(t1 == 0)
		throw "Divide by zero exception in calc_dcvalue";
	return t2/t1;
}

//
//Compute the magnitude and phase of the 1D gaussian filter 
//modulated by a complex exponential of frequency f
//
void FastGabor::calc_frequency_response(float *filter_coeffs, double freq, double *magnitude, double *phase)
{
	complex<double> freqresp;
	complex<double> pureimag(0.0,1.0);
	complex<double> z1, z2, z3, z4, z5;
	complex<double> w(freq, 0.0);
	z1 = exp(-pureimag*w);
	z2 = exp(-2.0*pureimag*w);
	z3 = exp(-3.0*pureimag*w);
	z4 = exp(-4.0*pureimag*w);
	z5 = exp(-5.0*pureimag*w);

	double b0, a1, a2, a3, a4, a5;
	b0 = filter_coeffs[0];
	a1 = filter_coeffs[1];
	a2 = filter_coeffs[2];
	a3 = filter_coeffs[3];
	a4 = filter_coeffs[4];
	a5 = filter_coeffs[5];

	freqresp = b0/(1.0+a1*z1+a2*z2+a3*z3+a4*z4+a5*z5);
	*magnitude = abs(freqresp);
	*phase = arg(freqresp);
}

/*
void FastGabor::compute_step_forward_ic(float bord_val, float *coeffs, float *i0)
{
	//filter coefficients
	float b0 = coeffs[0];
	float a1 = coeffs[1];
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	i0[0] = i0[1] = i0[2] = bord_val*b0/(1+a1+a2+a3);
}

void FastGabor::compute_natural_backward_ic( float *resid_ic, float *init_cond )
{
	float i0 = init_cond[0];
	float i1 = init_cond[1];
	float i2 = init_cond[2];

	init_cond[0]=i0*resid_ic[0]+i1*resid_ic[1]+i2*resid_ic[2];
	init_cond[1]=i0*resid_ic[3]+i1*resid_ic[4]+i2*resid_ic[5];
	init_cond[2]=i0*resid_ic[6]+i1*resid_ic[7]+i2*resid_ic[8];
}
*/
/*
void FastGabor::add_step_backward_ic( float *resid_step, float val, float *init_cond )
{
	init_cond[0] += val*resid_step[0];
	init_cond[1] += val*resid_step[1];
	init_cond[2] += val*resid_step[2];
}
*/

void FastGabor::add_horz_real_backward_ic(float *resid_cosx, float *resid_sinx, int bord_loc, float bord_val, double phase0, double freq, float *init_cond )
{
	double d = freq*(bord_loc+1)-phase0;
	init_cond[0] += (float)(bord_val*(cos(d)*resid_cosx[0] - sin(d)*resid_sinx[0]));
	init_cond[1] += (float)(bord_val*(cos(d)*resid_cosx[1] - sin(d)*resid_sinx[1]));
	init_cond[2] += (float)(bord_val*(cos(d)*resid_cosx[2] - sin(d)*resid_sinx[2]));
}

void FastGabor::add_horz_imag_backward_ic(float *resid_cosx, float *resid_sinx, int bord_loc, float bord_val, double phase0, double freq, float *init_cond )
{
	double d = freq*(bord_loc+1)-phase0;
	init_cond[0] += (float)(bord_val*(sin(d)*resid_cosx[0] + cos(d)*resid_sinx[0]));
	init_cond[1] += (float)(bord_val*(sin(d)*resid_cosx[1] + cos(d)*resid_sinx[1]));
	init_cond[2] += (float)(bord_val*(sin(d)*resid_cosx[2] + cos(d)*resid_sinx[2]));
}

void FastGabor::add_vert_real_backward_ic(float *resid_cosy, float *resid_siny, float real_bord, float imag_bord,	double freq, float *init_cond )
{
	init_cond[0] += (float)((real_bord*cos(freq)+imag_bord*sin(freq))*resid_cosy[0] + (imag_bord*cos(freq)-real_bord*sin(freq))*resid_siny[0]);
	init_cond[1] += (float)((real_bord*cos(freq)+imag_bord*sin(freq))*resid_cosy[1] + (imag_bord*cos(freq)-real_bord*sin(freq))*resid_siny[1]);
	init_cond[2] += (float)((real_bord*cos(freq)+imag_bord*sin(freq))*resid_cosy[2] + (imag_bord*cos(freq)-real_bord*sin(freq))*resid_siny[2]);
}

void FastGabor::add_vert_imag_backward_ic(float *resid_cosy, float *resid_siny, float real_bord, float imag_bord,	double freq, float *init_cond )
{
	init_cond[0] += (float)((imag_bord*cos(freq)-real_bord*sin(freq))*resid_cosy[0] - (real_bord*cos(freq)+imag_bord*sin(freq))*resid_siny[0]);
	init_cond[1] += (float)((imag_bord*cos(freq)-real_bord*sin(freq))*resid_cosy[1] - (real_bord*cos(freq)+imag_bord*sin(freq))*resid_siny[1]);
	init_cond[2] += (float)((imag_bord*cos(freq)-real_bord*sin(freq))*resid_cosy[2] - (real_bord*cos(freq)+imag_bord*sin(freq))*resid_siny[2]);
}

void FastGabor::_compute_gabor3_horz_resids(complex<double> filt_poles[], float *filt_coeffs, double f, float *resid_cosx, float *resid_sinx)
{
	complex <double> p1, p2, p3, _p1, _p2, _p3;
	complex <double> _p4, _p5, p4, p5;
	complex <double> p1_1, p1_2, p1_3, p2_1, p2_2, p2_3, p3_1, p3_2, p3_3, p4_1, p4_2, p4_3, p5_1, p5_2, p5_3;
	double b0, a1, a2, a3;

	_p1 = filt_poles[0];
	_p2 = filt_poles[1];
	_p3 = filt_poles[2];
	b0 = filt_coeffs[0];
	a1 = filt_coeffs[1];
	a2 = filt_coeffs[2];
	a3 = filt_coeffs[3];

	complex <double> res1, res2, res3, res4, res5;
	complex <double> res1a, res2a, res3a, res1b, res2b, res3b, res1c, res2c, res3c; 

	p1 = 1.0/_p1;
	p2 = 1.0/_p2;
	p3 = 1.0/_p3;
	p1_1  = _p1;
	p1_2 = p1_1*_p1;
	p1_3 = p1_2*_p1;
	p2_1 = _p2;
	p2_2 = p2_1*_p2;
	p2_3 = p2_2*_p2;
	p3_1 = _p3;
	p3_2 = p3_1*_p3;
	p3_3 = p3_2*_p3;
	
	res1a = b0/a3*p1_3/(1.0-p1_2)/(1.0-p1_1*(p2+p2_1)+p1_2)/(1.0-p1_1*(p3+p3_1)+p1_2);
	res2a = b0/a3*p2_3/(1.0-p2_2)/(1.0-p2_1*(p1+p1_1)+p2_2)/(1.0-p2_1*(p3+p3_1)+p2_2);
	res3a = b0/a3*p3_3/(1.0-p3_2)/(1.0-p3_1*(p2+p2_1)+p3_2)/(1.0-p3_1*(p1+p1_1)+p3_2);

	p4 = complex<double>(cos(f), sin(f));
	p5 = complex<double>(cos(f), -sin(f));
	_p4 = 1.0/p4;
	_p5 = 1.0/p5;
	p4_1 = _p4;
	p4_2 = p4_1*_p4;
	p4_3 = p4_2*_p4;
	p5_1 = _p5;
	p5_2 = p5_1*_p5;
	p5_3 = p5_2*_p5;
	
	res1 = (1.0-cos(f)*p1_1)*b0*res1a/(1.0-p4*p1_1)/(1.0-p5*p1_1);
	res2 = (1.0-cos(f)*p2_1)*b0*res2a/(1.0-p4*p2_1)/(1.0-p5*p2_1);
	res3 = (1.0-cos(f)*p3_1)*b0*res3a/(1.0-p4*p3_1)/(1.0-p5*p3_1);
	res4 = (1.0-cos(f)*p4_1)*b0*b0/a3*p4_3/(1.0-p4_2)/(1.0-p4_1*(p1+p1_1)+p4_2)/(1.0-p4_1*(p2+p2_1)+p4_2)/(1.0-p4_1*(p3+p3_1)+p4_2);
	res5 = (1.0-cos(f)*p5_1)*b0*b0/a3*p5_3/(1.0-p5_2)/(1.0-p5_1*(p1+p1_1)+p5_2)/(1.0-p5_1*(p2+p2_1)+p5_2)/(1.0-p5_1*(p3+p3_1)+p5_2);

	resid_cosx[0] = (float)real(res1+res2+res3+res4+res5);
	resid_cosx[1] = (float)real(res1*p1+res2*p2+res3*p3+res4*p4+res5*p5);
	resid_cosx[2] = (float)real(res1*p1*p1+res2*p2*p2+res3*p3*p3+res4*p4*p4+res5*p5*p5);

	res1 = (sin(f)*p1_1)*b0*res1a/(1.0-p4*p1_1)/(1.0-p5*p1_1);
	res2 = (sin(f)*p2_1)*b0*res2a/(1.0-p4*p2_1)/(1.0-p5*p2_1);
	res3 = (sin(f)*p3_1)*b0*res3a/(1.0-p4*p3_1)/(1.0-p5*p3_1);
	res4 = (sin(f)*p4_1)*b0*b0/a3*p4_3/(1.0-p4_2)/(1.0-p4_1*(p1+p1_1)+p4_2)/(1.0-p4_1*(p2+p2_1)+p4_2)/(1.0-p4_1*(p3+p3_1)+p4_2);
	res5 = (sin(f)*p5_1)*b0*b0/a3*p5_3/(1.0-p5_2)/(1.0-p5_1*(p1+p1_1)+p5_2)/(1.0-p5_1*(p2+p2_1)+p5_2)/(1.0-p5_1*(p3+p3_1)+p5_2);

	resid_sinx[0] = (float)real(res1+res2+res3+res4+res5);
	resid_sinx[1] = (float)real(res1*p1+res2*p2+res3*p3+res4*p4+res5*p5);
	resid_sinx[2] = (float)real(res1*p1*p1+res2*p2*p2+res3*p3*p3+res4*p4*p4+res5*p5*p5);
}

void FastGabor::_compute_gabor3_vert_resids(complex<double> filt_poles[], float *filt_coeffs, double f, float *resid_cosy, float *resid_siny)
{
	complex <double> p1, p2, p3, _p1, _p2, _p3;
	complex <double> _p4, _p5, p4, p5;
	complex <double> p1_1, p1_2, p1_3, p2_1, p2_2, p2_3, p3_1, p3_2, p3_3, p4_1, p4_2, p4_3, p5_1, p5_2, p5_3;
	double b0, a1, a2, a3;

	_p1 = filt_poles[0];
	_p2 = filt_poles[1];
	_p3 = filt_poles[2];
	b0 = filt_coeffs[0];
	a1 = filt_coeffs[1];
	a2 = filt_coeffs[2];
	a3 = filt_coeffs[3];

	complex <double> res1, res2, res3, res4, res5;
	complex <double> res1a, res2a, res3a, res1b, res2b, res3b, res1c, res2c, res3c; 

	p1 = 1.0/_p1;
	p2 = 1.0/_p2;
	p3 = 1.0/_p3;
	p1_1  = _p1;
	p1_2 = p1_1*_p1;
	p1_3 = p1_2*_p1;
	p2_1 = _p2;
	p2_2 = p2_1*_p2;
	p2_3 = p2_2*_p2;
	p3_1 = _p3;
	p3_2 = p3_1*_p3;
	p3_3 = p3_2*_p3;
	
	res1a = b0/a3*p1_3/(1.0-p1_2)/(1.0-p1_1*(p2+p2_1)+p1_2)/(1.0-p1_1*(p3+p3_1)+p1_2);
	res2a = b0/a3*p2_3/(1.0-p2_2)/(1.0-p2_1*(p1+p1_1)+p2_2)/(1.0-p2_1*(p3+p3_1)+p2_2);
	res3a = b0/a3*p3_3/(1.0-p3_2)/(1.0-p3_1*(p2+p2_1)+p3_2)/(1.0-p3_1*(p1+p1_1)+p3_2);

	p4 = complex<double>(cos(f), sin(f));
	p5 = complex<double>(cos(f), -sin(f));
	_p4 = 1.0/p4;
	_p5 = 1.0/p5;
	p4_1 = _p4;
	p4_2 = p4_1*_p4;
	p4_3 = p4_2*_p4;
	p5_1 = _p5;
	p5_2 = p5_1*_p5;
	p5_3 = p5_2*_p5;
	
	res1 = (1.0-cos(f)*p1_1)*b0*res1a/(1.0-p4*p1_1)/(1.0-p5*p1_1);
	res2 = (1.0-cos(f)*p2_1)*b0*res2a/(1.0-p4*p2_1)/(1.0-p5*p2_1);
	res3 = (1.0-cos(f)*p3_1)*b0*res3a/(1.0-p4*p3_1)/(1.0-p5*p3_1);
	res4 = (1.0-cos(f)*p4_1)*b0*b0/a3*p4_3/(1.0-p4_2)/(1.0-p4_1*(p1+p1_1)+p4_2)/(1.0-p4_1*(p2+p2_1)+p4_2)/(1.0-p4_1*(p3+p3_1)+p4_2);
	res5 = (1.0-cos(f)*p5_1)*b0*b0/a3*p5_3/(1.0-p5_2)/(1.0-p5_1*(p1+p1_1)+p5_2)/(1.0-p5_1*(p2+p2_1)+p5_2)/(1.0-p5_1*(p3+p3_1)+p5_2);

	resid_cosy[0] = (float)real(res1+res2+res3+res4+res5);
	resid_cosy[1] = (float)real(res1*p1+res2*p2+res3*p3+res4*p4+res5*p5);
	resid_cosy[2] = (float)real(res1*p1*p1+res2*p2*p2+res3*p3*p3+res4*p4*p4+res5*p5*p5);

	res1 = (sin(f)*p1_1)*b0*res1a/(1.0-p4*p1_1)/(1.0-p5*p1_1);
	res2 = (sin(f)*p2_1)*b0*res2a/(1.0-p4*p2_1)/(1.0-p5*p2_1);
	res3 = (sin(f)*p3_1)*b0*res3a/(1.0-p4*p3_1)/(1.0-p5*p3_1);
	res4 = (sin(f)*p4_1)*b0*b0/a3*p4_3/(1.0-p4_2)/(1.0-p4_1*(p1+p1_1)+p4_2)/(1.0-p4_1*(p2+p2_1)+p4_2)/(1.0-p4_1*(p3+p3_1)+p4_2);
	res5 = (sin(f)*p5_1)*b0*b0/a3*p5_3/(1.0-p5_2)/(1.0-p5_1*(p1+p1_1)+p5_2)/(1.0-p5_1*(p2+p2_1)+p5_2)/(1.0-p5_1*(p3+p3_1)+p5_2);

	resid_siny[0] = static_cast<float>(real(res1+res2+res3+res4+res5));
	resid_siny[1] = static_cast<float>(real(res1*p1+res2*p2+res3*p3+res4*p4+res5*p5));
	resid_siny[2] = static_cast<float>(real(res1*p1*p1+res2*p2*p2+res3*p3*p3+res4*p4*p4+res5*p5*p5));
}




void FastGabor::compute_real_horz_forward_ic(double filtermag, double filterphase, double carrierfreq, double carrierphase, float bord_val, float *i0) 
{
		i0[0] = (float)(filtermag*bord_val*cos(carrierfreq*(-1)+filterphase+carrierphase));
		i0[1] = (float)(filtermag*bord_val*cos(carrierfreq*(-2)+filterphase+carrierphase));
		i0[2] = (float)(filtermag*bord_val*cos(carrierfreq*(-3)+filterphase+carrierphase));
}

void FastGabor::compute_imag_horz_forward_ic(double filtermag, double filterphase, double carrierfreq, double carrierphase, float bord_val, float *i0) 
{
		i0[0] = (float)(filtermag*bord_val*sin(carrierfreq*(-1)+filterphase+carrierphase));
		i0[1] = (float)(filtermag*bord_val*sin(carrierfreq*(-2)+filterphase+carrierphase));
		i0[2] = (float)(filtermag*bord_val*sin(carrierfreq*(-3)+filterphase+carrierphase));
}

void FastGabor::compute_real_vert_forward_ic(double filtermag, double filterphase, double carrierfreq, float bord_real_val, float bord_imag_val, float *i0) 
{
		i0[0] = (float)(filtermag*bord_real_val*cos(carrierfreq*(-1)+filterphase)+filtermag*bord_imag_val*sin(carrierfreq*(-1)+filterphase));
		i0[1] = (float)(filtermag*bord_real_val*cos(carrierfreq*(-2)+filterphase)+filtermag*bord_imag_val*sin(carrierfreq*(-2)+filterphase));
		i0[2] = (float)(filtermag*bord_real_val*cos(carrierfreq*(-3)+filterphase)+filtermag*bord_imag_val*sin(carrierfreq*(-3)+filterphase));
}

void FastGabor::compute_imag_vert_forward_ic(double filtermag, double filterphase, double carrierfreq, float bord_real_val, float bord_imag_val, float *i0) 
{
		i0[0] = (float)(filtermag*bord_imag_val*cos(carrierfreq*(-1)+filterphase)-filtermag*bord_real_val*sin(carrierfreq*(-1)+filterphase));
		i0[1] = (float)(filtermag*bord_imag_val*cos(carrierfreq*(-2)+filterphase)-filtermag*bord_real_val*sin(carrierfreq*(-2)+filterphase));
		i0[2] = (float)(filtermag*bord_imag_val*cos(carrierfreq*(-3)+filterphase)-filtermag*bord_real_val*sin(carrierfreq*(-3)+filterphase));
}

void FastGabor::_iir_gaborfilt3_real_horz(float * in, float * even, float *tempbuf, float * real, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx)
{
	
	//initial condition vector - to compute
	float i0[3]; 
	//local vars
	int i,s,bi,bf;
	float bi_val, bf_val;
	double initial_phase;

	s = stridepix;

	for(i = 0; i < height; i++)
	{
		bi = i*s;
		bf = i*s+width-1;
		bi_val = in[bi];
		bf_val = in[bf];

		//causal phase
		//initial contitions at the boundary for forward pass
		initial_phase = -wy*i;
		compute_real_horz_forward_ic(mag, phase, wx, initial_phase, bi_val, i0);

		iir_filt_forward(even+bi, tempbuf, width,  coeffs, i0);

		//anticausal phase
		i0[0] = tempbuf[width-1];
		i0[1] = tempbuf[width-2];
		i0[2] = tempbuf[width-3];
		compute_natural_backward_ic(resid_ic,i0);
		if(wx == 0.0)
			add_step_backward_ic(resid_step,bf_val,i0);
		else
			add_horz_real_backward_ic(resid_cosx, resid_sinx, width-1, bf_val, wy*i, wx, i0);

		iir_filt_backward(tempbuf, real+bi, width,  coeffs, i0);
	}
}

void FastGabor::_iir_gaborfilt3_imag_horz(float * in, float * odd, float *tempbuf, float * imag, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx)
{
	//initial condition vector - to compute
	float i0[3]; 
	//local vars
	int i,s,bi,bf;
	float bi_val, bf_val;
	double initial_phase;

	s = stridepix;

	for(i = 0; i < height; i++)
	{
		bi = i*s;
		bf = i*s+width-1;
		bi_val = in[bi];
		bf_val = in[bf];

		//causal phase
		//initial contitions at the boundary for forward pass
		initial_phase = -wy*i;
		compute_imag_horz_forward_ic(mag, phase, wx, initial_phase, bi_val, i0);

		iir_filt_forward(odd+bi, 1, tempbuf, 1, width,  coeffs, i0);


		//anticausal phase
		i0[0] = tempbuf[width-1];
		i0[1] = tempbuf[width-2];
		i0[2] = tempbuf[width-3];

		compute_natural_backward_ic(resid_ic,i0);
		if(wx == 0.0)
			add_step_backward_ic(resid_step,bf_val,i0);
		else
			add_horz_imag_backward_ic(resid_cosx, resid_sinx, width-1, bf_val, wy*i, wx, i0);

		iir_filt_backward(tempbuf, imag+bi, width,  coeffs, i0);
	}
}


void FastGabor::_iir_gaborfilt3_real_vert(float * real, float * imag, float *tempbuf, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosy, float *resid_siny, float *up_bound, float *low_bound)
{
	//initial condition vector - to compute
	float i0[3]; 
	//local vars
	int j,s, bi, bf;
	float  real_bi_val, imag_bi_val, real_bf_val, imag_bf_val;

	s = stridepix;

	for(j =0 ; j < width; j++)
	{
		bi = j;
		bf = (height-1)*s+j;
		real_bi_val = real[bi];
		imag_bi_val = imag[bi];
		real_bf_val = real[bf];
		imag_bf_val = imag[bf];

		//must save real boundary values to use on the imag part
		up_bound[j] = real[bi];
		low_bound[j] = real[bf];

		//causal phase
		compute_real_vert_forward_ic(mag, phase, wy, real_bi_val, imag_bi_val, i0); 

        iir_filt_forward(real+bi, s, tempbuf, height,  coeffs, i0);
        		
		i0[0] = tempbuf[height-1];
		i0[1] = tempbuf[height-2];
		i0[2] = tempbuf[height-3];
	
		compute_natural_backward_ic(resid_ic,i0);
		if(wy == 0.0)
			add_step_backward_ic(resid_step,real_bf_val,i0);
		else
			add_vert_real_backward_ic(resid_cosy, resid_siny, real_bf_val, imag_bf_val, wy, i0);

		iir_filt_backward(tempbuf, real+bi, s, height,  coeffs, i0);
	}
}

void FastGabor::_iir_gaborfilt3_imag_vert(float * real, float * imag, float *tempbuf, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosy, float *resid_siny, float *up_bound, float *low_bound)
{
	//initial condition vector - to compute
	float i0[3]; 
	//local vars
	int j,s,bi,bf;
	float real_bi_val, imag_bi_val, real_bf_val, imag_bf_val;

	s = stridepix;

	for(j = 0; j < width; j++)
	{
		bi = j;
		bf = (height-1)*s+j;
		real_bi_val = up_bound[j];
		real_bf_val = low_bound[j];
		imag_bi_val = imag[bi];
		imag_bf_val = imag[bf];
		
		//causal phase
		compute_imag_vert_forward_ic(mag, phase, wy, real_bi_val, imag_bi_val, i0); 

		iir_filt_forward(imag+bi, s, tempbuf, height,  coeffs, i0);

		i0[0] = tempbuf[height-1]; 
		i0[1] = tempbuf[height-2];
		i0[2] = tempbuf[height-3];

		compute_natural_backward_ic(resid_ic,i0);
		if(wy == 0.0)
			add_step_backward_ic(resid_step,imag_bf_val,i0);
		else
			add_vert_imag_backward_ic(resid_cosy, resid_siny, real_bf_val, imag_bf_val, wy, i0);

		iir_filt_backward(tempbuf, imag+bi, s, height,  coeffs, i0);

	}
}



void FastGabor::_iir_gaborfilt3(float * in, float * even, float * odd, float * real, float * imag, float *tempbuf, 
		int width, int height, int stridepix, float *coeffs, double wx, double wy, double magx, double magy, double phasex, double phasey, 
		float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx, float *resid_cosy, float *resid_siny,
		float *up_bound, float *low_bound)
{
	//
	// image 'even+j*odd' is the modulation of image 'in' by the complex exponential :
	// exp{j(w*cos(theta)*x) - w*sin(theta)*y)} = exp{j(w*cos(theta)*x)}*exp{-j(w*sin(theta)*y)}
	//

	//The order of the operations is important
	_iir_gaborfilt3_real_horz(in, even, tempbuf, real, width, height, stridepix, coeffs, wx, wy, magx, phasex, resid_ic, resid_step, resid_cosx, resid_sinx);
	_iir_gaborfilt3_imag_horz(in, odd, tempbuf, imag, width, height, stridepix, coeffs, wx, wy, magx, phasex, resid_ic, resid_step, resid_cosx, resid_sinx);

	//
	// na imagem out, a margem superior 'e dada por 
	// ms(x,y) = out(x,Y)*exp(-j*w*sin(theta)*(y-Y))
	// onde Y a indice da localiza��o da borda da imagem
	// Isto vai dar duas componentes - real e imaginaria:
	// ms_real(x,y) = out_real(x,Y)*cos(w*sin(theta)*(y-Y)) + out_imag(x,Y)*sin(w*sin(theta)*(y-Y))
	// ms_imag(x,y) = out_imag(x,Y)*cos(w*sin(theta)*(y-Y)) - out_real(x,Y)*sin(w*sin(theta)*(y-Y))
	//
	
	_iir_gaborfilt3_real_vert(real, imag, tempbuf, width, height, stridepix, coeffs, wx, wy, magy, phasey, resid_ic, resid_step, resid_cosy, resid_siny, up_bound, low_bound);
	_iir_gaborfilt3_imag_vert(real, imag, tempbuf, width, height, stridepix, coeffs, wx, wy, magy, phasey, resid_ic, resid_step, resid_cosy, resid_siny, up_bound, low_bound);

}

