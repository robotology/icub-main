// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file FastGauss.cpp
 * \brief Implements 3 Tap IIR Gaussian Filtering with Boundary Conditions.
 * \see "Fast IIR Isotropic 2-D Complex Gabor Filters With Boundary Initialization", A. Bernardino and J. Santos-Victor, 2006.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#include <iCub/vis/FastGauss.h>
#include <iCub/vis/IIRGausDeriv.h>
#include <iCub/vis/IIRFilt.h>

#include <memory.h>

FastGauss::FastGauss(void)
{
	//Initialization of constants and variables
	m_i_lines = 0;
	m_i_cols = 0;
	stride = 0;
	m_i_stridepix = 0;
	temp = NULL;
	m_scale = 0;
	filt_poles = NULL;
	filt_coeffs = NULL;
	m_bAllocated = false;
	m_resid_step = NULL;
	m_resid_ic = NULL;
}

FastGauss::~FastGauss(void)
{
	FreeResources();
}

bool FastGauss::IsAllocated()
{
	return m_bAllocated;
}
long FastGauss::GetLines()
{
	return m_i_lines;
}

long FastGauss::GetCols()
{
	return m_i_cols;
}


long FastGauss::GetStridePix()
{
	return m_i_stridepix;
}

double FastGauss::GetScale()
{
	return m_scale;
}

bool FastGauss::AllocateResources(long lines, long cols, double scale)
{
	if(IsAllocated())
		FreeResources();

	m_i_lines = lines;
	m_i_cols = cols;
	
    if(scale < 0.5)
	{
		throw "Scale values lower than 0.5 are not alowed";
	}
	if(scale > 100)
	{
		throw "Danger: Bad Approximation for scale values higher than 100";
	}
	m_scale = scale;
	
	//allocation of level structures -  filter poles, coefficients, residues
	filt_poles = new complex<double>[5]; //max 5 complex poles 
	//filt_coeffs = (float*)malloc(6*sizeof(float)); //max 6 coeffs : 1 gain + 5 autoregressive terms
	filt_coeffs = new float[6]; //max 6 coeffs : 1 gain + 5 autoregressive terms
	//m_resid_step = (float*)malloc(3*sizeof(float));
	m_resid_step = new float[3];
	//m_resid_ic = (float*)malloc(9*sizeof(float));
	m_resid_ic = new float[9];
	
	//allocation of image buffers for computations
	
	m_i_stridepix = m_i_cols;
	stride = m_i_cols*4;
	//temp = (float*)malloc(m_i_lines*m_i_cols*sizeof(float));
	temp = new float[m_i_lines*m_i_cols];

	  //compute filter poles, coefficients and boundary gaussian residues for each scale
	calc_poles(3,m_scale,d0_N3_Linf, filt_poles);
	calc_coeffs(3, filt_poles, filt_coeffs);
	_compute_gauss3_resids(filt_poles, filt_coeffs, m_resid_ic, m_resid_step);
	
	m_bAllocated = true;
	return true;
}

bool FastGauss::FreeResources()
{
	m_bAllocated = false;
	if(m_resid_step != NULL)
	{
		//free(m_resid_step);
                delete[] m_resid_step;
		m_resid_step = NULL;
	}

	if(m_resid_ic!= NULL)
	{
		//free(m_resid_ic);
		delete[] m_resid_ic;
		m_resid_ic = NULL;
	}

	if(filt_coeffs != NULL)
	{
		//free(filt_coeffs);
		delete[] filt_coeffs;
		filt_coeffs = NULL;
	}
	if(filt_poles != NULL)
	{
		delete[] filt_poles;
		filt_poles = NULL;
	}

	if(temp != NULL) 
		//free(temp);	
		delete[] temp;	

	temp = NULL;
	return true;
}

void FastGauss::_iir_gaussfilt3_horz(float * in, float * tempbuf, float * out, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step)
{
	
	float i0[3]; //initial condition vector - to compute
	int i,s,bi,bf;
	float bi_val, bf_val;
	s = stridepix;
	//filtering rows
	for(i = 0; i < height; i++)
	{
		bi = i*s;
		bf = i*s+width-1;
		bi_val = in[bi];
		bf_val = in[bf];
		//causal phase
		//compute forward initial conditions (response to a step of bi_val amplitude)
		compute_step_forward_ic(bi_val, coeffs, i0);
		
		/*mag = b0/(1+a1+a2+a3);
		i0[0] = i0[1] = i0[2] = (float)(mag*bi_val);*/

		iir_filt_forward(in+bi,tempbuf,width,coeffs,i0);
		//anticausal phase
		i0[0] = tempbuf[width-1];
		i0[1] = tempbuf[width-2];
		i0[2] = tempbuf[width-3];
		
		compute_natural_backward_ic(resid_ic,i0);
		add_step_backward_ic(resid_step,bf_val,i0);

		iir_filt_backward(tempbuf,out+bi,width,coeffs,i0);
	}
}

void FastGauss::_iir_gaussfilt3_vert(float * inout, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step)
{

	//filter coefficients
	float b0 = coeffs[0];
	float a1 = coeffs[1];
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	float i0[3]; //initial condition vector - to compute
	int j,s,bi,bf;
	float bi_val, bf_val;
	s = stridepix;

	//filtering columns	
	for(j = 0; j < width; j++)
	{
		bi = j;
		bf = (height-1)*s+j;
		bi_val = inout[bi];
		bf_val = inout[bf];
		//causal phase
		//compute forward initial conditions (response to a step of bi_val amplitude)
		compute_step_forward_ic(bi_val, coeffs, i0);

		/*mag = b0/(1+a1+a2+a3);
		i0[0] = i0[1] = i0[2] = (float)(mag*bi_val);*/

		iir_filt_forward(inout+bi,s,tempbuf,height,coeffs,i0);

		i0[0] = tempbuf[height-1];
		i0[1] = tempbuf[height-2];
		i0[2] = tempbuf[height-3];

		compute_natural_backward_ic(resid_ic,i0);
		add_step_backward_ic(resid_step,bf_val,i0);

		iir_filt_backward(tempbuf,inout+bi,s,height,coeffs,i0);
	}
}

void FastGauss::_iir_gaussfilt3(float * in, float * out, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step )
{
	_iir_gaussfilt3_horz(in, tempbuf, out, width, height, stridepix, coeffs, resid_ic, resid_step);
	_iir_gaussfilt3_vert(out, tempbuf, width, height, stridepix, coeffs, resid_ic, resid_step);
}

bool FastGauss::GaussFilt(float * in, float * out)
{	
	if(!IsAllocated())
		throw "Resources not allocated";

	int i;

	//copy image to internal buffer
	for( i = 0; i < m_i_lines; i++ )
		memcpy(in + i*m_i_stridepix, in + m_i_cols*i, m_i_cols*sizeof(float) );

	//computing the gaussian filtered image
	_iir_gaussfilt3(in, out, temp, m_i_cols, m_i_lines, m_i_stridepix, filt_coeffs, m_resid_ic, m_resid_step );

	return true;
}

void FastGauss::compute_step_forward_ic(float bord_val, float *coeffs, float *i0)
{
	//filter coefficients
	float b0 = coeffs[0];
	float a1 = coeffs[1];
	float a2 = coeffs[2];
	float a3 = coeffs[3];

	i0[0] = i0[1] = i0[2] = bord_val*b0/(1+a1+a2+a3);
}

void FastGauss::compute_natural_backward_ic( float *resid_ic, float *init_cond )
{
	float i0 = init_cond[0];
	float i1 = init_cond[1];
	float i2 = init_cond[2];

	init_cond[0]=i0*resid_ic[0]+i1*resid_ic[1]+i2*resid_ic[2];
	init_cond[1]=i0*resid_ic[3]+i1*resid_ic[4]+i2*resid_ic[5];
	init_cond[2]=i0*resid_ic[6]+i1*resid_ic[7]+i2*resid_ic[8];
}

void FastGauss::add_step_backward_ic( float *resid_step, float val, float *init_cond )
{
	init_cond[0] += val*resid_step[0];
	init_cond[1] += val*resid_step[1];
	init_cond[2] += val*resid_step[2];
}

void FastGauss::_compute_gauss3_resids( complex<double> filt_poles[], float *filt_coeffs, float *resid_ic, float *resid_step )
{

	complex <double> p1, p2, p3, _p1, _p2, _p3;
	complex <double> p1_1, p1_2, p1_3, p2_1, p2_2, p2_3, p3_1, p3_2, p3_3;
	double b0, a1, a2, a3;

	_p1 = filt_poles[0];
	_p2 = filt_poles[1];
	_p3 = filt_poles[2];
	b0 = filt_coeffs[0];
	a1 = filt_coeffs[1];
	a2 = filt_coeffs[2];
	a3 = filt_coeffs[3];


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

	complex <double> res1a, res2a, res3a, res1b, res2b, res3b, res1c, res2c, res3c;
	//a3 = -p1*p2*p3
	res1a = b0/a3*p1_3/(1.0-p1_2)/(1.0-p1_1*(p2+p2_1)+p1_2)/(1.0-p1_1*(p3+p3_1)+p1_2);
	res2a = b0/a3*p2_3/(1.0-p2_2)/(1.0-p2_1*(p1+p1_1)+p2_2)/(1.0-p2_1*(p3+p3_1)+p2_2);
	res3a = b0/a3*p3_3/(1.0-p3_2)/(1.0-p3_1*(p2+p2_1)+p3_2)/(1.0-p3_1*(p1+p1_1)+p3_2);
	res1b = p1_1*res1a;
	res2b = p2_1*res2a;
	res3b = p3_1*res3a;
	res1c = p1_2*res1a;
	res2c = p2_2*res2a;
	res3c = p3_2*res3a;

	//initial condition residues
	complex<double> r0, r1, r2, r3, r4, r5, r6, r7, r8;

	r0 = (-a1*res1a-a2*res1b-a3*res1c);
	r1 = (-a2*res1a-a3*res1b);
	r2 = (-a3*res1a);
	r3 = (-a1*res2a-a2*res2b-a3*res2c);
	r4 = (-a2*res2a-a3*res2b);
	r5 = (-a3*res2a);
	r6 = (-a1*res3a-a2*res3b-a3*res3c);
	r7 = (-a2*res3a-a3*res3b);
	r8 = (-a3*res3a);

	resid_ic[0] = (float)real(r0+r3+r6);
	resid_ic[1] = (float)real(r1+r4+r7);
	resid_ic[2] = (float)real(r2+r5+r8);
	resid_ic[3] = (float)real(r0*p1+r3*p2+r6*p3);
	resid_ic[4] = (float)real(r1*p1+r4*p2+r7*p3);
	resid_ic[5] = (float)real(r2*p1+r5*p2+r8*p3);
	resid_ic[6] = (float)real(r0*p1*p1+r3*p2*p2+r6*p3*p3);
	resid_ic[7] = (float)real(r1*p1*p1+r4*p2*p2+r7*p3*p3);
	resid_ic[8] = (float)real(r2*p1*p1+r5*p2*p2+r8*p3*p3);

	//step residues
	complex <double> res1, res2, res3, res4;

	res1 = b0*res1a/(1.0-_p1);
	res2 = b0*res2a/(1.0-_p2);
	res3 = b0*res3a/(1.0-_p3);
	res4 = b0*b0/(1+a1+a2+a3)/(1+a1+a2+a3);
	resid_step[0] = (float)real(res1+res2+res3+res4);
	resid_step[1] = (float)real(res1*p1+res2*p2+res3*p3+res4);
	resid_step[2] = (float)real(res1*p1*p1+res2*p2*p2+res3*p3*p3+res4);
}


