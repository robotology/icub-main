// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file FastGauss.h
 * \brief Implements 3 Tap IIR Gaussian Filtering with Boundary Conditions.
 * \see "Fast IIR Isotropic 2-D Complex Gabor Filters With Boundary Initialization", A. Bernardino and J. Santos-Victor, 2006.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#ifndef _FASTGAUSS_H_
#define _FASTGAUSS_H_

#include <complex>
using namespace std;


class FastGauss
{
private:
    
    bool m_bAllocated;
    float * temp;

protected:
    long m_i_lines;			        //image height (pixels)
	long m_i_cols;			        //image width (pixels)
	long m_i_stridepix;		        //number of pixels in a line (for data aligment)
    int stride;				        //number of bytes in a line (for data alignment) : stride = m_i_stridepix*sizeof(float)
	double m_scale;                 //scale value (in pixels)
    float *filt_coeffs;			    //filter coefficients (max 6 coeffs : 1 gain + 5 autoregressive coefs)
	complex<double> *filt_poles;    //filter poles (max 5 poles)
	float *m_resid_step;            //step forced response residuals for a 3 tap gauss filter.
	float *m_resid_ic;              //step natural response residuals for a 3 tap gauss filter.
    
public:
    FastGauss(void);
	virtual ~FastGauss(void);
    long GetLines();
	long GetCols();
	long GetStridePix();
    double GetScale();
    bool GaussFilt(float * in, float * out);
	bool AllocateResources(long lines, long cols, double scale);
	bool FreeResources();
	bool IsAllocated();
	
    void compute_step_forward_ic(float bord_val, float *coeffs, float *i0);
    void compute_natural_backward_ic( float *resid_ic, float *i0 );
    void add_step_backward_ic( float *resid_step, float val, float *i0 );
    void _iir_gaussfilt3_horz(float * in, float *tempbuf, float * out, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step);
    void _iir_gaussfilt3_vert(float * inout, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step);
    void _iir_gaussfilt3(float * in, float * out, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step );
    void _compute_gauss3_resids( complex<double> poles[], float *coeffs, float *resid_ic, float *resid_step );
};

#endif /*_FASTGAUSS_H_*/
