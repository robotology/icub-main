// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file FastGabor.h
 * \brief Implements Gabor Filters by Gaussian Filtering exponentially modulated images.
 * \see "Fast IIR Isotropic 2-D Complex Gabor Filters With Boundary Initialization", A. Bernardino and J. Santos-Victor, 2006.
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#ifndef _FASTGABOR_H_
#define _FASTGABOR_H_

#include <complex>
using namespace std;

#include "iCub/FastGauss.h"


class FastGabor : public FastGauss
{
private:
    double pi;				//the irracional number 
    double m_wavelength;    //wavelength value (in pixels)
	double m_orientation;   //orientation value (in radians)
	double m_dc_gain;				//offset for zero mean gabor filters.
	double m_magnitudex;	    //horizontal magnitude of the freq. resp.
	double m_phasex;		    //phase of the horizontal filter freq. resp.
	double m_magnitudey;     //magnitude of the vertical filter freq. resp.
	double m_phasey;		    //phase of the vertical filter freq. resp.
	double m_wx;					//horizontal frequency of the filter.
	double m_wy;					//vertical frequency of the filter.
	float *m_resid_cosx;	    //horizontal cosine forced response residuals for a 3 tap gauss filter.
	float *m_resid_sinx;	    //horizontal sine forced response residuals for a 3 tap gauss filter.
	float *m_resid_cosy;	    //vertical cosine forced response residuals for a 3 tap gauss filter.
	float *m_resid_siny;	    //vertical sine forced response residuals for a 3 tap gauss filter.
	
    float * temp;					//temporary image to store intermediate computation steps
    float * temp2;					//temporary image to store intermediate computation steps
	float * even;					//temporary image to store intermediate computation steps
	float * odd;					//temporary image to store intermediate computation steps
    float * sine;				    //sine image for modulation
	float * cosine;			    //cosine image for modulation
	float *upper_boundary;			//temporary image line for boundary computations
	float *lower_boundary;			//temporary image line for boundary computations
	bool m_bAllocated;				//boolean variable indicating if the object has been allocated
public:
	//long GetLines();              // Inherited from FastGauss
	//long GetCols();               // Inherited from FastGauss
	//long GetStridePix();          // Inherited from FastGauss
	//double GetScale();              // Inherited from FastGauss

	double GetWavelength();
	double GetOrientation();
    double GetDcGain();
	
	FastGabor(void);
	virtual ~FastGabor(void);
    bool AllocateResources(long lines, long cols, double scale, double orient, double wav );
	bool FreeResources();
	bool IsAllocated();

	bool GaborFilt(float * in, float * real, float * imag);
    bool RemoveDC(float * real, float * gauss, float * zeromeanreal);
    bool ComputeMagnitude( float * real, float * imag, float * magn );
    bool GaborFiltZeroMean(float * in, float * real, float * imag);
	
    //auxiliary functions
    void _image_prod(float * i1, float * i2, float * out);
    void _const_prod(float * in, float * out, float gain);
    void _image_add(float * i1, float * i2, float * out);
    void _image_sub(float * i1, float * i2, float * out);
    void _image_sqrt(float * in, float * out);

    double calc_dcvalue_1D(float *coeffs, double w);
    double calc_dcvalue(float *coeffs, double wx, double wy);
    void calc_frequency_response(float *filter_coeffs, double freq, double *magnitude, double *phase);
    void calc_real_horizontal_backward_initial_conditions(float *filter_coeffs, double *filter_poles, int bord_loc, float bord_val, double phase0, double freq, float *init_cond );
    void calc_imag_horizontal_backward_initial_conditions(float *filter_coeffs, double *filter_poles, int bord_loc, float bord_val, double phase0, double freq, float *init_cond );
    void calc_real_vertical_backward_initial_conditions(float *filter_coeffs,double *filter_poles,float real_bord,float imag_bord,double freq,float *init_cond );
    void calc_imag_vertical_backward_initial_conditions(float *filter_coeffs,double *filter_poles,float real_bord,float imag_bord,double freq,float *init_cond );

    //void _iir_gaussfilt3_horz(float * in, float *tempbuf, float * out, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step);
    //void _iir_gaussfilt3_vert(float * inout, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step);
    //void _iir_gaussfilt3(float * in, float * out, float *tempbuf, int width, int height, int stridepix, float *coeffs, float *resid_ic, float *resid_step );
    void _iir_gaborfilt3(float * in, float * even, float * odd, float * real, float * imag, float *tempbuf, 
		                    int width, int height, int stridepix, float *coeffs, double wx, double wy, 
						    double magx, double magy, double phasex, double phasey, 
		                    float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx, float *resid_cosy, float *resid_siny,
		                    float *up_bound, float *low_bound);

    void _iir_gaborfilt3_real_horz(float * in, float * even, float *tempbuf, float * real, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx);
    void _iir_gaborfilt3_imag_horz(float * in, float * odd, float *tempbuf, float * imag, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosx, float *resid_sinx);
    void _iir_gaborfilt3_real_vert(float * real, float * imag, float *tempbuf, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosy, float *resid_siny, float *up_bound, float *low_bound);
    void _iir_gaborfilt3_imag_vert(float * real, float * imag, float *tempbuf, int width, int height, int stridepix, float *coeffs, double wx, double wy, double mag, double phase, float *resid_ic, float *resid_step, float *resid_cosy, float *resid_siny, float *up_bound, float *low_bound);

    void _compute_gabor3_horz_resids(complex<double> poles[], float *coeffs, double f, float *resid_cosx, float *resid_sinx);
    void _compute_gabor3_vert_resids(complex<double> poles[], float *coeffs, double f, float *resid_cosy, float *resid_siny);

    void compute_real_horz_forward_ic(double filtermag, double filterphase, double carrierfreq, double carrierphase, float bord_val, float *i0) ;
    void compute_imag_horz_forward_ic(double filtermag, double filterphase, double carrierfreq, double carrierphase, float bord_val, float *i0);
    void compute_real_vert_forward_ic(double filtermag, double filterphase, double carrierfreq, float bord_real_val, float bord_imag_val, float *i0); 
    void compute_imag_vert_forward_ic(double filtermag, double filterphase, double carrierfreq, float bord_real_val, float bord_imag_val, float *i0);


    void add_horz_real_backward_ic(float *resid_cosx, float *resid_sinx, int bord_loc, float bord_val, double phase0, double freq, float *init_cond );
    void add_horz_imag_backward_ic(float *resid_cosx, float *resid_sinx, int bord_loc, float bord_val, double phase0, double freq, float *init_cond );
    void add_vert_real_backward_ic(float *resid_cosy, float *resid_siny, float real_bord, float imag_bord, double freq, float *init_cond );
    void add_vert_imag_backward_ic(float *resid_cosy, float *resid_siny, float real_bord, float imag_bord, double freq, float *init_cond );
};



#endif /*_FASTGABOR_H_*/
