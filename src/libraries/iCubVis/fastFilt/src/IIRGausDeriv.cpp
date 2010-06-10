// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/**
 * \file IIRGausDeriv.cpp
 * \brief Coefficients and poles of IIR Gaussian Derivative Filters (order 0, 1 and 2).
 * \see "Recursive Gaussian Derivative Filters", L.van.Vliet,I.T.Young and P.W.Verbeek, 1998
 * \author Alex Bernardino, ISR-IST
 * \date 2006-2007
 * \note Release under GNU GPL v2.0
 * 
 */


#include <complex>
using namespace std;

#include <iCub/vis/IIRFilt.h>

//
//Values of the pole locations for the iir recursive gaussian derivative filters of scale s = 2.
//From "Recursive Gaussian Derivative Filters", L.van.Vliet,Ian.T.Young and Piet.W.Verbeek
//

// d - differentiation order
// N - number of poles
// L - approximation norm

static const complex<double> d0_N3_L2[] = {
	complex<double>(1.41650,1.00829),
	complex<double>(1.41650,-1.00829),
	complex<double>(1.86543,0),
};

static const complex<double> d0_N4_L2[] = {
	complex<double>(1.13228,1.28114),
	complex<double>(1.13228,-1.28114),
	complex<double>(1.78534,0.46763),
	complex<double>(1.78534,-0.46763),
};

static const complex<double> d0_N5_L2[] = {
	complex<double>(0.86430,1.45389),
	complex<double>(0.86430,-1.45389),
	complex<double>(1.61433,0.83134),
	complex<double>(1.61433,-0.83134),
	complex<double>(1.87504,0),
};

extern const complex<double> d0_N3_Linf[] = {
	complex<double>(1.40098,1.00236),
	complex<double>(1.40098,-1.00236),
	complex<double>(1.85132,0),
};

static const complex<double> d0_N4_Linf[] = {
	complex<double>(1.12075,1.27788),
	complex<double>(1.12075,-1.27788),
	complex<double>(1.76952,0.46611),
	complex<double>(1.76952,-0.46611),
};

static const complex<double> d0_N5_Linf[] = {
	complex<double>(0.85480,1.43749),
	complex<double>(0.85480,-1.43749),
	complex<double>(1.61231,0.82053),
	complex<double>(1.61231,-0.82053),
	complex<double>(1.87415,0)
};

static const complex<double> d1_N3_Linf[] = {
	complex<double>(1.31553,0.97057),
	complex<double>(1.31553,-0.97057),
	complex<double>(1.77635,0),
};

static const complex<double> d1_N4_Linf[] = {
	complex<double>(1.04185,1.24034),
	complex<double>(1.04185,-1.24034),
	complex<double>(1.69747,0.44790),
	complex<double>(1.69747,-0.44790),

};

static const complex<double> d1_N5_Linf[] = {
	complex<double>(0.77934,1.41423),
	complex<double>(0.77934,-1.41423),
	complex<double>(1.50941,0.80828),
	complex<double>(1.50941,-0.80828),
	complex<double>(1.77181,0)
};


static const complex<double> d3_N3_Linf[] = {
	complex<double>(1.22886,0.93058),
	complex<double>(1.22886,-0.93058),
	complex<double>(1.70493,0),
};

static const complex<double> d2_N4_Linf[] = {
	complex<double>(0.94570,1.21064),
	complex<double>(0.94570,-1.21064),
	complex<double>(1.60161,0.42647),
	complex<double>(1.60161,-0.42647),
};

static const complex<double> d2_N5_Linf[] = {
	complex<double>(0.69843,1.37655),
	complex<double>(0.69843,-1.37655),
	complex<double>(1.42631,0.77399),
	complex<double>(1.42631,-0.77399),
	complex<double>(1.69668,0)
};



//compute the poles of the filter for scale s given the poles at scale 2
void calc_poles(int taps, const double scale, const complex<double> oldpoles[], complex<double> newpoles[])
{
	if((taps < 3)||(taps>5))
		throw "Invalid number of taps in calc_poles";
	
	if(newpoles == NULL)
		throw "NULL Pointer argument in calc_poles";

	complex <double> d1_2, d2_2, d3_2, d4_2, d5_2;
	d1_2 = oldpoles[0];
	d2_2 = oldpoles[1];
	d3_2 = oldpoles[2];
	if(taps > 3)
		d4_2 = oldpoles[3];
	else
		d4_2 = 0;
	if(taps > 4)
		d5_2 = oldpoles[4];
	else
		d5_2 = 0;
	
	double q, std, lambda;
	double tol = 0.01;
	complex <double> j(0,1), var;
	complex <double> d1_s, d2_s, d3_s, d4_s, d5_s;
	// computing new values for the poles
	q = scale/2;
	d1_s = exp(log(abs(d1_2))/q)*exp(j*arg(d1_2)/q);
	d2_s = exp(log(abs(d2_2))/q)*exp(j*arg(d2_2)/q);
	d3_s = exp(log(abs(d3_2))/q)*exp(j*arg(d3_2)/q);
	if( abs(d4_2) != 0 )
		d4_s = exp(log(abs(d4_2))/q)*exp(j*arg(d4_2)/q);
	else
		d4_s = 0;
	if( abs(d5_2) != 0 )
		d5_s = exp(log(abs(d5_2))/q)*exp(j*arg(d5_2)/q);
	else
		d5_s = 0;
	// computing the variance of the new filter
	var =  d1_s*2.0/(d1_s-1.0)/(d1_s-1.0) + d2_s*2.0/(d2_s-1.0)/(d2_s-1.0) + d3_s*2.0/(d3_s-1.0)/(d3_s-1.0)+d4_s*2.0/(d4_s-1.0)/(d4_s-1.0)+d5_s*2.0/(d5_s-1.0)/(d5_s-1.0);
	std = sqrt(var.real());
	while( fabs(scale-std) > tol )
	{
		lambda = scale/std;
		q = q*lambda;
		// computing new values for the poles
		d1_s = exp(log(abs(d1_2))/q)*exp(j*arg(d1_2)/q);
		d2_s = exp(log(abs(d2_2))/q)*exp(j*arg(d2_2)/q);
		d3_s = exp(log(abs(d3_2))/q)*exp(j*arg(d3_2)/q);
		if( abs(d4_2) != 0)
			d4_s = exp(log(abs(d4_2))/q)*exp(j*arg(d4_2)/q);
		else
			d4_s = 0;
		if( abs(d5_2) != 0)
			d5_s = exp(log(abs(d5_2))/q)*exp(j*arg(d5_2)/q);
		else
			d5_s = 0;
		// computing the variance of the new filter
		var =  d1_s*2.0/(d1_s-1.0)/(d1_s-1.0) + d2_s*2.0/(d2_s-1.0)/(d2_s-1.0) + d3_s*2.0/(d3_s-1.0)/(d3_s-1.0)+d4_s*2.0/(d4_s-1.0)/(d4_s-1.0)+d5_s*2.0/(d5_s-1.0)/(d5_s-1.0);
		std = sqrt(var.real());
	}
	newpoles[0] = d1_s;
	newpoles[1] = d2_s;
	newpoles[2] = d3_s;
	newpoles[3] = d4_s;
	newpoles[4] = d5_s;
}

//compute the coefficients of the filter, given its poles
//the output is written in array coeffs - the first element is the gain and 
//the remaining elements correspond to the autoregressive coefficients 
void calc_coeffs(int taps, const complex<double> poles[], float *coeffs)
{
	if((taps < 3)||(taps>5))
		throw "Invalid number of taps in calc_coeffs";
	
	if(coeffs == NULL)
		throw "NULL Pointer argument in calc_coeffs";

	complex <double> d1_s, d2_s, d3_s, d4_s, d5_s;
	d1_s = poles[0];
	d2_s = poles[1];
	d3_s = poles[2];
	if(taps > 3)
		d4_s = poles[3];
	else
		d4_s = 0;
	if(taps > 4)
		d5_s = poles[4];
	else
		d5_s = 0;
	
	//computing the filter coeffs
	if( taps == 3 )
	{
		complex<double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s;
		coeffs[1] = (float)real(-b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s));
		coeffs[2] = (float)real(b*(d1_s + d2_s + d3_s));
		coeffs[3] = (float)real(-b);
		coeffs[4] = 0.0f;
		coeffs[5] = 0.0f;
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3];
	}
	else if(taps == 4)
	{
		complex<double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s/d4_s;
		coeffs[1] = (float)real(-b*(d3_s*d2_s*d1_s + d4_s*d2_s*d1_s + d4_s*d3_s*d1_s + d4_s*d3_s*d2_s));
		coeffs[2] = (float)real(b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s + d4_s*d1_s + d4_s*d2_s + d4_s*d3_s));
		coeffs[3] = (float)real(-b*(d1_s + d2_s + d3_s + d4_s));
		coeffs[4] = (float)real(b);
		coeffs[5] = 0.0f;
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3] + coeffs[4];
	}
	else if(taps == 5)
	{
		complex <double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s/d4_s/d5_s;
		coeffs[1] = (float)real(-b*(d4_s*d3_s*d2_s*d1_s + d5_s*d3_s*d2_s*d1_s + d5_s*d4_s*d2_s*d1_s + d5_s*d4_s*d3_s*d1_s + d5_s*d4_s*d3_s*d2_s));
		coeffs[2] = (float)real(b*(d3_s*d2_s*d1_s + d4_s*d2_s*d1_s + d4_s*d3_s*d1_s + d4_s*d3_s*d2_s + d5_s*d2_s*d1_s + d5_s*d3_s*d1_s + d5_s*d3_s*d2_s + d5_s*d4_s*d1_s + d5_s*d4_s*d2_s + d5_s*d4_s*d3_s));
		coeffs[3] = (float)real(-b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s + d4_s*d1_s + d4_s*d2_s + d4_s*d3_s + d5_s*d1_s + d5_s*d2_s + d5_s*d3_s + d5_s*d4_s));
		coeffs[4] = (float)real(b*(d1_s + d2_s + d3_s + d4_s + d5_s));
		coeffs[5] = (float)real(-b);
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3] + coeffs[4] + coeffs[5];    
	}
}



//compute the coefficients of the filter for scale s given the poles at scale 2
//the output is written in array coeffs - the first element is the gain and 
//the remaining elements correspond to the autoregressive coefficients 
void calc_coeffs(int taps, const complex<double> poles[], const double s, float *coeffs)
{

	if((taps < 3)||(taps>5))
		throw "Invalid number of taps in calc_coeffs";
	
	if(coeffs == NULL)
		throw "NULL Pointer argument in calc_coeffs";

	complex <double> d1_2, d2_2, d3_2, d4_2, d5_2;
	d1_2 = poles[0];
	d2_2 = poles[1];
	d3_2 = poles[2];
	if(taps > 3)
		d4_2 = poles[3];
	else
		d4_2 = 0;
	if(taps > 4)
		d5_2 = poles[4];
	else
		d5_2 = 0;
	
	double q, std, lambda;
	double tol = 0.01;
	complex <double> j(0,1), var;
	complex <double> d1_s, d2_s, d3_s, d4_s, d5_s;
	// computing new values for the poles
	q = s/2;
	d1_s = exp(log(abs(d1_2))/q)*exp(j*arg(d1_2)/q);
	d2_s = exp(log(abs(d2_2))/q)*exp(j*arg(d2_2)/q);
	d3_s = exp(log(abs(d3_2))/q)*exp(j*arg(d3_2)/q);
	if( abs(d4_2) != 0 )
		d4_s = exp(log(abs(d4_2))/q)*exp(j*arg(d4_2)/q);
	else
		d4_s = 0;
	if( abs(d5_2) != 0 )
		d5_s = exp(log(abs(d5_2))/q)*exp(j*arg(d5_2)/q);
	else
		d5_s = 0;
	// computing the variance of the new filter
	var =  d1_s*2.0/(d1_s-1.0)/(d1_s-1.0) + d2_s*2.0/(d2_s-1.0)/(d2_s-1.0) + d3_s*2.0/(d3_s-1.0)/(d3_s-1.0)+d4_s*2.0/(d4_s-1.0)/(d4_s-1.0)+d5_s*2.0/(d5_s-1.0)/(d5_s-1.0);
	std = sqrt(var.real());
	while( fabs(s-std) > tol )
	{
		lambda = s/std;
		q = q*lambda;
		// computing new values for the poles
		d1_s = exp(log(abs(d1_2))/q)*exp(j*arg(d1_2)/q);
		d2_s = exp(log(abs(d2_2))/q)*exp(j*arg(d2_2)/q);
		d3_s = exp(log(abs(d3_2))/q)*exp(j*arg(d3_2)/q);
		if( abs(d4_2) != 0)
			d4_s = exp(log(abs(d4_2))/q)*exp(j*arg(d4_2)/q);
		else
			d4_s = 0;
		if( abs(d5_2) != 0)
			d5_s = exp(log(abs(d5_2))/q)*exp(j*arg(d5_2)/q);
		else
			d5_s = 0;
		// computing the variance of the new filter
		var =  d1_s*2.0/(d1_s-1.0)/(d1_s-1.0) + d2_s*2.0/(d2_s-1.0)/(d2_s-1.0) + d3_s*2.0/(d3_s-1.0)/(d3_s-1.0)+d4_s*2.0/(d4_s-1.0)/(d4_s-1.0)+d5_s*2.0/(d5_s-1.0)/(d5_s-1.0);
		std = sqrt(var.real());
	}

	//computing the filter coeffs
	if( taps == 3 )
	{
		complex<double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s;
		coeffs[1] = (float)real(-b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s));
		coeffs[2] = (float)real(b*(d1_s + d2_s + d3_s));
		coeffs[3] = (float)real(-b);
		coeffs[4] = 0.0f;
		coeffs[5] = 0.0f;
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3];
	}
	else if(taps == 4)
	{
		complex<double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s/d4_s;
		coeffs[1] = (float)real(-b*(d3_s*d2_s*d1_s + d4_s*d2_s*d1_s + d4_s*d3_s*d1_s + d4_s*d3_s*d2_s));
		coeffs[2] = (float)real(b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s + d4_s*d1_s + d4_s*d2_s + d4_s*d3_s));
		coeffs[3] = (float)real(-b*(d1_s + d2_s + d3_s + d4_s));
		coeffs[4] = (float)real(b);
		coeffs[5] = 0.0f;
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3] + coeffs[4];
	}
	else if(taps == 5)
	{
		complex <double> b = complex<double>(1.0,0.0)/d1_s/d2_s/d3_s/d4_s/d5_s;
		coeffs[1] = (float)real(-b*(d4_s*d3_s*d2_s*d1_s + d5_s*d3_s*d2_s*d1_s + d5_s*d4_s*d2_s*d1_s + d5_s*d4_s*d3_s*d1_s + d5_s*d4_s*d3_s*d2_s));
		coeffs[2] = (float)real(b*(d3_s*d2_s*d1_s + d4_s*d2_s*d1_s + d4_s*d3_s*d1_s + d4_s*d3_s*d2_s + d5_s*d2_s*d1_s + d5_s*d3_s*d1_s + d5_s*d3_s*d2_s + d5_s*d4_s*d1_s + d5_s*d4_s*d2_s + d5_s*d4_s*d3_s));
		coeffs[3] = (float)real(-b*(d2_s*d1_s + d3_s*d1_s + d3_s*d2_s + d4_s*d1_s + d4_s*d2_s + d4_s*d3_s + d5_s*d1_s + d5_s*d2_s + d5_s*d3_s + d5_s*d4_s));
		coeffs[4] = (float)real(b*(d1_s + d2_s + d3_s + d4_s + d5_s));
		coeffs[5] = (float)real(-b);
		coeffs[0] = 1.0f + coeffs[1] + coeffs[2] + coeffs[3] + coeffs[4] + coeffs[5];    
	}
}





