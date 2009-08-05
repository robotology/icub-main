// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

#ifndef __CLOGPOLARPARAMS_H_
#define __CLOGPOLARPARAMS_H_

///		Class for the definition of the logpolar map: parameters and geometric functions
///
///		The logpolarmap is defined as:
///	eccentr = log((radius+m_fLogShift)/(m_fBlindRadius+m_fLogShift)) / log(m_fLogFact)
///
///		The parameter logshift is introduced to allow to set blind radius to 0, so that the whole fovea is mapped.
/// This parameter can vary in the interval ]-m_fBlindRadius, infinity[, and the second eccentricity (m_SecondRadius) 
/// will vary in the interval ]m_fBlindRadius, 1/m_iEccentr * max_radius + (m_iEccentr+1)/m_iEccentr * m_fBlindRadius[
/// To avoid excessive distortion from the pure logpolar mapping it is recommended to use values between 0 and 1.
///


/// TODO: CLogPolarSensor - Define the type of parameter computation (enum); Define the type of image data (enum)
/// TODO: Funções de cálculo de parametros podem ser estáticas


#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// Sugestão do Julio
#define pi (3.14159265358979323846f)

class CLogPolarParams
{
    
	// Sugestão do Julio
	//static double pi;
protected:
	double m_fBlindRadius;
	double m_fLogFact;
	double m_fLogShift;
	double m_fSecondRadius;
	int m_iAngles;
	int m_iEccentr;
	bool logpolar_has_changed;
public:
	CLogPolarParams()
		: m_fBlindRadius(10.0f)
		, m_fLogFact(0.0f)
		, m_fLogShift(0.0f)
		, m_fSecondRadius(0.0f)
		, m_iAngles(64)
		, m_iEccentr(32)
		, logpolar_has_changed(false)
	{
		
	}

	virtual ~CLogPolarParams()
	{
	}
    
    /*
	FUNCTION: 		compute_params_fixed_design
	DESCRIPTION:    Compute the logpolar parameters log_fact and second_radius given the current
					the other parameter and the size of the image plane.
					The parameters are computed such that the limit of the image plane coincides with 
					the outer cells of the logpolar space.
	NOTE:           If blind_radius == 0, log_shift must be != 0.


	*/
	static void compute_params_fixed_design(
		int angles, int eccentr,
		double imageplane_height, double imageplane_width,
		double log_shift, double blind_radius,  
		double *log_fact, double *second_radius );

	
	/*
		FUNCTION: 		compute_params_balanced_design
		DESCRIPTION:	Compute the logpolar parameters log_fact, blind_radius and second_radius given the 
						the other parameter and the size of the image plane.
						The parameters are computed such that the logpolar cells has low eccentricity and 
						the limit of the image plane coincides with the outer cells of the logpolar space.

	*/
	static void compute_params_balanced_design(
		int angles, int eccentr,
		double imageplane_height, double imageplane_width,
		double log_shift, double *blind_radius,  
		double *log_fact, double *second_radius );
	
	// obsolete (just for reference) - do not use
	bool compute_params_generic_design( //double sensor_height, double sensor_width, long *angles, long *eccentr, double *rmin, double *logfact, double *logshift, double *firstradius)
		int *angles, int *eccentr,
		double imageplane_height, double imageplane_width,
		double *log_shift, double *blind_radius,  
        double *log_fact, double *second_radius );
	
	void map_coordinates(double x, double y, double *u, double *v);
	
    void invmap_coordinates(double u, double v, double *x, double *y);
	
	void set_logpolar_params(int angles, int eccentr, double blind_rad, double sec_rad, double log_fact, double log_shift);
	
	double get_blind_radius();
	
	void put_blind_radius(double newVal);
	
	double get_log_shift();
	
	void put_log_shift(double newVal);
	
	double get_log_fact();
	
    void put_log_fact( double newVal );
	
	double get_second_radius();
	
	void put_second_radius( double newVal );
	
	int get_angles();
	
	void put_angles(int newVal);
	
	int get_eccentr();
	
	void put_eccentr(long newVal);
};

//Sugestão do Julio !!
//double CLogPolarParams::pi = 3.14159265358979323846f;

#endif //__CLOGPOLARPARAMS_H_

