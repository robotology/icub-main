// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

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


#include "CLogPolarParams.h"


/*
FUNCTION: 		compute_params_fixed_design
DESCRIPTION:    Compute the logpolar parameters log_fact and second_radius given the current
				the other parameter and the size of the image plane.
				The parameters are computed such that the limit of the image plane coincides with 
				the outer cells of the logpolar space.
NOTE:           If blind_radius == 0, log_shift must be != 0.


*/
void CLogPolarParams::compute_params_fixed_design(
	int angles, int eccentr,
	double imageplane_height, double imageplane_width,
	double log_shift, double blind_radius,  
	double *log_fact, double *second_radius )
{
	//parameter "angles" is not used
	double dim_image;

	if( imageplane_height <= 0.0f )  throw "Invalid parameter";  //nothing to do
	if( imageplane_width <= 0.0f )	 throw "Invalid parameter";

	if((blind_radius == 0.0f) && (log_shift == 0.0f)) throw "Invalid function use";

	if(imageplane_width < imageplane_height)
		dim_image = imageplane_width;
	else
		dim_image = imageplane_height;

	*log_fact = (double)exp( log((dim_image/2.0f+log_shift)/(blind_radius+log_shift))/(double)eccentr );
	*second_radius = (blind_radius + log_shift)*(*log_fact) - log_shift;
}

/*
	FUNCTION: 		compute_params_balanced_design
	DESCRIPTION:	Compute the logpolar parameters log_fact, blind_radius and second_radius given the 
					the other parameter and the size of the image plane.
					The parameters are computed such that the logpolar cells has low eccentricity and 
					the limit of the image plane coincides with the outer cells of the logpolar space.

*/
void CLogPolarParams::compute_params_balanced_design(
	int angles, int eccentr,
	double imageplane_height, double imageplane_width,
	double log_shift, double *blind_radius,  
	double *log_fact, double *second_radius )
{
	double dim_image;

	if( imageplane_height <= 0.0f )  throw "Invalid parameter";  //nothing to do
	if( imageplane_width <= 0.0f )	 throw "Invalid parameter";

	if(imageplane_width < imageplane_height)
		dim_image = imageplane_width;
	else
		dim_image = imageplane_height;

	if(log_shift == 0.0f) //
	{
		*blind_radius = 1/(2*(double)tan(pi/(double)angles));
		*log_fact = (double)exp( log(dim_image/2.0/(*blind_radius))/(double)eccentr );
	}
	else
	{
		*blind_radius = 0.0f;
		*log_fact = (double)exp( log((dim_image/2.0 + log_shift/log_shift))/(double)eccentr );
	}
	*second_radius = (*blind_radius + log_shift)*(*log_fact) - log_shift;
}

// obsolete (just for reference) - do not use
bool CLogPolarParams::compute_params_generic_design( //double sensor_height, double sensor_width, long *angles, long *eccentr, double *rmin, double *logfact, double *logshift, double *firstradius)
	int *angles, int *eccentr,
	double imageplane_height, double imageplane_width,
	double *log_shift, double *blind_radius,  
	double *log_fact, double *second_radius )
{
	double dim_image;

	if( imageplane_height <= 0.0f )  throw "Invalid parameter";  //nothing to do
	if( imageplane_width <= 0.0f )	 throw "Invalid parameter";

	if(imageplane_width < imageplane_height)
		dim_image = imageplane_width;
	else
		dim_image = imageplane_height;

	int k = 0, m = 0, r = 0, s = 0;

	if( *angles <= 0  )
		*angles = 64;   // assume a default value

	if( *eccentr > 0 )			m = 1;
	if( *blind_radius > 0 )		r = 1;
	if( *log_fact > 0 )			k = 1;
	if( *log_shift > 0 )		s = 1;

	if( !k && !r && !m && !s)		 //nothing is specified assume logshift = 0;
	{
		(*log_shift) = 0;
		(*log_fact) = 1+2.0f*pi/(*angles);				// ideal value
		(*blind_radius) = 1/(2*(double)tan(pi/(*angles)));      // ideal value
		(*eccentr) = (int)(log(dim_image/2.0f/(*blind_radius))/log((*log_fact)));
	}
	if( !k && !r && !m && s)		 //logshift is specified - assume rhomin = 0;
	{
		(*log_fact) = 1+2.0f*pi/(*angles);				// ideal value
		(*blind_radius) = 0;
		(*eccentr) = (int)(log((dim_image/2.0f+(*log_shift))/(*log_shift))/log((*log_fact)));
	}
	if( !k && r && !m && !s)		 //only min radius is specified
	{
		(*log_shift) = 0;
		(*log_fact) = 1+2.0f*pi/(*angles);        // ideal value
		(*eccentr) = (int)(log(dim_image/2.0f/(*blind_radius))/log((*log_fact)));
	}
	if( !k && r && !m && !s)		 //min radius and logshift specified
	{
		(*log_fact) = 1+2.0f*pi/(*angles);        // ideal value
		(*eccentr) = (int)(log((dim_image/2.0f+(*log_shift))/((*blind_radius)+(*log_shift)))/log((*log_fact)));
	}
	if( !k && r && m && !s)		//log-fact and logshift not specified
	{
		(*log_shift = 0);
		(*log_fact) = (double)exp( log(dim_image/2.0f/(*blind_radius))/(*eccentr) );
	}
	if( !k && r && m && s)		//only log-fact not specified
	{
		(*log_fact) = (double)exp( log((dim_image/2.0f+(*log_shift))/((*blind_radius)+(*log_shift)))/(*eccentr) );
	}
	if( k && !r && !m && !s)		//only log-fact is specified
	{
		(*log_shift) = 0;
		(*blind_radius) = 1/(2*(double)tan(pi/(*angles)));      // ideal value
		(*eccentr) = (int)(log(dim_image/2.0f/(*blind_radius))/log((*log_fact)));
	}
	if( k && !r && !m && !s)		//only log-fact and logshift specified
	{
		(*blind_radius) = 0;
		(*eccentr) = (int)(log((dim_image/2.0f+(*log_shift))/(*log_shift))/log((*log_fact)));
	}
	if( k && !r && m && !s)		//only min radius is not specified
	{
		(*log_shift) = 0;
		(*blind_radius) = (dim_image/2.0f)/(double)pow((*log_fact), (*eccentr));
	}
	if( k && !r && m && s)		//only min radius is not specified
	{
		(*blind_radius) = (dim_image/2.0f + (*log_shift)*(1-(double)pow((*log_fact), (*eccentr))))/(double)pow((*log_fact), (*eccentr));
	}
	if( k && r && !m && !s)		//eccentr and logshift not specified
	{
		(*log_shift) = 0;
		(*eccentr) = (int)(log(dim_image/2.0/(*blind_radius))/log((*log_fact)));
	}
	if( k && r && !m && s)		//only eccentr not specified
	{
		(*eccentr) = (int)(log((dim_image/2.0f+(*log_shift))/(((*blind_radius)+(*log_shift))))/log((*log_fact)));
	}
	if( !k && !r && m && !s)		//only eccentr is specified
	{
		// FOR BALANCED_CELLS
		//(*log_fact) = 1+2.0*M_PI/(*angles);
		//(*blind_radius) = dim_image/2.0/pow((*log_fact), (*eccentr));
		// FOR BALANCED INNER RADIUS
		(*log_shift) = 0;
		(*blind_radius) = 1/(2*(double)tan(pi/(*angles)));      // ideal value
		(*log_fact) = (double)exp( log(dim_image/2.0/(*blind_radius))/(*eccentr) );
	}
	if( !k && !r && m && s)		//eccentr and logshift specified
	{
		// FOR BALANCED_CELLS
		//(*log_fact) = 1+2.0*M_PI/(*angles);
		//(*blind_radius) = dim_image/2.0/pow((*log_fact), (*eccentr));
		// FOR BALANCED INNER RADIUS
		(*blind_radius) = 0;
		(*log_fact) = (double)exp( log((dim_image/2.0 + (*log_shift))/(*log_shift))/(*eccentr) );
	}
	if( k & r & m )		   //everything is specified ? Unexpected !
	{
		throw "Invalid parameters";
	}
	*second_radius = ((*blind_radius) + (*log_shift))*(*log_fact) - (*log_shift);
}

void CLogPolarParams::map_coordinates(double x, double y, double *u, double *v)
{
	double rho = (double)sqrt(x*x + y*y);
	double theta = (double)atan2(y,x);
	double csi = (double)log((rho+m_fLogShift)/(m_fBlindRadius+m_fLogShift))/(double)log(m_fLogFact);
	double eta = theta/2.0f/pi*m_iAngles;

	if(eta < 0.0f)
		eta += m_iAngles;

	if(eta >= m_iAngles)
		eta -=  m_iAngles;

	*u = csi;
	*v = eta;
}

void CLogPolarParams::invmap_coordinates(double u, double v, double *x, double *y)
{
	double rho = (m_fBlindRadius+m_fLogShift)*(double)pow(m_fLogFact,u) - m_fLogShift;
	double theta = v*2.0f*pi/m_iAngles;
	*x = rho*(double)cos(theta);
	*y = rho*(double)sin(theta);
}

void CLogPolarParams::set_logpolar_params(int angles, int eccentr, double blind_rad, double sec_rad, double log_fact, double log_shift)
{
	put_angles(angles);
	put_eccentr(eccentr);
	put_blind_radius(blind_rad);
	put_second_radius(sec_rad);
	put_log_fact(log_fact);
	put_log_shift(log_shift);
}

double CLogPolarParams::get_blind_radius()
{
	return m_fBlindRadius;
}

void CLogPolarParams::put_blind_radius(double newVal)
{
	if(newVal < 0.0f)
		throw "Invalid Argument";
	if(newVal <= -m_fLogShift)
		throw "blind_radius must be strictly greater than -log_shift";
	if(m_fBlindRadius != newVal)
	{
		m_fBlindRadius = newVal;
		logpolar_has_changed = true;
	}
}

double CLogPolarParams::get_log_shift()
{
	return m_fLogShift;
} 

void CLogPolarParams::put_log_shift(double newVal)
{
	if(newVal <= -m_fBlindRadius)
		throw "log_shift must be strictly greater than -blind_radius";
	if(m_fLogShift != newVal)
	{
		m_fLogShift = newVal;
		logpolar_has_changed = true;
	}
}

double CLogPolarParams::get_log_fact()
{
	return m_fLogFact;
}

void CLogPolarParams::put_log_fact( double newVal )
{
	if(newVal <= 1.0)
		throw "log_fact should be grater than 1";
	if(m_fLogFact != newVal)
	{
		m_fLogFact = newVal;
		logpolar_has_changed = true;
	}
}

double CLogPolarParams::get_second_radius()
{
	return m_fSecondRadius;
}

void CLogPolarParams::put_second_radius( double newVal )
{
	if(newVal <= m_fBlindRadius)
		throw "second_radius should be grater than blind_radius";
	if(m_fSecondRadius != newVal)
	{
		m_fSecondRadius = newVal;
		logpolar_has_changed = true;
	}
}

int CLogPolarParams::get_angles()
{
	return m_iAngles;
}

void CLogPolarParams::put_angles(int newVal)
{
	if(newVal <= 0)
		throw "Invalid Argument";

	if(newVal != m_iAngles)
	{
		m_iAngles = newVal;
		logpolar_has_changed = true;
	}
}

int CLogPolarParams::get_eccentr()
{
	return m_iEccentr;
}

void CLogPolarParams::put_eccentr(long newVal)
{
	if(newVal <= 0)
		throw "Invalid Argument";

	if(newVal != m_iEccentr)
	{
		m_iEccentr = newVal;
		logpolar_has_changed = true;
	}
}

//Sugestão do Julio !!
//double CLogPolarParams::pi = 3.14159265358979323846f;

