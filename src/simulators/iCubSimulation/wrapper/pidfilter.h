
/**
 * \file pidfilter.h
 * \brief Header to implement the PID filter
 * \author Lorenzo Natale
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

#ifndef __PidFilterh__
#define __PidFilterh__

#include <math.h>

class PidFilter  
{
private:
	double error_old;		//error at previous step
	double Kp,Kd,Ki;		//proportional, derivative and integral gains

	// integrative stuff
	double Umax;			//maxium value of the control
	double Sn;				//integal value

	//computes the pd portion of the control
	inline double pd(double error) 
	{
		double ret=Kp*error+Kd*(error-error_old);
		return ret;
	}

public:
	PidFilter(void);
	PidFilter(double kp, double kd=0, double ki = 0, double u = 0.0);
	PidFilter(const PidFilter& f);
	void operator=(const PidFilter& f);
	virtual ~PidFilter(void);

	inline void setKs(double kp, double kd=0.0, double ki=0.0, double u_max = 0.0)
	{
		Kp = kp;
		Kd = kd;
		Ki = ki;

		Sn = 0;
		Umax = u_max;
	}

	// computes the PID control with anti reset wind up scheme
	inline double pid (double error)
	{
		double u_tmp;
		double Sn_tmp;
		double u_pd;
		double u;

		//compute the pd part
		u_pd = pd(error);

		//compute the temporary integral part
		Sn_tmp = Sn + Ki * error;

		//compute the temporary control
		u_tmp = u_pd + Sn_tmp;

		//if no saturation occur, then temporary works fine
		Sn = Sn_tmp;

		//if saturation occur, redifine integral part
		if (u_tmp > Umax)
			Sn = Umax - u_pd; 
		if (u_tmp < -Umax)
			Sn = -Umax - u_pd;

		//redifine error_old
		error_old = error;

		//compute the control
		u = Sn + u_pd;

		return u;
	}

	inline double getProportional(void) const { return Kp; }
	inline double getDerivative(void) const { return Kd; }
	inline double getIntegrative(void) const { return Ki; }
};

#endif 
