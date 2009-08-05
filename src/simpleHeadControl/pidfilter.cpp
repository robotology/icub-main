//
//  $Id: pidfilter.cpp,v 1.3 2008/01/09 11:37:40 nat Exp $
//
//

#include "pidfilter.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
PidFilter::PidFilter(void)
{
	error_old=0;
	Kp=0;
	Kd=0;
	Ki=0;

	Umax = 0.0;
	Sn = 0.0;
}

PidFilter::PidFilter(double kp,double kd,double ki,double u_max)
{
	error_old=0;
	Kp=kp;
	Kd=kd;
	Ki=ki;

	Umax = u_max;
	Sn = 0.0;
}

PidFilter::~PidFilter(void)
{
}

PidFilter::PidFilter(const PidFilter& f)
{
	error_old=f.error_old;
	Kp=f.Kp;
	Kd=f.Kd;
	Ki=f.Ki;

	Umax = f.Umax;
	Sn = f.Sn;
}

void PidFilter::operator=(const PidFilter& f)
{
	error_old=f.error_old;
	Kp=f.Kp;
	Kd=f.Kd;
	Ki=f.Ki;
	
	Umax = f.Umax;
	Sn = f.Sn;
}
