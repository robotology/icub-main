// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Lorenzo Natale
* email:  lorenzo.natale@iit.it
* website: www.robotcub.org
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/
/**
 * \file pidfilter.cpp
 * \brief This file deals with the PID controller
 * \author Lorenzo Natale
 * \date 2007
 * \note Release under GNU GPL v2.0
 **/

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
