// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#include <iCub/DebugInterfaces.h>
//A collection of debug methods useful to send raw commands directly to the control boards.

bool yarp::dev::ImplementDebugInterface::setParameter(int j, unsigned int type, double value)
{
	int k=castToMapper2(helper)->toHw(j);
	return raw->setParameterRaw(j, type, value);
}

bool yarp::dev::ImplementDebugInterface::getParameter(int j, unsigned int type, double* value)
{
	int k=castToMapper2(helper)->toHw(j);
    return raw->getParameterRaw(j, type, value);
}

bool yarp::dev::ImplementDebugInterface::setDebugParameter(int j, unsigned int index, double value)
{
	int k=castToMapper2(helper)->toHw(j);
	return raw->setDebugParameterRaw(j, index, value);
}

bool yarp::dev::ImplementDebugInterface::getDebugParameter(int j, unsigned int index, double* value)
{
	int k=castToMapper2(helper)->toHw(j);
    return raw->getDebugParameterRaw(j, index, value);
}

bool yarp::dev::ImplementDebugInterface::setDebugReferencePosition(int j, double value)
{
	int k=castToMapper2(helper)->toHw(j);
	double enc=castToMapper2(helper)->posA2E(value,k);
	return raw->setDebugReferencePositionRaw(j, enc);
}

bool yarp::dev::ImplementDebugInterface::getDebugReferencePosition(int j, double* value)
{
	int k=castToMapper2(helper)->toHw(j);
	double enc=0.0;
    bool r= raw->getDebugReferencePositionRaw(j, &enc);
	if (r)
	{
		*value = castToMapper2(helper)->posE2A(enc,k);
	}
	else
	{
		*value = 0.0;
	}
	return r;
}

yarp::dev::ImplementDebugInterface::ImplementDebugInterface(IDebugInterfaceRaw *r)
{
    raw=r;
    helper=0;
}

bool yarp::dev::ImplementDebugInterface::initialize(int size, const int *amap, const double *angleToEncoder)
{
    if (helper!=0)
        return false;
    
    double *dummy=new double [size];
    for(int k=0;k<size;k++)
        dummy[k]=0;

    helper=(void *)(new ControlBoardHelper2(size, amap, angleToEncoder, dummy, dummy));
    _YARP_ASSERT_DEBUG(helper != 0);

    delete [] dummy;
    return true;
}

yarp::dev::ImplementDebugInterface::~ImplementDebugInterface()
{
    uninitialize();
}

bool yarp::dev::ImplementDebugInterface::uninitialize ()
{
    if (helper!=0)
    {
        delete castToMapper2(helper);
        helper=0;
    }
 
    return true;
}

