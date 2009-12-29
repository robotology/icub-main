// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

 
#include "MyDefaultThread.h"

#include <string.h>


MyDefaultThread::MyDefaultThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

MyDefaultThread::~MyDefaultThread()
{}

bool MyDefaultThread::threadInit()
{
    char portName[256];
    snprintf(portName,256,"/%s/input",mBaseName);
    mInputPort.open(portName);

    snprintf(portName,256,"/%s/output",mBaseName);
    mOutputPort.open(portName);

    return true;
}

void MyDefaultThread::threadRelease()
{
    mInputPort.close();
    mOutputPort.close();
}

void MyDefaultThread::run()
{
    mMutex.wait();
    
    // Read data from input port
    Vector *inputVec = mInputPort.read(false);
    if(inputVec!=NULL){}

    // Write data to output port
    Vector &outputVec = mOutputPort.prepare();
    mOutputPort.write();

    mMutex.post();
}

