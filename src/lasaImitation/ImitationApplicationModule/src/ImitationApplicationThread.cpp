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

 
#include "ImitationApplicationThread.h"

#include <string.h>

#include <iostream>
using namespace std;

ImitationApplicationThread::ImitationApplicationThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

ImitationApplicationThread::~ImitationApplicationThread()
{}

bool ImitationApplicationThread::threadInit()
{
    char portName[256];
    //snprintf(portName,256,"/%s/input",mBaseName);
    //mInputPort.open(portName);

    snprintf(portName,256,"/%s/VC",mBaseName);
    mVelocityControllerPort.open(portName);

    return true;
}

void ImitationApplicationThread::threadRelease()
{
    mVelocityControllerPort.close();
    //mInputPort.close();
    //mOutputPort.close();
}

void ImitationApplicationThread::run()
{
    mMutex.wait();
    
    // Read data from input port
    //Vector *inputVec = mInputPort.read(false);
    //if(inputVec!=NULL){}

    // Write data to output port
    //Vector &outputVec = mOutputPort.prepare();
    //mOutputPort.write();

    mMutex.post();
}


    
int ImitationApplicationThread::respond(const Bottle& command, Bottle& reply){
    int  cmdSize    = command.size();
    int  retVal     = -1;
    
    if(cmdSize<=0){
        retVal = -1;
    }else{
        switch(command.get(0).asVocab()) {
        case VOCAB4('i','n','i','t'):
            retVal = 1;
            break;
        case VOCAB3('r','u','n'):
            {
                Bottle &vcCmd = mVelocityControllerPort.prepare();
                vcCmd.addString("run");
                mVelocityControllerPort.write();
                retVal = 1;
            }
            break;
        case VOCAB4('s','t','o','p'):
            retVal = 1;
            break;
        }
    }
    if(retVal>0){
        reply.addVocab(Vocab::encode("ack"));
    }
    return retVal;
}