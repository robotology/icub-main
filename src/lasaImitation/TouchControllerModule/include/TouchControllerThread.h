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

#ifndef TouchControllerTHREAD_H_
#define TouchControllerTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include "MultipleMiceDriver/MMiceDeviceDriver.h"
#include "TouchController.h"

class TouchControllerThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    int                     mType;
    char                    mDeviceName[256];
    
    MMiceDeviceDriver       mMiceDriver;
    TouchController         mTouchController;
    
    double                  mTransGain;
    double                  mRotGain;
    double                  mTransLimit;
    double                  mRotLimit;
    Vector                  mCoefs;
    
    Matrix                  mFrameOfRef;
    
    BufferedPort<Matrix>    mFrameOfRefPort;
    BufferedPort<Vector>    mOutputPort;
    
    bool                    bRunning;
    
public:
    TouchControllerThread(int period, const char* baseName);
    virtual ~TouchControllerThread();

            void    SetDevice(const char* deviceName,int type);
            void    SetSensorIdByTouch(int id);
            void    Activate(bool act = true);
            void    SetTransGain(double gain);
            void    SetTransLimit(double gain);
            void    SetRotGain(double gain);
            void    SetRotLimit(double gain);
    
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
};

#endif

