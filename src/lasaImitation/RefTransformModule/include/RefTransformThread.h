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

#ifndef RefTransformTHREAD_H_
#define RefTransformTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include "MathLib/MathLib.h"

class RefTransformThread: public RateThread
{
private:    
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    double                  mTime;
    double                  mPrevTime;

    BufferedPort<Vector>    mInputPosePort;
    BufferedPort<Vector>    mOutputPosePort;
    BufferedPort<Vector>    mInputPoseInRefPort;
    BufferedPort<Vector>    mOutputPoseInRefPort;
    BufferedPort<Vector>    mInputRefPort;

    double                  mInputPoseLastTime;
    double                  mInputPoseInRefLastTime;
    double                  mInputRefPortLastTime;

    
    MathLib::Vector3        mInputPosePos;
    MathLib::Vector3        mInputPoseOri;
    MathLib::Vector3        mOutputPosePos;
    MathLib::Vector3        mOutputPoseOri;
    MathLib::Vector3        mInputPoseInRefPos;
    MathLib::Vector3        mInputPoseInRefOri;
    MathLib::Vector3        mOutputPoseInRefPos;
    MathLib::Vector3        mOutputPoseInRefOri;
    MathLib::Vector3        mInputRefPos;
    MathLib::Vector3        mInputRefOri;

    MathLib::Matrix4        mRef;
    MathLib::Matrix4        mRefNoOri;
    MathLib::Matrix4        mInvRef;
    MathLib::Matrix4        mInvRefNoOri;
    
    bool                    bBypass;
    bool                    bLocked;
    bool                    bEnableOrient;
    bool                    bRun;
    
public:
    RefTransformThread(int period, const char* baseName);
    virtual ~RefTransformThread();

            void            Enable(bool enable = true);
            void            Lock(bool lock = true);
            void            EnableOrient(bool enable = true);
            void            Activate(bool enable = true);


    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
};

#endif

