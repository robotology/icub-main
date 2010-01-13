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

#ifndef GaussianMixtureModelTHREAD_H_
#define GaussianMixtureModelTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::sig;

#include "MathLib/MathLib.h"
#include "MathLib/GMR.h"



class GaussianMixtureModelThread: public RateThread
{
private:    
    Semaphore                   mMutex;
    int                         mPeriod;
    char                        mBaseName[256];

    double                      mTime;
    double                      mPrevTime;

    BufferedPort<Vector>        mInputPort;
    BufferedPort<Vector>        mOutputPort;
    double                      mInputLastTime;
    
    int                         mIOSize;
    int                         mGMMIOSize;
    MathLib::Vector             mInputVector;
    MathLib::Vector             mOutputVector;
    
    enum State{
        GMM_IDLE = 0,
        GMM_REPRO_INIT,
        GMM_REPRO_INIT_PAUSE,
        GMM_REPRO_RUN,
        GMM_REPRO_PAUSE,
        GMM_REPRO_END
    };
    
    State                       mPrevState;
    State                       mState;
    State                       mNextState;
    
    
    MathLib::GaussianMixture    mGMM;
    bool                        bGMMIsReady;
    
    double                      mGMMTime;
    double                      mGMMInternalTimeSpan;
    double                      mGMMReproTime;

    MathLib::Matrix             mGMMInput;
    MathLib::Vector             mGMMInputV;
    MathLib::Vector             mGMMLambda;
    MathLib::Matrix             mGMMTarget;
    MathLib::Vector             mGMMTargetV;
    MathLib::Matrix            *mGMMSigmas;
    
    MathLib::Vector             mGMMInComp;
    MathLib::Vector             mGMMOutComp;

    MathLib::Vector             mGMMCurrState;
    
    double                      mGMMLambdaTreshold;
    double                      mGMMLambdaTau;
    
    // Learning
public:
    enum ProcessMode{
        PM_NONE = 0,
        PM_SIMPLE,
        PM_WEIGHTED
    };
    enum InitMode{
        IM_NONE = 0,
        IM_TIMESPLIT,
        IM_KMEANS,
        IM_RAND
    };
    
private:
    ProcessMode                 mEMProcessMode;
    InitMode                    mEMInitMode;
    int                         mEMNbComponents;
    int                         mEMNbDemos;
    int                         mEMNbCorrDemos;
    int                         mEMNbDimensions;
    int                         mEMDemoLength;
    
    double                      mEMTimeSpan;
    
    char                        mEMDemosPath    [256];
    char                        mEMCorrDemosPath[256];
    
    MathLib::Matrix             mEMWeights;


public:
    GaussianMixtureModelThread(int period, const char* baseName);
    virtual ~GaussianMixtureModelThread();

            void    Init();
            void    Free();
            void    Load(const char* modelPath);
            void    Save(const char* modelPath);

            void    InitRepro();
            void    StartRepro();
            void    PauseRepro();
            void    ResumeRepro();
            void    StopRepro();
    
            void    SetEMDemosPath(const char* path);
            void    SetEMCorrDemosPath(const char* path);
            void    SetEMProcessingMode(ProcessMode mode);
            void    SetEMInitialisationMode(InitMode mode);
            void    SetEMNbComponents(int nb);
            void    SetEMDemoLength(int len);

            bool    Learn();

            bool    ProcessRawDemos();
    
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();


};


MathLib::Vector& AxisToQuat  (const MathLib::Vector& axis,  MathLib::Vector& quat, bool inv = false);
MathLib::Vector& QuatToAxis  (const MathLib::Vector& quat,  MathLib::Vector& axis);
MathLib::Vector& Pose6ToQPose(const MathLib::Vector& pose,  MathLib::Vector& qpose,bool inv = false);
MathLib::Vector& QPoseToPose6(const MathLib::Vector& qpose, MathLib::Vector& pose);

MathLib::Matrix& AxisToQuat  (const MathLib::Matrix& axis,  MathLib::Matrix& quat, bool inv = false);

MathLib::Matrix& SmoothQuatStream(MathLib::Matrix& qstream);
MathLib::Matrix& AlignQuatStreamEnd(MathLib::Matrix& qstream, const MathLib::Vector& end);
MathLib::Matrix& Resample(const MathLib::Matrix& src, MathLib::Matrix& result, int length);
MathLib::Matrix& Pose6ToQPose(const MathLib::Matrix& pose,  MathLib::Matrix& qpose);

#endif

