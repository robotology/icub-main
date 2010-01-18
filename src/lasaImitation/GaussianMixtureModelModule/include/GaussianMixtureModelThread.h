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

    BufferedPort<Vector>        mSignalPort;
    double                      mSignalLastTime;

    BufferedPort<Vector>        mDSOutputPort;
    BufferedPort<Bottle>        mDSRpcPort;

    Vector                      mSignalVector;
    
    int                         mIOSize;
    int                         mGMMIOSize;
    MathLib::Vector             mInputVector;
    MathLib::Vector             mOutputVector;
    MathLib::Vector             mDSOutputVector;
    
    enum State{
        GMMS_IDLE = 0,
        GMMS_INIT,
        GMMS_INIT_PAUSE,
        GMMS_RUN,
        GMMS_PAUSE,
        GMMS_END
    };
    
    State                       mPrevState;
    State                       mState;
    State                       mNextState;
    
    bool                        bRunPaused;
    
    
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
    
    int                         mCurrDemoId;
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
    enum RunMode{
        RM_NONE = 0,
        RM_REC,
        RM_REPRO,
        RM_CORR
    };
    
private:
    RunMode                     mRunMode;

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
    char                        mEMCurrModelPath[256];
    
    MathLib::Matrix             mEMWeights;
    
    double                      mCurrCorrWeight;
    MathLib::Vector             mCurrOffset;
    MathLib::Vector             mGlobalOffset;

public:
    GaussianMixtureModelThread(int period, const char* baseName);
    virtual ~GaussianMixtureModelThread();

            void    Init();
            void    Free();
            void    Load(const char* modelPath);
            void    Save(const char* modelPath);

            void    SetRunMode(RunMode mode);
    
            void    InitRun();
            void    StartRun();
            void    PauseRun(bool useMutex = true);
            void    ResumeRun(bool useMutex = true, bool once = false);
            void    StopRun();
    
            void    SetGMRReproTime(double time);
    
            void    SetEMDemosPath(const char* path);
            void    SetEMCorrDemosPath(const char* path);
            void    SetEMProcessingMode(ProcessMode mode);
            void    SetEMInitialisationMode(InitMode mode);
            void    SetEMNbComponents(int nb);
            void    SetEMDemoLength(int len);
            void    SetEMTimeSpan(double timeSpan);
            void    SetCorrectionMode(bool mode);
            
            void    SetCurrDemoId(int id);
    
            bool    Learn();

            bool    ProcessRawDemos();
    
            void    GenerateDefaultRegression();
            void    GenerateGnuplotScript();
    
    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();

protected:
            void SendCommandToDataStreamer(const char *cmd);

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

MathLib::Vector& FilterSW(const MathLib::Vector& src, MathLib::Vector& result, int windowLength);
MathLib::Matrix& FilterQuatSW(const MathLib::Matrix& src, MathLib::Matrix& result, int windowLength);
MathLib::Matrix& FilterQPoseSW(const MathLib::Matrix& src, MathLib::Matrix& result, int windowLength);

#endif

