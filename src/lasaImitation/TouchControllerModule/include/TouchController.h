#ifndef TOUCHCONTROLLER_H_
#define TOUCHCONTROLLER_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/Matrix.h>

#include "MultipleMiceDriver/MMiceDeviceDriver.h"

using namespace yarp::dev;
using namespace yarp::os;

class TouchControllerImpl;

class TouchController
{
private:
    TouchControllerImpl *impl;

public:
    TouchController();
    TouchController(MMiceDeviceDriver *miceDriver);
    ~TouchController();
    
    void    Update();

    void    Get6DOFSensingOutput(yarp::sig::Vector &output);
    void    Get6DOFSensingOutputMask(yarp::sig::Vector &mask);

    void    Get2DOFSensingOutput(yarp::sig::Vector &output);

    void    GetOrientationFrameOutput(yarp::sig::Vector &output);    

    void    GetOrientationFrameIntegratedOutput(yarp::sig::Vector &output);    
    void    ResetIntegratedOutput();    

    void    Set6DOFSensingCoefs(yarp::sig::Vector &coefs);
    void    SetMMiceDriver(MMiceDeviceDriver *miceDriver);
    
    void    SetOrientationFrame(yarp::sig::Matrix &frame);    
    void    SetOutputLimits(double transLimit, double orientLimit);    

    void    SetDecayFactor(double factor = -1.0);

    bool    TouchDetected();
    bool    ResetDetected();
    
    bool    GetLastMouseTouch();
};
    
#ifdef TOUCHCONTROLLER_CPP_

#include "MathLib/MathLib.h"
using namespace MathLib;
class TouchControllerImpl
{
public:
    TouchControllerImpl();

    void    Update();

    void    Get6DOFSensingOutput(yarp::sig::Vector &output);
    void    Get6DOFSensingOutputMask(yarp::sig::Vector &mask);
    void    GetOrientationFrameOutput(yarp::sig::Vector &output);    
    void    GetOrientationFrameIntegratedOutput(yarp::sig::Vector &output);    
    void    ResetIntegratedOutput();    
    
    void    Get2DOFSensingOutput(yarp::sig::Vector &output);

    void    Set6DOFSensingCoefs(yarp::sig::Vector &coefs);
    void    SetMMiceDriver(MMiceDeviceDriver *miceDriver);

    void    SetOrientationFrame(yarp::sig::Matrix &frame);    
    void    SetOutputLimits(double transLimit, double orientLimit); 

    void    SetDecayFactor(double factor = -1.0);

    bool    TouchDetected();
    bool    ResetDetected();

    bool    GetLastMouseTouch();

protected:
    Matrix3            mOrientFrame;
    Vector3            mTransComp;
    Vector3            mOrientComp;
    Vector3            mTransCompOut;
    Vector3            mOrientCompOut;
    Vector3            mTransCompIntOut;
    Vector3            mOrientCompIntOut;
    double             mTransLimit;
    double             mOrientLimit;
    
    double             mLastTouchTime;
    double             mTouchResetTime;
    
    bool               bResetDetected;
    
    double             mLastTime;

    MMiceDeviceDriver *mMiceDriver;
    double             mMiceLimitX;
    double             mMiceLimitY;
    double             mMiceLimitRelX;
    double             mMiceLimitRelY;
    
    double             m6DofSensorsRelOutput[6];
    double             m6DofSensorsRelOutputMem[6];
    double             m6DofSensorsOutput[6];
    double             m6DofSensorsOutputMask[6];
    double             mCoefs[6];
    bool               bLastBtnState;

    double             m2DofSensorsOutput[6];
    double             m2DofCompOut[2];
                
    double             mLastRelTime[6];
    double             mDecayFactor;
    
    bool               mLastBigMoveStatus;
    
    bool               bLastMiceTouch;
};
#endif



#endif /*TOUCHCONTROLLER_H_*/

