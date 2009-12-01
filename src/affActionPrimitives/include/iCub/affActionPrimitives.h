
#ifndef __AFFACTIONPRIMITIVES_H__
#define __AFFACTIONPRIMITIVES_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/handMetrics.h>

#include <deque>
#include <map>
#include <set>


class affActionPrimitives : public yarp::os::RateThread
{
protected:
    yarp::dev::PolyDriver        *polyHand;
    yarp::dev::PolyDriver        *polyCart;
    yarp::dev::IEncoders         *encCtrl;
    yarp::dev::IPidControl       *pidCtrl;
    yarp::dev::IAmplifierControl *ampCtrl;
    yarp::dev::IPositionControl  *posCtrl;
    yarp::dev::ICartesianControl *cartCtrl;

    yarp::os::Semaphore *mutex;
    void                *motionDoneEvent;

    bool armMoveDone;
    bool handMoveDone;
    bool configured;
    bool closed;
    bool checkEnabled;

    bool latchArmMoveDone;
    bool latchHandMoveDone;

    int iMin, iMax;

    yarp::sig::Vector oGrasp;
    yarp::sig::Vector oTap;

    yarp::sig::Vector xHome;
    yarp::sig::Vector oHome;

    yarp::sig::Vector xd;
    yarp::sig::Vector od;
    yarp::sig::Vector dTap;
    yarp::sig::Vector dGrasp;

    yarp::sig::Vector thresholds;
    yarp::sig::Vector fingerOpenPos;
    yarp::sig::Vector fingerClosePos;
    yarp::sig::Vector fingerAlignPos;
    std::set<int> enabledJoints;
    std::set<int> activeJoints;
    std::map<const std::string, yarp::sig::Matrix> sensingConstants;
    HandMetrics      *handMetrics;
    FunctionSmoother *fs;

    typedef struct
    {
        bool execReach;
        yarp::sig::Vector x;
        yarp::sig::Vector o;
        bool handId;
    } MotorIFQueue;

    std::map<const int,bool (affActionPrimitives::*)(const bool)> executeHand;
    std::deque<MotorIFQueue> q;

    virtual void stopBlockedJoints(std::set<int> &activeJoints);
    virtual bool handMotionDone(const std::set<int> &joints);
    virtual bool moveHand(const yarp::sig::Vector &fingerPos, const bool sync);
    virtual bool nopHand(const bool sync=false) { return true; }

    void queue_clear();
    void queue_push(const yarp::sig::Vector &x, const yarp::sig::Vector &o, const int handId);
    void queue_push(const yarp::sig::Vector &x, const yarp::sig::Vector &o);
    void queue_push(const int handId);
    void queue_exec();

    virtual void run();    

public:
    affActionPrimitives();
    ~affActionPrimitives();

    bool open(yarp::os::Property &opt);
    void close();

    // predefined quantities
    void setGraspOrien(const yarp::sig::Vector &o);
    void setTapOrien(const yarp::sig::Vector &o);
    void setHome(const yarp::sig::Vector &x, const yarp::sig::Vector &o);
    void setGraspDisplacement(const yarp::sig::Vector &disp);
    void setTapDisplacement(const yarp::sig::Vector &disp);
                                            
    yarp::sig::Vector getGraspOrien()        { return oGrasp; }
    yarp::sig::Vector getTapOrien()          { return oTap;   }
    yarp::sig::Vector getGraspDisplacement() { return dGrasp; }
    yarp::sig::Vector getTapDisplacement()   { return dTap;   }

    // actions list
    virtual bool reach(const yarp::sig::Vector &x, const yarp::sig::Vector &o, const bool sync=false);
    virtual bool grasp(const yarp::sig::Vector &x, const bool sync=false);
    virtual bool touch(const yarp::sig::Vector &x, const bool sync=false);
    virtual bool tap  (const yarp::sig::Vector &x, const bool sync=false);
    virtual bool home(const bool sync=false);
    virtual bool openHand(const bool sync=false);
    virtual bool closeHand(const bool sync=false);
    virtual bool alignHand(const bool sync=false);

    virtual bool getPose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool check(bool &f, const bool sync=false);

    virtual void syncCheckInterrupt();
    virtual void syncCheckReinstate();
};

#endif


