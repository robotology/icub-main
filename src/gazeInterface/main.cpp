// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>


#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>

#include <gsl/gsl_math.h>

#ifdef USE_ICUB_MOD
    #include "drivers.h"
#endif

#include <iostream>
#include <iomanip>
#include <string>
#include <stdio.h>
#include <deque>

#define MAX_TORSO_PITCH     30.0    // [deg]
#define EXECTIME_THRESDIST  0.3     // [m]
#define PRINT_STATUS_PER    1.0     // [s]
#define CTRL_THREAD_PER     0.02        // [s]
#define PRINT_STATUS_PER    1.0         // [s]
#define STORE_POI_PER       3.0         // [s]
#define SWITCH_STATE_PER    10.0        // [s]
#define STILL_STATE_TIME    5.0         // [s]


#define STATE_TRACK         0
#define STATE_RECALL        1
#define STATE_WAIT          2
#define STATE_STILL         3

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;



class CtrlThread: public RateThread
{
protected:
    PolyDriver       *clientGaze;
    PolyDriver       *clientTorso;
    IGazeControl     *igaze;
    IEncoders        *ienc;
    IPositionControl *ipos;

    int state;

    Vector fp;

    deque<Vector> poiList;

    double t;
    double t0;
    double t1;
    double t2;
    double t3;
    double t4;

public:
    CtrlThread(const double period) : RateThread(int(period*1000.0)) { }

    virtual bool threadInit()
    {
        // open a client interface to connect to the gaze server
        // we suppose that:
        // 1 - the iCub simulator (icubSim) is running
        // 2 - the gaze server iKinGazeCtrl is running and
        //     launched with --robot icubSim option
        Property optGaze("(device gazecontrollerclient)");
        optGaze.put("remote","/iKinGazeCtrl");
        optGaze.put("local","/gaze_client");

        clientGaze=new PolyDriver;
        if (!clientGaze->open(optGaze))
        {
            delete clientGaze;    
            return false;
        }

        // open the view
        clientGaze->view(igaze);

        // set trajectory time:
        // we'll go like hell since we're using the simulator :)
        igaze->setNeckTrajTime(0.4);
        igaze->setEyesTrajTime(0.1);

        // put the gaze in tracking mode, so that
        // when the torso moves, the gaze controller 
        // will compensate for it
        igaze->setTrackingMode(true);

        Property optTorso("(device remote_controlboard)");
        optTorso.put("remote","/icub/torso");
        optTorso.put("local","/torso_client");

        clientTorso=new PolyDriver;
        if (!clientTorso->open(optTorso))
        {
            delete clientTorso;    
            return false;
        }

        // open the view
        clientTorso->view(ienc);
        clientTorso->view(ipos);
        ipos->setRefSpeed(0,10.0);

        fp.resize(3);

        state=STATE_TRACK;

		t=t0=t1=t2=t3=t4=Time::now();

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");        
    }

    virtual void run()
    {
        t=Time::now();

        generateTarget();        

        if (state==STATE_TRACK)
        {
            // look at the target (streaming)
            igaze->lookAtFixationPoint(fp);

            // some verbosity
            printStatus();

            // we collect from time to time
            // some interesting points (POI)
            // where to look at soon afterwards
            storeInterestingPoint();

            if (t-t2>=SWITCH_STATE_PER)
            {
                // switch state
                state=STATE_RECALL;
            }
        }

        if (state==STATE_RECALL)
        {
            // pick up the first POI
            // and clear the list
            Vector ang=poiList.front();
            poiList.clear();

            fprintf(stdout,"Retrieving POI #0 ... %s [deg]\n",
                    ang.toString().c_str());

            // look at the chosen POI
            igaze->lookAtAbsAngles(ang);

            // switch state
            state=STATE_WAIT;
        }

        if (state==STATE_WAIT)
        {
            bool done=false;
            igaze->checkMotionDone(&done);

            if (done)
            {
                Vector ang;
                igaze->getAngles(ang);            

                fprintf(stdout,"Actual gaze configuration: %s [deg]\n",
                        ang.toString().c_str());

                fprintf(stdout,"Moving the torso; see if the gaze is compensated ... ");
                
                // move the torso yaw
                double val;
                ienc->getEncoder(0,&val);
                ipos->positionMove(0,val>0.0?-30.0:30.0);

                t4=t;

                // switch state
                state=STATE_STILL;
            }
        }

        if (state==STATE_STILL)
        {
            if (t-t4>=STILL_STATE_TIME)
            {
                fprintf(stdout,"done\n");

                t1=t2=t3=t;

                // switch state
                state=STATE_TRACK;
            }
        }
    }

    virtual void threadRelease()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        // (anyway it's already done internally in the
        // destructor)
        igaze->stopControl();

        // it's a good rule to reinstate the tracking mode
        // as it was
        igaze->setTrackingMode(false);

        delete clientGaze;
        delete clientTorso;
    }

    void generateTarget()
    {   
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.5,0.0,0.3] with radius=0.1 m
        // and frequency 0.1 Hz
        fp[0]=-0.5;
        fp[1]=+0.0+0.1*cos(2.0*M_PI*0.1*(t-t0));
        fp[2]=+0.3+0.1*sin(2.0*M_PI*0.1*(t-t0));            
    }

    void storeInterestingPoint()
    {
        if (t-t3>=STORE_POI_PER)
        {
            Vector ang;

            // we store the current azimuth, elevation
            // and vergence wrt the absolute reference frame
            // The absolute reference frame for the azimuth/elevation couple
            // is head-centered with the robot in rest configuration
            // (i.e. torso and head angles zeroed). 
            igaze->getAngles(ang);            

            fprintf(stdout,"Storing POI #%d ... %s [deg]\n",
                    poiList.size(),ang.toString().c_str());

            poiList.push_back(ang);

            t3=t;
        }
    }

    double norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void printStatus()
    {        
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x;

            // we get the current fixation point in the
            // operational space
            igaze->getFixationPoint(x);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"fp         [m] = %s\n",fp.toString().c_str());
            fprintf(stdout,"x          [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"norm(fp-x) [m] = %g\n",norm(fp-x));
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CtrlThread(CTRL_THREAD_PER);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};  



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--ctrlName name: controller name (default iKinArmCtrlIF)"                    << endl;
        cout << "\t--robot    name: robot name to connect to (default: icub)"                   << endl;
        cout << "\t--part     type: robot arm type, left_arm or right_arm (default: right_arm)" << endl;
        cout << "\t--T        time: specify the task execution time in seconds (default: 2.0)"  << endl;
        cout << "\t--DOF10        : control the torso yaw/roll/pitch as well"                   << endl;
        cout << "\t--DOF9         : control the torso yaw/pitch as well"                        << endl;
        cout << "\t--DOF8         : control the torso yaw as well"                              << endl;
        cout << "\t--onlyXYZ      : disable orientation control"                                << endl;

        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

#ifdef USE_ICUB_MOD
    DriverCollection dev;
#endif

    CtrlModule mod;

    return mod.runModule(rf);
}



