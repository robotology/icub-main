// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Gaze Interface.
//
// Author: Ugo Pattacini - Francesco Rea



#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
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
#define STATE_SACCADE       4


// general command vocab's
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_SAC VOCAB3('s','a','c')
#define COMMAND_VOCAB_IMG VOCAB3('i','m','g')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_CHILD_COUNT VOCAB2('c','c')
#define COMMAND_VOCAB_WEIGHT VOCAB1('w')
#define COMMAND_VOCAB_CHILD_WEIGHT VOCAB2('c','w')
#define COMMAND_VOCAB_CHILD_WEIGHTS VOCAB3('c','w','s')
#define COMMAND_VOCAB_NAME VOCAB2('s','1')
#define COMMAND_VOCAB_CHILD_NAME VOCAB2('c','n')
#define COMMAND_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define COMMAND_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')
#define COMMAND_VOCAB_RGB_PROCESSOR VOCAB3('r','g','b')
#define COMMAND_VOCAB_YUV_PROCESSOR VOCAB3('y','u','v')
// directional saliency filter vocab's
#define COMMAND_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define COMMAND_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')

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
    int substate;

    Vector fp;
    Vector vectorImage;


    deque<Vector> poiList;

    double t;
    double t0;
    double t1;
    double t2;
    double t3;
    double t4;

    double z;

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
        vectorImage.resize(2); 

        state=STATE_TRACK;
        substate=0;

		t=t0=t1=t2=t3=t4=Time::now();

        z=1.0;

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

        if(state==STATE_SACCADE){

           printf("----------------- change tracking  -------%f ----%f----- \n",vectorImage[0],vectorImage[1]); 
            
           if (t-t1>=SWITCH_STATE_PER)
            {
                printf("switch state \n");
                state=STATE_TRACK;
            } 
            
        }   

        if (state==STATE_TRACK)
        {
            /*if(substate==STATE_SACCADE){r

               printf("Saccade \n");
               igaze->lookAtMonoPixel(1,vectorImage,z); 
            }
            else{*/
                // look at the target (streaming)
                igaze->lookAtFixationPoint(fp);
            //}

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
        //delete clientTorso;
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

    void changeTracking(double u, double v){
         
        vectorImage[0]=u;
        vectorImage[1]=v;      
        printf("----------------- change tracking  -------%f ----%f----- ",vectorImage[0],vectorImage[1]);        
        // look at the target (streaming)
        igaze->lookAtMonoPixel(1,vectorImage);
        state= STATE_SACCADE;     
    
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
    Semaphore mutex;

    Port resPort;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();
        printf("starting the control thread \n");
        thr=new CtrlThread(CTRL_THREAD_PER);
        if (!thr->start())
        {
            delete thr;
            return false;
        }
        
        resPort.open("/gazeInterface/respondPort:i");
        attach(resPort);
        attachTerminal();
        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        resPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }

    virtual bool respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("help");

            reply.addString("\n");
            reply.addString("get fn \t: general get command \n");
            

            reply.addString("\n");
            reply.addString("set s1 <s> \t: general set command \n");

            reply.addString("\n");
            reply.addString("run rgb : run the rgb processor \n");
            reply.addString("run yuv : run the yuv processor");
            

            reply.addString("\n");


            ok = true;
        }
        break;

    case COMMAND_VOCAB_SAC:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr=0.0;
                reply.addDouble(thr);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = 0;
                reply.addInt(nb);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_IMG:{
                double x = command.get(2).asDouble();
                double y = command.get(3).asDouble();
                string s("sac img");
                sprintf((char *)s.c_str(),"sac img ",x,y);
                thr->changeTracking(x,y);
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NAME:{
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_COUNT:{
                int count =0;
                reply.addInt(count);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_WEIGHT:{
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                //ok = filter->getChildWeights(&weights);
                for (int k = 0; k < weights.size(); k++)
                    reply.addDouble(0.0);
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
    

    }
};  



int main(int argc, char *argv[])
{
    printf("hello");    
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



