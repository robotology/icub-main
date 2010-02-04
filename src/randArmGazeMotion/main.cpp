/** 
\defgroup randArmGazeMotion
 
@ingroup icub_module  
 
A module that keeps the gaze focused on the hand 
while performing random movements with the arm
 
Copyright (C) 2009 RobotCub Consortium
 
Author: Carlo Ciliberto 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module repeatedly generates a random 3-d point inside a bounded box
and send commands to the cartesian controllers to focus the gaze toward it
and reach it with the arm while keeping the hand's back constanly oriented
toward the robot's cameras.
 
\section lib_sec Libraries 
- YARP libraries.
- ctrlLib
 
\section parameters_sec Parameters
--genName \e name 
- The parameter \e name identifies the module's name; all 
  the open ports will be tagged with the prefix
  /<genName>/<part>/. If not specified \e randArmGazeMotion is
  assumed.
 
--arm \e type 
- The parameter \e type selects the robot's arm to work with. It
  can be \e right_arm or \e left_arm; if not specified
  \e right_arm is assumed.

--handFreq \e frequency 
- The parameter \e frequency selects the frequency by which the module
  will command the arm controller to reach the randomly generated target.
  It can vary between 0.0 and 1.0. If not specified 1.0 (constant presence) is assumed.
 
--T \e time
- specify the interval between the generation of two random points
  in seconds if not specified \e time is 20.0 seconds.
 
\section portsa_sec Ports Accessed
None.
 
\section portsc_sec Ports Created 
 
- \e /<genName>/<arm> sends out commands to the arm controller
 
- \e /<genName>/head sends out commands to the gaze controller

- \e /<genName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
    -'susp' suspend the module
    -'run' resume the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Windows

\author Carlo Ciliberto
*/ 




#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <iCub/ctrlMath.h>
#include <yarp/math/Rand.h>

#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;



//Limits for the random target point
#define X_MIN -0.3
#define X_MAX -0.2
#define Y_MIN -0.15
#define Y_MAX 0.3
#define Z_MIN 0.1
#define Z_MAX 0.3

//Head position
#define HEAD_X 0.0
#define HEAD_Y 0.0
#define HEAD_Z 0.4



class RandThread: public RateThread
{
private:
    string                     genName;                 //local name
    string                     arm;                     //arm type

    Vector                     head;                    //head position
    Vector                     target_min;
    Vector                     target_max;


    double                     hand_freq;

    bool                       isHandIn;


    BufferedPort<Vector>       *port_arm;
    BufferedPort<Vector>       *port_gaze;



    Vector cross(const Vector &a, const Vector &b)
    {
        Vector c(3);

        if(a.size() != 3 || b.size() != 3)
        {
            cout << "Error! vectors not of size 3 for the cross product" << endl;
            return c;
        }

        c[0] = a[1]*b[2] - a[2]*b[1];
        c[1] = a[2]*b[0] - a[0]*b[2];
        c[2] = a[0]*b[1] - a[1]*b[0];

        return c;
}


public:
     
    //Constructor
    RandThread(unsigned int _period, string &_genName, string &_arm, double &_hand_freq)
        :RateThread(_period*1000),genName(_genName), arm(_arm),hand_freq(_hand_freq){}


    virtual bool threadInit()
    {
        string localArmName = "/" + genName + "/" + arm;
        string localGazeName = "/" + genName + "/head";

        //open ports
        port_arm = new BufferedPort<Vector>;
        port_arm->open(localArmName.c_str());

        port_gaze = new BufferedPort<Vector>;
        port_gaze->open(localGazeName.c_str());
        
        //Head position
        head.resize(3);
        head[0] = HEAD_X;
        head[1] = HEAD_Y;
        head[2] = HEAD_Z;

        //Bounds of the box for the random targets
        target_min.resize(3);
        target_min[0] = X_MIN;
        target_min[1] = Y_MIN;
        target_min[2] = Z_MIN;

        target_max.resize(3);
        target_max[0] = X_MAX;
        target_max[1] = Y_MAX;
        target_max[2] = Z_MAX;

        return true;
    }

    

    virtual void run()
    {
        //Generate the random target point
        Vector target = generateTarget();

        //Send the command to the gaze ctrl
        lookAt(target);

        //Send the command to the arm ctrl
        reach(target);
    }


    virtual void threadRelease()
    {
        port_arm->interrupt();
        port_arm->close();
        delete port_arm;
 
        port_gaze->interrupt();
        port_gaze->close();
        delete port_gaze;
    }


    Vector generateTarget()
    {
        //decide if the hand will be in the scene or not
        isHandIn = (math::Rand::scalar() < hand_freq)? true: false;
        
        //Generate the random target point.
        Vector target = math::Rand::vector(target_min,target_max);

        cout <<"\n\n----------------------------------------------" << endl;
        cout << "          Target point coordinates: " ;
        cout << "( " << target[0] << " , " << target[1] << " , " << target[2] << ")" << endl;

        return target;
    }



    void lookAt(const Vector &target)
    {
       
        if(target.size() != 3)
        {
            cout << "Error! wrong size for gaze target point" << endl;
            return;
        }


        Vector &gaze = port_gaze->prepare();
        gaze.resize(3);
        for(int i = 0; i < 3; i++)
            gaze[i] = target[i];

        //keep the gaze out of the hand's way
        if(isHandIn && gaze[2] > 0.08)
            gaze[2] = 0.08;


        port_gaze->write();

    }


    void reach(const Vector &target)
{
        if(target.size() != 3)
        {
            cout << "Error! wrong size for arm target point" << endl;
            return;
        }


        Vector &hand = port_arm->prepare();
        hand.resize(7);

        if(isHandIn)
        {
            Vector hand2head(3);
            Vector hand_0(3);


            for(int i = 0; i < 3; i++)
            {
                hand[i] = target[i];
                hand2head[i] = hand[i] - head[i];
                hand_0[i] = -hand[i];
            }
        

            //orient the hand so that the palm is directed toward the eyes
            Vector orient_axis_z = hand2head;
            Vector orient_axis_x = cross(orient_axis_z,hand_0);
            Vector orient_axis_y = cross(orient_axis_x,orient_axis_z);

            //normalize the axis to get the new basis
            double norm_x = sqrt(yarp::math::dot(orient_axis_x,orient_axis_x));
            double norm_y = sqrt(yarp::math::dot(orient_axis_y,orient_axis_y));
            double norm_z = sqrt(yarp::math::dot(orient_axis_z,orient_axis_z));

            for(int i = 0; i < 3; i++)
            {
                orient_axis_x[i] /= norm_x;
                orient_axis_y[i] /= norm_y;
                orient_axis_z[i] /= norm_z;
            }



            //put the vector in column in the rotation matrix
            Matrix R(3,3);
            //R.setCol(0,orient_axis_x);
            //R.setCol(1,orient_axis_y);
            //R.setCol(2,orient_axis_z);

            //temporary
            for(int i = 0; i < 3; i++)
            {
                R(i,0) = orient_axis_x[i];
                R(i,1) = orient_axis_y[i];
                R(i,2) = orient_axis_z[i];
            }



            Vector orient = ctrl::dcm2axis(R);
            for(int i = 0; i < 4; i++)
                hand[i+3] = orient[i];
        }
        else
        {
            hand[0] = -0.21;
            hand[1] = 0.36;
            hand[2] = yarp::math::Rand::scalar()*0.3;
        }

        port_arm->write();
    }

};









// Usual YARP stuff...
class RandModule: public RFModule
{
protected:
    RandThread       *rnd;
    Port             rpcPort;

public:
    RandModule() { }

    virtual bool configure(ResourceFinder &rf)
    {
        string genName;
        string arm;
        unsigned int period;
        double handPresence;

        Time::turboBoost();
        yarp::math::Rand::init();




        if (rf.check("genName"))
            genName=rf.find("genName").asString();
        else
            genName="randArmGazeMotion";

        if (rf.check("arm"))
            arm=rf.find("arm").asString();
        else
            arm="right_arm";
        
        if (rf.check("T"))
            period=rf.find("T").asInt();
        else
            period=20;
        
        if (rf.check("handPresence"))
            handPresence = rf.find("handPresence").asDouble();
        else
            handPresence = 1.0;






        rnd = new RandThread(period,genName,arm,handPresence);
        rnd->start();

        string rpcPortName= "/" + genName +"/rpc";
        rpcPort.open(rpcPortName.c_str());
        attach(rpcPort);
        attachTerminal();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        cout << "Receiving command from rpc port" << endl;

        if (command.size())
        {
            switch (command.get(0).asVocab())
            {
                case VOCAB4('s','u','s','p'):
                {                    
                    rnd->suspend();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
        
                case VOCAB3('r','u','n'):
                {                    
                    rnd->resume();
                    reply.addVocab(Vocab::encode("ack"));
                    return true;
                }
        
                default:
                    return RFModule::respond(command,reply);
            }
        }
        else
        {
            reply.add("command size==0");
            return false;
        }
    }

    virtual bool close()
    {
        rnd->stop();

        delete rnd;


        rpcPort.interrupt();
        rpcPort.close();

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
        cout << "\t--genName        name: module name (default randArmGazeMotion)"                           << endl;
        cout << "\t--arm            type: robot arm type, left_arm or right_arm (default: right_arm)"                         << endl;
        cout << "\t--handFreq       frequency: frequency of hand presence in the Fov (default: 1)"    << endl;
        cout << "\t--T              time: interval between movements in sec (default: 30)"    << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    RandModule mod;

    return mod.runModule(rf);
}

