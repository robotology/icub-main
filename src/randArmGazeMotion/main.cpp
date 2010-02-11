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



class RandThread: public RateThread
{
private:
    string                     name;                 //local name

    double                     hand_freq;
    double                     gaze_limit;

    Vector                     head;                    //head position
    Vector                     min_target;
    Vector                     max_target;
    Vector                     hand_constraints;


    bool                       isHandIn;


    BufferedPort<Vector>       *port_arm;
    BufferedPort<Vector>       *port_gaze;

    ResourceFinder             &rf;




public:
     
    //Constructor
    RandThread(const string &_name, ResourceFinder &_rf, unsigned int period) : 
               RateThread(period*1000), name(_name), rf(_rf) {}

    virtual bool threadInit()
    {
        Bottle &bGeneral = rf.findGroup("general");
        if(bGeneral.isNull())
        {
            cout << "general part is missing!" << endl;
            return false;
        }

        //hand frequency
        if(bGeneral.check("hand_freq"))
            hand_freq = bGeneral.findGroup("hand_freq").get(1).asDouble();
        else
            hand_freq = 1.0;


        //head position information
        if(bGeneral.check("head_position"))
        {
            head.resize(3);
            Bottle &bHead = bGeneral.findGroup("head_position");
            if(bHead.size()-1 == 3)
                for(int i = 0; i < bHead.size()-1; i++)
                    head[i] = bHead.get(1+i).asDouble();
            else
            {
                cout << "option size != 3" << endl;
                return false;
            }
        }
        else
        {
            cout << "head position is missing!" << endl;
            return false;
        }



        string arm = bGeneral.check("arm",Value("right_arm")).asString().c_str();

        Bottle &bArm = rf.findGroup(arm.c_str());
        if(bArm.isNull())
        {
            cout << arm << " is missing!" << endl;
            return false;
        }
        
        //gaze limit
        if(bArm.check("gaze_limit"))
            gaze_limit = bArm.findGroup("gaze_limit").get(1).asDouble();
        else
        {
            cout << "gaze limit is missing!" << endl;
            return false;
        }


        //hand constraints
        if(bArm.check("hand_constraints"))
        {
            hand_constraints.resize(3);
            Bottle &bHandConstraints = bArm.findGroup("hand_constraints");
            if(bHandConstraints.size()-1 == 3)
            {
                for(int i = 0; i < bHandConstraints.size()-1; i++)
                    hand_constraints[i] = bHandConstraints.get(1+i).asDouble();
            }
            else
            {
                cout << "option size != 3" << endl;
                return false;
            }
        }
        else
        {
            cout << "hand constraints are missing!" << endl;
            return false;
        }


        //max target limit
        if(bArm.check("max_target"))
        {
            max_target.resize(3);
            Bottle &bMaxTarget = bArm.findGroup("max_target");
            if(bMaxTarget.size()-1 == 3)
            {
                for(int i = 0; i < bMaxTarget.size()-1; i++)
                    max_target[i] = bMaxTarget.get(1+i).asDouble();
            }
            else
            {
                cout << "option size != 3" << endl;
                return false;
            }
        }
        else
        {
            cout << "max target limit is missing!" << endl;
            return false;
        }

        //min target limit
        if(bArm.check("min_target"))
        {
            min_target.resize(3);
            Bottle &bMinTarget = bArm.findGroup("min_target");
            if(bMinTarget.size()-1 == 3)
            {
                for(int i = 0; i < bMinTarget.size()-1; i++)
                    min_target[i] = bMinTarget.get(1+i).asDouble();
            }
            else
            {
                cout << "option size != 3" << endl;
                return false;
            }
        }
        else
        {
            cout << "min target limit is missing!" << endl;
            return false;
        }




        //open ports
        string localArmName = name + "/" + arm;
        string localGazeName = name + "/head";

        port_arm = new BufferedPort<Vector>;
        port_arm->open(localArmName.c_str());

        port_gaze = new BufferedPort<Vector>;
        port_gaze->open(localGazeName.c_str());


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
        Vector target = math::Rand::vector(min_target,max_target);

        cout <<"\n\n----------------------------------------------" << endl;
        cout << "          Target point coordinates: " << endl;
        cout << "     ( " << target[0] << " , " << target[1] << " , " << target[2] << ")" << endl;

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
        if(!isHandIn && abs(gaze[2]) > abs(gaze_limit))
            gaze[2] = gaze_limit;


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
            Vector orient_axis_x = ctrl::cross(orient_axis_z,hand_0);
            Vector orient_axis_y = ctrl::cross(orient_axis_z,orient_axis_x);

            //normalize the axis to get the new basis
            double norm_x = ctrl::norm(orient_axis_x);
            double norm_y = ctrl::norm(orient_axis_y);
            double norm_z = ctrl::norm(orient_axis_z);

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
            hand[0] = hand_constraints[0];
            hand[1] = hand_constraints[1];
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
        int period;

        Time::turboBoost();
        yarp::math::Rand::init();


        Bottle &general = rf.findGroup("general");
        if(!general.isNull())
            period = rf.check("period",Value((int) 20)).asInt();
        else
        {
            cout << "general part is missing!" << endl;
            return false;
        }


        rnd = new RandThread(getName().c_str(),rf,period);
        if(!rnd->start())
        {
            delete rnd;
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

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

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("randArmGazeMotion/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    RandModule mod;
    mod.setName("/randArmGazeMotion");

    return mod.runModule(rf);
}

