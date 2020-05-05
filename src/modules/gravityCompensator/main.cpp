/**
@ingroup icub_module

\defgroup gravityCompensator gravityCompensator
 
Estimates the gravitational contribute to motors based on estimation of the robot dynamics
 
Copyright (C) 2008 RobotCub Consortium
 
Author: Matteo Fumagalli
 
Date: first release 24/07/2010 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

This module estimates the gravitational term acting on joints due to links weight.
The estimation is perfomed relying on rigid body dynamics using CAD 
parameters. 
For further information about the use of this module and of the iCub force control interface, please refer to the force control page:
http://wiki.icub.org/wiki/Force_Control

\section lib_sec Libraries 
- YARP libraries. 
- iDyn library.  

\section parameters_sec Parameters

--rate \e r 
- The parameter \e r identifies the rate the thread will work. If not
  specified \e 20ms is assumed. The minimum suggested rate is \e 10ms.

--no_legs
- This option disables the gravity compensation for the legs joints.

\section portsa_sec Ports Accessed
The port the service is listening to.

\section portsc_sec Ports Created
None.
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None. 
 
\section conf_file_sec Configuration Files
None
 
\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example
By launching the following command: 
 
\code 
gravityCompensator 
\endcode 
 
the module add offset values which are assigned to the IImpedanceControl interface and ITorqueControl interface
  
\author Matteo Fumagalli

This file can be edited at \in src/gravityCompensator/main.cpp.
*/ 

#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>

#include "gravityThread.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;



class gravityModuleCompensator: public RFModule
{
private:
    int rate{};
    gravityCompensatorThread *g_comp{};

    Property OptionsLeftArm;
    Property OptionsRightArm;
    Property OptionsHead;
    Property OptionsLeftLeg;
    Property OptionsRightLeg;
    Property OptionsTorso;

    Port rpcPort;

    PolyDriver *dd_left_arm;
    PolyDriver *dd_right_arm;
    PolyDriver *dd_head;
    PolyDriver *dd_left_leg;
    PolyDriver *dd_right_leg;
    PolyDriver *dd_torso;

    bool legs_enabled;
    bool right_arm_enabled;
    bool left_arm_enabled;
    bool torso_enabled;
    bool head_enabled;
    string m_side;
    string m_part;

public:
    gravityModuleCompensator()
    {
        legs_enabled  = true;
        left_arm_enabled  = true;
        right_arm_enabled  = true;
        torso_enabled  = true;
        head_enabled  = true;
        dd_left_arm   = nullptr;
        dd_right_arm  = nullptr;
        dd_head       = nullptr;
        dd_left_leg   = nullptr;
        dd_right_leg  = nullptr;
        dd_torso      = nullptr;
        m_side = "right";
        m_part = "arm";
    }

   virtual bool createDriver(PolyDriver *&_dd, Property options)
    {
        int trials=0;
        double start_time = yarp::os::Time::now();

        do
        {
            double current_time = yarp::os::Time::now();

            //remove previously existing drivers
            if (_dd)
            {
                delete _dd;
                _dd=nullptr;
            }

            //creates the new device driver
            _dd = new PolyDriver(options);
            bool connected =_dd->isValid();

            //check if the driver is connected
            if (connected) break;
        
            //check if the timeout (60s) is expired
            if (current_time-start_time > 60.0)
            {
                yError("It is not possible to instantiate the device driver. I tried %d times!\n", trials);
                return false;
            }

            yarp::os::Time::delay(5);
            trials++;
            yWarning("Unable to connect the device driver, trying again...\n");
        }
        while (true);

        IEncoders         *encs     = nullptr;
        IControlMode      *ctrlMode = nullptr;
        IImpedanceControl *imp      = nullptr;
        ITorqueControl    *tqs      = nullptr;

        bool ok = true;
        ok = ok & _dd->view(encs);
        ok = ok & _dd->view(ctrlMode);
        ok = ok & _dd->view(imp);
        ok = ok & _dd->view(tqs);
        if (!ok)
        {
            yError("One or more devices has not been viewed\nreturning...");
            return false;
        }

        return true;
    }

    bool configure(ResourceFinder &rf) override
    {
        string fwdSlash = "/";

        string name;
        name = "gravityCompensator";    

        if (rf.check("period"))
            rate = rf.find("period").asInt();
        else rate = 20;

        if (rf.check("rate"))
        {
            yError ("'rate' parameter is deprecated. Use 'period' instead");
            return false;
        }

        //-----------------GET THE ROBOT NAME-------------------//
        string robot_name;
        if (rf.check("robot"))
             robot_name = rf.find("robot").asString();
        else robot_name = "icub";

        //------------SPECIAL PARAM TP DEFINE THE HEAD TYPE-----//
        version_tag icub_type;
        if (rf.check("headV2"))
        {
            yInfo("'headV2' option found. Using icubV2 head kinematics.\n");
            icub_type.head_version = 2;
        }

        if (rf.check("headV2.6"))
        {
            yInfo("'headV2.6' option found. Using icubV2.6 head kinematics.\n");
            icub_type.head_version = 2;
            icub_type.head_subversion = 6;
        }

        //------------------CHECK IF LEGS ARE ENABLED-----------//
        if (rf.check("no_legs"))
        {
            legs_enabled= false;
            yInfo("'no_legs' option found. Legs will be disabled.\n");
        }
        //------------------CHECK IF ARMS ARE ENABLED-----------//
        if (rf.check("no_left_arm"))
        {
            left_arm_enabled= false;
            yInfo("'no_left_arm' option found. Left arm will be disabled.\n");
        }
        //------------------CHECK IF ARMS ARE ENABLED-----------//
        if (rf.check("no_right_arm"))
        {
            right_arm_enabled= false;
            yInfo("'no_right_arm' option found. Right arm will be disabled.\n");
        }
        //------------------CHECK IF TORSO IS ENABLED-----------//
        if (rf.check("no_torso_legs"))
        {
            torso_enabled= false;
            legs_enabled= false;
            yInfo("no_torso_legs' option found. Torso and legs will be disabled.\n");
        }
        if (rf.check("no_torso"))
        {
            torso_enabled= false;
            yInfo("'no_torso' option found. Torso will be disabled.\n");
        }
        //------------------CHECK IF HEAD IS ENABLED-----------//
        if (rf.check("no_head"))
        {
            head_enabled= false;
            yInfo("'no_head' option found. Head will be disabled.\n");
        }
        //---------------------DEVICES--------------------------//
        if (head_enabled)
        {
            OptionsHead.put("device","remote_controlboard");
            OptionsHead.put("local","/gravityCompensator/head/client");
            OptionsHead.put("remote","/"+robot_name+"/head");

            if (!createDriver(dd_head, OptionsHead))
            {
                yError("unable to create head device driver...quitting\n");
                return false;
            }
        }

        if (left_arm_enabled)
        {
            OptionsLeftArm.put("device","remote_controlboard");
            OptionsLeftArm.put("local","/gravityCompensator/left_arm/client");
            OptionsLeftArm.put("remote","/"+robot_name+"/left_arm");
            if (!createDriver(dd_left_arm,OptionsLeftArm))
            {
                yError("unable to create left arm device driver...quitting\n");
                return false;
            }
        }

        if (right_arm_enabled)
        {
            OptionsRightArm.put("device","remote_controlboard");
            OptionsRightArm.put("local","/gravityCompensator/right_arm/client");
            OptionsRightArm.put("remote","/"+robot_name+"/right_arm");
            if (!createDriver(dd_right_arm,OptionsRightArm))
            {
                yError("unable to create right arm device driver...quitting\n");
                return false;
            }
        }

        if (legs_enabled)
        {
            OptionsLeftLeg.put("device","remote_controlboard");
            OptionsLeftLeg.put("local","/gravityCompensator/left_leg/client");
            OptionsLeftLeg.put("remote","/"+robot_name+"/left_leg");
            if (!createDriver(dd_left_leg,OptionsLeftLeg))
            {
                yError("unable to create left leg device driver...quitting\n");
                return false;
            }

            OptionsRightLeg.put("device","remote_controlboard");
            OptionsRightLeg.put("local","/gravityCompensator/right_leg/client");
            OptionsRightLeg.put("remote","/"+robot_name+"/right_leg");
            if (!createDriver(dd_right_leg,OptionsRightLeg))
            {
                yError("unable to create right leg device driver...quitting\n");
                return false;
            }
        }
        
        if (torso_enabled)
        {
            OptionsTorso.put("device","remote_controlboard");
            OptionsTorso.put("local","/gravityCompensator/torso/client");
            OptionsTorso.put("remote","/"+robot_name+"/torso");

            if (!createDriver(dd_torso,OptionsTorso))
            {
                yError("unable to create torso device driver...quitting\n");
                return false;
            }
        }
        
        yInfo("device driver created\n");

        rpcPort.open("/"+name+"/rpc");
        attach(rpcPort);        

        //------------------CHECK FOR WHOLEBODYNAME -----------//
        std::string wholeBodyName = "wholeBodyDynamics";
        if (rf.check("wholebody_name"))
        {
            wholeBodyName = rf.find("wholebody_name").asString();
            yInfo("'wholeBodyName' option found. Using /%s prefix for connections.\n", wholeBodyName.c_str());
        }

        //------------------CHECK FOR INERTIAL -----------//
        bool inertial_enabled=true;
        if (rf.check("no_inertial"))
        {
            inertial_enabled=false;
            yInfo("'no_inertial' option found. Disabling inertial measurment.\n");
        }

        //--------------------------THREAD--------------------------

        g_comp = new gravityCompensatorThread(wholeBodyName, rate, dd_left_arm, dd_right_arm, dd_head, dd_left_leg, dd_right_leg, dd_torso, icub_type, inertial_enabled);
        yInfo("ft thread istantiated...\n");
        g_comp->start();
        yInfo("thread started\n");

        //------------------CHECK CONTROL OPTIONS -----------//
        if (rf.check("gravity_on"))
        {
            g_comp->gravity_mode = GRAVITY_COMPENSATION_ON;
        }
        if (rf.check("gravity_off"))
        {
            g_comp->gravity_mode = GRAVITY_COMPENSATION_OFF;
        }
        if (rf.check("external_on"))
        {
            g_comp->external_mode = EXTERNAL_TRQ_ON;
        }
        if (rf.check("external_on"))
        {
            g_comp->external_mode = EXTERNAL_TRQ_OFF;
        }
        if (g_comp->gravity_mode== GRAVITY_COMPENSATION_ON)       yInfo("gravity compensation on");
        else if (g_comp->gravity_mode== GRAVITY_COMPENSATION_OFF) yInfo("gravity compensation off");
        if (g_comp->external_mode==EXTERNAL_TRQ_ON)               yInfo("external input on");
        else if (g_comp->external_mode==EXTERNAL_TRQ_OFF)         yInfo("external input off");

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) override
    {
                        reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands:");
                reply.addString("calib all");

        Bottle position_bot;
        string helpMessage =  getName() +
                            " commands are: \n" +  
                            "help         to display this message\n" + 
                            "gravity_on   to enabl e the gravity compensation \n" + 
                            "gravity_off  to disbale the gravity compensation \n" +
                            "external_on  to enable the external input torque \n" +
                            "external_off to disable the external input torque \n";

          reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            cout << helpMessage;
            reply.addVocab(Vocab::encode("many"));
            reply.addString(" commands are: \n" );
            reply.addString(helpMessage.c_str());
        }
        else if (command.get(0).asString()=="gravity_on")
        {
            if (g_comp) 
            {
                g_comp->gravity_mode = GRAVITY_COMPENSATION_ON;
                reply.addString("gravity compensation on");
            }
        }
        else if (command.get(0).asString()=="gravity_off")
        {
            if (g_comp)
            {
                g_comp->gravity_mode = GRAVITY_COMPENSATION_OFF;
                reply.addString("gravity compensation off");
            }
        }
        else if (command.get(0).asString()=="external_on")
        {
            if (g_comp) 
            {
                g_comp->external_mode = EXTERNAL_TRQ_ON;
                reply.addString("external input on");
            }
        }
        else if (command.get(0).asString()=="external_off")
        {
            if (g_comp)
            {
                g_comp->external_mode = EXTERNAL_TRQ_OFF;
                reply.addString("external input off");
            }
        }
        else
        {
            reply.addString("unknown command. type help.");
        }
        
        return true;
    }


    bool close() override
    {
        //stop thread 
        if(g_comp)
        {
            g_comp->stop();
            delete g_comp; g_comp = nullptr;
        }
        
        //closing interfaces
        if (dd_left_arm)    {delete dd_left_arm;  dd_left_arm=nullptr;  }
        if (dd_right_arm)   {delete dd_right_arm; dd_right_arm=nullptr; }
        if (dd_left_leg)    {delete dd_left_leg;  dd_left_leg=nullptr;  }
        if (dd_right_leg)   {delete dd_right_leg; dd_right_leg=nullptr; }
        if (dd_head)        {delete dd_head;      dd_head=nullptr;      }
        if (dd_torso)       {delete dd_torso;     dd_torso=nullptr;     }

        //closing ports
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    double getPeriod() override { return 1.0;  }
    bool updateModule() override
    {
        static unsigned long int alive_counter = 0;
        static double curr_time = Time::now();
        if (Time::now() - curr_time > 60)
        {
            yInfo ("gravityCompensator is alive! running for %ld mins.\n",++alive_counter);
            curr_time = Time::now();
        }

        if (g_comp==nullptr) return false;

        thread_status_enum thread_status = g_comp->getThreadStatus();

        if (thread_status==STATUS_OK)
            return true;
        else if (thread_status==STATUS_DISCONNECTED)
        {
            yError("gravityCompensator module lost connection with iCubInterface, now closing...\n");
            return false;
        }
        else
        {
            yInfo("gravityCompensator module was closed successfully! \n");    
            return true;
        }
    }
};


int main(int argc, char * argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:";
        yInfo() << "--context context: where to find the called resource (referred to $ICUB_ROOT/app:)";
        yInfo() << "--from       from: the name of the file.ini to be used for calibration";
        yInfo() << "--rate       rate: the period used by the module. default 100ms (not less than 15ms)";
        yInfo() << "--no_legs    this option disables the gravity compensation for the legs joints" ;
        yInfo() << "--headV2     use the model of the headV2";
        yInfo() << "--no_left_arm      disables the left arm";
        yInfo() << "--no_right_arm     disables the right arm";
        yInfo() << "--no_legs          disables the legs";
        yInfo() << "--no_left_arm      disables the left arm";
        yInfo() << "--no_torso         disables the torso";
        yInfo() << "--no_torso_legs    disables the torso and the legs";
        yInfo() << "--no_head          disables the head";
        yInfo() << "--wholebody_name   the wholeBodyDyanmics port prefix (e.g. 'wholeBodyDynamics' / 'wholeBodyDynamicsTree')";
        yInfo() << "--no_inertial      disables the inertial";
        yInfo() << "--gravity_on       enables gravity compensation (default)";
        yInfo() << "--gravity_off      disables gravity compensation";
        yInfo() << "--external_on      enables external torque command (default)";
        yInfo() << "--external_off     disables external torque command";
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return 1;

    gravityModuleCompensator gcomp;

    return gcomp.runModule(rf);
}


