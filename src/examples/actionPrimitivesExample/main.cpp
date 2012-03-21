/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
\defgroup ationPrimitivesExample actionPrimitivesExample
 
@ingroup icub_module  
 
Example of grasping module based upon \ref ActionPrimitives 
library. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0. 

\section intro_sec Description 
An example module that makes use of \ref ActionPrimitives 
library in order to execute a sequence of simple actions: 
reaches for an object, tries to grasp it, then lifts it and 
finally releases it. 
 
1) A bottle containing the 3-d position of the object to grasp 
is received (let be x1).
 
2) To this 3-d point a systematic offset is added in order to
compensate for the uncalibrated kinematics of the arm (let be
x2=x1+systematic_offset). 
 
3) The robot reaches for a position located on top of the object 
with the specified orientation (let be reach(x2+grasp_disp,o)). 
 
4) The robot grasps the object (let be reach(x2,o)). 
 
5) The hand is closed. 
 
6) The robot lifts the object to a specified location (let be 
reach(x2+lift_displacement,o)). 
 
7) The robot releases the grasped object. 
 
8) The robot steers the arm to home position. 
 
\note A video on iCub grasping objects can be seen <a 
    href="http://eris.liralab.it/misc/icubvideos/icub_grasps_sponges.wmv">here</a>.
  
\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library.  

\section parameters_sec Parameters
--name \e name
- specify the module name, which is \e ActionPrimitivesMod by
  default.
 
--part \e type 
- specify which arm has to be used: type can be \e left_arm, \e 
  right_arm.
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/in receives a bottle containing the 3-d position 
  of the object to grasp.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files 
--grasp_model_type \e type 
- specify the grasp model type according to the \ref 
  ActionPrimitives documentation.
 
--grasp_model_file \e file 
- specify the path to the file containing the grasp model 
  options.
 
--hand_sequences_file \e file 
- specify the path to the file containing the hand motion 
  sequences relative to the current context ( \ref
  ActionPrimitives ).
 
--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context).
 
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general]
// options used to open a ActionPrimitives object 
robot                           icub
thread_period                   50
default_exec_time               3.0
reach_tol                       0.007
verbosity                       on 
torso_pitch                     on
torso_roll                      off
torso_yaw                       on
torso_pitch_max                 30.0 
tracking_mode                   off 
verbosity                       on 
 
[arm_dependent]
grasp_orientation               (-0.171542 0.124396 -0.977292 3.058211)
grasp_displacement              (0.0 0.0 0.05)
systematic_error_displacement   (-0.03 -0.07 -0.02)
lifting_displacement            (0.0 0.0 0.2)
home_position                   (-0.29 -0.21 0.11)
home_orientation                (-0.029976 0.763076 -0.645613 2.884471) 
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <iCub/perception/models.h>
#include <iCub/action/actionPrimitives.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>

#define USE_LEFT    0
#define USE_RIGHT   1

#define AFFACTIONPRIMITIVESLAYER    ActionPrimitivesLayer1

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::perception;
using namespace iCub::action;


/************************************************************************/
class ExampleModule: public RFModule
{
protected:
    AFFACTIONPRIMITIVESLAYER *action;
    BufferedPort<Bottle>      inPort;
    Port                      rpcPort;

    Vector graspOrien;
    Vector graspDisp;
    Vector dOffs;
    Vector dLift;
    Vector home_x;

    bool openPorts;
    bool firstRun;

public:
    /************************************************************************/
    ExampleModule()
    {
        graspOrien.resize(4);
        graspDisp.resize(3);
        dOffs.resize(3);
        dLift.resize(3);
        home_x.resize(3);

        // default values for arm-dependent quantities
        graspOrien[0]=-0.171542;
        graspOrien[1]= 0.124396;
        graspOrien[2]=-0.977292;
        graspOrien[3]= 3.058211;

        graspDisp[0]=0.0;
        graspDisp[1]=0.0;
        graspDisp[2]=0.05;

        dOffs[0]=-0.03;
        dOffs[1]=-0.07;
        dOffs[2]=-0.02;

        dLift[0]=0.0;
        dLift[1]=0.0;
        dLift[2]=0.15;
        
        home_x[0]=-0.29;
        home_x[1]=-0.21;
        home_x[2]= 0.11;

        action=NULL;

        openPorts=false;
        firstRun=true;
    }

    /************************************************************************/
    void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
                                Vector &_dOffs, Vector &_dLift, Vector &_home_x)
    {
        if (Bottle *pB=b.find("grasp_orientation").asList())
        {
            int sz=pB->size();
            int len=_gOrien.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gOrien[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("systematic_error_displacement").asList())
        {
            int sz=pB->size();
            int len=_dOffs.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dOffs[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("lifting_displacement").asList())
        {
            int sz=pB->size();
            int len=_dLift.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dLift[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("home_position").asList())
        {
            int sz=pB->size();
            int len=_home_x.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _home_x[i]=pB->get(i).asDouble();
        }
    }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        string partUsed=rf.find("part").asString().c_str();
        if ((partUsed!="left_arm") && (partUsed!="right_arm"))
        {
            cout<<"Invalid part requested!"<<endl;
            return false;
        }        

        Property config; config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;
            return false;
        }

        // parsing general config options
        Property option(bGeneral.toString().c_str());
        option.put("local",name.c_str());
        option.put("part",rf.find("part").asString().c_str());
        option.put("grasp_model_type",rf.find("grasp_model_type").asString().c_str());
        option.put("grasp_model_file",rf.findFile("grasp_model_file").c_str());
        option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());        

        // parsing arm dependent config options
        Bottle &bArm=config.findGroup("arm_dependent");
        getArmDependentOptions(bArm,graspOrien,graspDisp,dOffs,dLift,home_x);

        cout<<"***** Instantiating primitives for "<<partUsed<<endl;
        action=new AFFACTIONPRIMITIVESLAYER(option);
        if (!action->isValid())
        {
            delete action;
            return false;
        }
        
        deque<string> q=action->getHandSeqList();
        cout<<"***** List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
            cout<<q[i]<<endl;

        string fwslash="/";
        inPort.open((fwslash+name+"/in").c_str());
        rpcPort.open((fwslash+name+"/rpc").c_str());
        attach(rpcPort);

        openPorts=true;

        // check whether the grasp model is calibrated,
        // otherwise calibrate it and save the results
        Model *model; action->getGraspModel(model);
        if (model!=NULL)
        {
            if (!model->isCalibrated())
            {
                Property prop("(finger all)");
                model->calibrate(prop);

                ofstream fout;
                fout.open(option.find("grasp_model_file").asString().c_str());
                model->toStream(fout);
                fout.close();
            }
        }

        return true;
    }

    /************************************************************************/
    bool close()
    {
        if (action!=NULL)
            delete action;

        if (openPorts)
        {
            inPort.close();
            rpcPort.close();
        }

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /************************************************************************/
    void init()
    {
        bool f;

        action->pushAction(home_x,"open_hand");
        action->checkActionsDone(f,true);
        action->enableArmWaving(home_x);
    }

    // we don't need a thread since the actions library already
    // incapsulates one inside dealing with all the tight time constraints
    /************************************************************************/
    bool updateModule()
    {
        // do it only once
        if (firstRun)
        {
            init();
            firstRun=false;
        }

        // get a target object position from a YARP port
        Bottle *b=inPort.read();    // blocking call

        if (b!=NULL)
        {
            Vector xd(3);
            bool f;

            xd[0]=b->get(0).asDouble();
            xd[1]=b->get(1).asDouble();
            xd[2]=b->get(2).asDouble();

            // apply systematic offset
            // due to uncalibrated kinematic
            xd=xd+dOffs;

            // safe thresholding
            xd[0]=xd[0]>-0.1?-0.1:xd[0];

            // grasp (wait until it's done)
            action->grasp(xd,graspOrien,graspDisp);
            action->checkActionsDone(f,true);
            action->areFingersInPosition(f);

            // if fingers are not in position,
            // it's likely that we've just grasped
            // something, so lift it up!
            if (!f)
            {
                cout<<"Wow, got something!"<<endl;

                // lift the object (wait until it's done)
                action->pushAction(xd+dLift,graspOrien);
                action->checkActionsDone(f,true);
            }
            else
                cout<<"Sorry :( ... nothing to grasp"<<endl;

            // release the object or just open the
            // hand (wait until it's done)
            action->pushAction("open_hand");
            action->checkActionsDone(f,true);

            // go home :) (wait until it's done, since
            // we may use two arms that share the torso)
            action->pushAction(home_x);
            action->checkActionsDone(f,true);

            // let the hand wave a bit around home position
            // the waving will be disabled before commencing
            // a new action
            action->enableArmWaving(home_x);
        }

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        // since a call to checkActionsDone() blocks
        // the execution until it's done, we need to 
        // take control and exit from the waiting state
        action->syncCheckInterrupt(true);        

        inPort.interrupt();
        rpcPort.interrupt();

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("actionPrimitivesExample/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("part","left_arm");
    rf.setDefault("grasp_model_type","springy");
    rf.setDefault("grasp_model_file","grasp_model.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");
    rf.setDefault("name","actionPrimitivesMod");
    rf.configure("ICUB_ROOT",argc,argv);

    ExampleModule mod;
    return mod.runModule(rf);
}



