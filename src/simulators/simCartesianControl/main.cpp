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
\defgroup simCartesianControl simCartesianControl
 
@ingroup icub_module  
 
A simple module that makes the Cartesian Interface available 
with the robot simulator. 
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module allows to integrate the Cartesian Interface with the
the robot simulator. 
 
\section lib_sec Dependencies
- YARP libraries. 
- The \ref iKin library. 
- The \ref servercartesiancontroller module. 
 
\section usage_sec Usage 
Follow this steps: 
-# Launch the \ref icub_Simulation "iCub Simulator".
-# Launch the \ref simCartesianControl module.
-# Launch the \ref iKinCartesianSolver "Cartesian Solvers" for 
   the required limbs: have a look to the template located in
   the directory <i>
   icub-main/app/simCartesianControl/scripts </i>.

\section parameters_sec Parameters
--robot \e name 
- specifies the simulated robot name to connect to. 
 
--no_legs 
- disable the control of robot's legs. 
 
Other options are available but their default values should be 
fine for normal use. If you are really curious then get into the
short code :) 

\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <iostream>
#include <iomanip>
#include <string>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;


/************************************************************************/
class SimCartCtrlModule: public RFModule
{
protected:
    PolyDriver torso;
    PolyDriver armR,       armL;
    PolyDriver legR,       legL;
    PolyDriver serverArmR, serverArmL;
    PolyDriver serverLegR, serverLegL;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        bool noLegs=rf.check("no_legs");

        Property optTorso("(device remote_controlboard)");
        Property optArmR("(device remote_controlboard)");
        Property optArmL("(device remote_controlboard)");
        Property optLegR("(device remote_controlboard)");
        Property optLegL("(device remote_controlboard)");

        string robot=rf.find("robot").asString().c_str();
        optTorso.put("remote",("/"+robot+"/torso").c_str());
        optArmR.put("remote",("/"+robot+"/right_arm").c_str());
        optArmL.put("remote",("/"+robot+"/left_arm").c_str());
        optLegR.put("remote",("/"+robot+"/right_leg").c_str());
        optLegL.put("remote",("/"+robot+"/left_leg").c_str());

        string local=rf.find("local").asString().c_str();
        optTorso.put("local",("/"+local+"/torso").c_str());
        optArmR.put("local",("/"+local+"/right_arm").c_str());
        optArmL.put("local",("/"+local+"/left_arm").c_str());
        optLegR.put("local",("/"+local+"/right_leg").c_str());
        optLegL.put("local",("/"+local+"/left_leg").c_str());

        optTorso.put("writeStrict","on");
        optArmR.put("writeStrict","on");
        optArmL.put("writeStrict","on");
        optLegR.put("writeStrict","on");
        optLegL.put("writeStrict","on");

        if (!torso.open(optTorso) || !armR.open(optArmR) || !armL.open(optArmL))
        {
            cout<<"Device drivers not available!"<<endl;
            close();

            return false;
        }

        if (!noLegs)
        {
            if (!legR.open(optLegR) || !legL.open(optLegL))
            {
                cout<<"Device drivers not available!"<<endl;
                close();

                return false;
            }
        }

        PolyDriverList listArmR, listArmL, listLegR, listLegL;
        listArmR.push(&torso,"torso");
        listArmR.push(&armR,"right_arm");
        listArmL.push(&torso,"torso");
        listArmL.push(&armL,"left_arm");
        listLegR.push(&legR,"right_leg");
        listLegL.push(&legL,"left_leg");

        Property optServerArmR("(device cartesiancontrollerserver)");
        Property optServerArmL("(device cartesiancontrollerserver)");
        Property optServerLegR("(device cartesiancontrollerserver)");
        Property optServerLegL("(device cartesiancontrollerserver)");
        optServerArmR.fromConfigFile(rf.findFile("right_arm_file"),false);
        optServerArmL.fromConfigFile(rf.findFile("left_arm_file"),false);
        optServerLegR.fromConfigFile(rf.findFile("right_leg_file"),false);
        optServerLegL.fromConfigFile(rf.findFile("left_leg_file"),false);

        IMultipleWrapper *wrapperArmR, *wrapperArmL, *wrapperLegR, *wrapperLegL;
        if (!serverArmR.open(optServerArmR) || !serverArmL.open(optServerArmL))
        {
            close();
            return false;
        }

        serverArmR.view(wrapperArmR); serverArmL.view(wrapperArmL);
        if (!wrapperArmR->attachAll(listArmR) || !wrapperArmL->attachAll(listArmL))
        {
            close();    
            return false;
        }

        if (!noLegs)
        {
            if (!serverLegR.open(optServerLegR) || !serverLegL.open(optServerLegL))
            {
                close();
                return false;
            }

            serverLegR.view(wrapperLegR); serverLegL.view(wrapperLegL);
            if (!wrapperLegR->attachAll(listLegR) || !wrapperLegL->attachAll(listLegL))
            {
                close();    
                return false;
            }
        }

        return true;
    }

    /************************************************************************/
    bool close()
    {
        if (serverArmR.isValid())
            serverArmR.close();

        if (serverArmL.isValid())
            serverArmL.close();

        if (serverLegR.isValid())
            serverLegR.close();

        if (serverLegL.isValid())
            serverLegL.close();

        if (torso.isValid())
            torso.close();

        if (armR.isValid())
            armR.close();

        if (armL.isValid())
            armL.close();

        if (legR.isValid())
            legR.close();

        if (legL.isValid())
            legL.close();

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("simCartesianControl");
    rf.setDefault("robot","icubSim");
    rf.setDefault("local","simCartesianControl");
    rf.setDefault("right_arm_file","cartesianRightArm.ini");
    rf.setDefault("left_arm_file","cartesianLeftArm.ini");
    rf.setDefault("right_leg_file","cartesianRightLeg.ini");
    rf.setDefault("left_leg_file","cartesianLeftLeg.ini");
    rf.configure(argc,argv);

    Network yarp;
    if (!yarp.checkNetwork())
    {
        cout<<"YARP server not available!"<<endl;
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    SimCartCtrlModule mod;
    return mod.runModule(rf);
}



