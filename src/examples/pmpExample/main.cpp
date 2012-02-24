/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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
\defgroup pmpExample pmpExample
 
@ingroup icub_module  
 
Example module for the use of \ref pmp "Pmp Library".

\section intro_sec Description 
This simple module steers the arm to a starting pose and then 
creates one pmp-target and one pmp-obstacle; thereby it enables 
the pmp field that in turn lets the robot end-effector attain 
the target avoiding the obstacle. 
 
It requires the \ref pmpServer running. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref pmp "Pmp" library.

\section parameters_sec Parameters 
--robot \e robot
- select the robot to connect to.

--part \e part
- select the part to control. 

--remote \e name
- specify the pmp server name to connect to.
 
--local \e name
- specify the pmp client stem-name.
 
--verbosity \e level
- specify the verbosity level of the pmp client print-outs.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ilaria Gori, Ugo Pattacini
*/ 

#include <string>
#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/ctrl/math.h>
#include <iCub/pmp/pmp_client.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::pmp;


class ClientModule: public RFModule
{
protected:
    PmpClient client;
    Property targetOptRx,ObsOptRx;
    bool init;
    bool closing;
    yarp::dev::PolyDriver         dCtrl;
    yarp::dev::ICartesianControl *iCtrl;
    int store_context_id;
    int target;
    int obstacle;
    Vector xTg;

public:

    bool configure(ResourceFinder &rf)
    {
        int verbosity=rf.check("verbosity",Value(0)).asInt();
        string remote=rf.check("remote",Value("/pmp_server")).asString().c_str();
        string local=rf.check("local",Value("/pmp_client")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        string part=rf.check("part",Value("left_arm")).asString().c_str();

        Property options;
        options.put("verbosity",verbosity);
        options.put("remote",remote.c_str());
        options.put("local",local.c_str());

        init=true;
        closing=false;

        Property optCtrl;
        optCtrl.put("device","cartesiancontrollerclient");
        optCtrl.put("remote",("/"+robot+"/cartesianController/"+part).c_str());
        optCtrl.put("local",("/"+local+"/cartesian").c_str());

        if (dCtrl.open(optCtrl))
            dCtrl.view(iCtrl);
        else
            return false;

        iCtrl->storeContext(&store_context_id);

        Vector dof;
        iCtrl->getDOF(dof);
        Vector newDof=dof;
        newDof[0]=1.0;
        newDof[2]=1.0;
        
        iCtrl->setDOF(newDof,dof);
        iCtrl->setLimits(7,-70.0,70.0);
        
        return client.open(options);
    }

    bool close()
    {
        if (!closing) 
        {
            client.disableControl();                
            client.disableField();
            
            iCtrl->restoreContext(store_context_id);
            dCtrl.close();

            client.clearItems();
        }

        client.close();
        return true;
    }

    bool updateModule()
    {
        if (init)
        {
            Vector x(3),o;
            x[0]=-0.2;
            x[1]=-0.05;
            x[2]=0.1;
            Matrix R(3,3); R=0.0;
            R(0,0)=-1.0; R(2,1)=-1.0; R(1,2)=-1.0;
            iCtrl->goToPoseSync(x,dcm2axis(R),3.0);
            iCtrl->waitMotionDone();
            iCtrl->getPose(x,o);
            iCtrl->setTrajTime(1.5);
            iCtrl->setInTargetTol(1e-3);

            xTg=x;
            xTg[0]-=0.2;
            Value centerTg; centerTg.fromString(("("+string(xTg.toString().c_str())+")").c_str());
            Value radiusTg; radiusTg.fromString("(0.01 0.01 0.01)");

            Property targetOpt;
            targetOpt.put("type","target_msd");
            targetOpt.put("active","on");
            targetOpt.put("K",1.5);
            targetOpt.put("D",3.0);
            targetOpt.put("name","target");
            targetOpt.put("center",centerTg);
            targetOpt.put("radius",radiusTg);
            client.addItem(targetOpt,target);

            Vector xOb=x;
            xOb[0]=(x[0]+xTg[0])/2.0;
            xOb[1]+=0.05;
            Value centerOb; centerOb.fromString(("("+string(xOb.toString().c_str())+")").c_str());
            Value radiusOb; radiusOb.fromString("(0.05 0.05 0.05)");

            Property obstacleOpt;
            obstacleOpt.put("type","obstacle_gaussian");
            obstacleOpt.put("active","on");
            obstacleOpt.put("G",5.0);
            obstacleOpt.put("name","obstacle");
            obstacleOpt.put("center",centerOb);
            obstacleOpt.put("radius",radiusOb);
            obstacleOpt.put("cut_tails","on");
            client.addItem(obstacleOpt,obstacle);

            client.setPointStateToTool();
            client.enableControl();
            client.enableField();

            init=false;
            return true;
        }
        else
        {
            Vector x,o,xdot,odot;
            client.getPointState(x,o,xdot,odot);
            double d1=norm(xTg-x);

            Vector xee,oee;
            iCtrl->getPose(xee,oee);
            double d2=norm(x-xee);

            fprintf(stdout,"|xTg-x|=%g [m], |x-xee|=%g[m]\n",d1,d2);

            if (d1<3e-3)
            {              
                client.disableControl();                
                client.disableField();
                
                iCtrl->restoreContext(store_context_id);

                dCtrl.close();
                client.clearItems();
                closing=true;

                return false;
            }
            else
                return true;
        }
    }

    double getPeriod()
    {
        return 0.1;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    ClientModule mod;
    return mod.runModule(rf);
}



