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
\defgroup iKinCartesianSolver iKinCartesianSolver 
 
@ingroup icub_module  
 
Just a container which runs the \ref iKinSlv "Cartesian Solver" 
taking parameters from a configuration file. 
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description 
See \ref iKinSlv "Cartesian Solver" for documentation. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT: see the 
  <a
  href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>).

\section parameters_sec Parameters
--part \e type [mandatory]
- select the part to run. \e type is the group name within the 
  configuration file (e.g. left_arm, right_arm, ...).

--context \e directory [optional]
- allow specifying a different path where to search the 
  configuration file.
 
--from \e file [optional]
- allow specifying a different configuration file from the 
  default one which is \e cartesianSolver.ini.
 
\section portsa_sec Ports Accessed
 
All ports which allow the access to motor interface shall be 
previously open. 

\section portsc_sec Ports Created 
 
- /<solverName>/in : for requests in streaming mode.
- /<solverName>/rpc : for requests and replies.
- /<solverName>/out : for output streaming. 
 
\note for a detailed description, see \ref iKinSlv.
 
\section conf_file_sec Configuration Files
 
Here's how the configuration file will look like for the 
specific icub part left_arm: 
 
\code 
[left_arm]
robot          icub
name           cartesianSolver/left_arm
type           left
period         20
dof            (0 0 0 1 1 1 1 1 1 1)
rest_pos       (0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0)
rest_weights   (1.0 1.0 1.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0) 
pose           full
mode           shot
verbosity      off
maxIter        200
tol            0.001
xyzTol         0.000001
interPoints    off 
ping_robot_tmo 20.0 
\endcode 
 
Here's how the configuration file will look like for a custom 
robot part: 
 
\code 
[left_arm]
robot           my-robot
name            cartesianSolver/left_arm 
period          20
dof             (0 0 0 1 1 1 1)
rest_pos        (0.0 0.0 0.0 0.0 0.0 0.0 0.0)
rest_weights    (1.0 1.0 1.0 0.0 0.0 0.0 0.0) 
pose            full
mode            shot
verbosity       off
maxIter         200
tol             0.001
xyzTol          0.000001
interPoints     off 
ping_robot_tmo  20.0 
 
CustomKinFile   cartesian/kinematics.ini 
NumberOfDrivers 2 
driver_0        (Key torso)    (JointsOrder reversed) 
driver_1        (Key left_arm) (JointsOrder direct) 
\endcode 
 
\note for a detailed description of options, see \ref iKinSlv
      "Cartesian Solver".
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <iCub/iKin/iKinSlv.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::iKin;

static string pathToCustomKinFile;


/************************************************************************/
class CustomCartesianSolver : public CartesianSolver
{
protected:
    /************************************************************************/
    PartDescriptor *getPartDesc(Searchable &options)
    {
        if (!options.check("robot"))
        {
            cout<<"Error: \"robot\" option is missing!"<<endl;
            return NULL;
        }

        if (!options.check("NumberOfDrivers"))
        {
            cout<<"Error: \"NumberOfDrivers\" option is missing!"<<endl;
            return NULL;
        }

        string robot=options.find("robot").asString().c_str();
        cout<<"Configuring solver for "<<robot<<" ..."<<endl;

        Property linksOptions;
        linksOptions.fromConfigFile(pathToCustomKinFile.c_str());
        iKinLimb *limb=new iKinLimb(linksOptions);
        if (!limb->isValid())
        {
            cout<<"Error: invalid links parameters!"<<endl;
            delete limb;
            return NULL;
        }

        PartDescriptor *p=new PartDescriptor;
        p->lmb=limb;
        p->chn=limb->asChain();
        p->cns=NULL;

        bool failure=false;
        p->num=options.find("NumberOfDrivers").asInt();
        for (int cnt=0; cnt<p->num; cnt++)
        {
            ostringstream str;
            str<<"driver_"<<cnt;
            Bottle &driver=options.findGroup(str.str().c_str());
            if (driver.isNull())
            {
                cout<<"Error: \""<<str.str()<<"\" option is missing!"<<endl;
                failure=true;
                break;
            }

            if (!driver.check("Key"))
            {
                cout<<"Error: \"Key\" option is missing!"<<endl;
                failure=true;
                break;
            }

            if (!driver.check("JointsOrder"))
            {
                cout<<"Error: \"JointsOrder\" option is missing!"<<endl;
                failure=true;
                break;
            }

            string part=driver.find("Key").asString().c_str();
            bool directOrder=(driver.find("JointsOrder").asString()=="direct");

            Property optPart;
            optPart.put("device","remote_controlboard");
            optPart.put("remote",("/"+robot+"/"+part).c_str());
            optPart.put("local",("/"+slvName+"/"+part).c_str());
            optPart.put("robot",robot.c_str());
            optPart.put("part",part.c_str());
            p->prp.push_back(optPart);
            p->rvs.push_back(directOrder);
        }

        if (failure)
        {
            delete limb;
            delete p;
            return NULL;
        }
        else
            return p;
    }

public:
    /************************************************************************/
    CustomCartesianSolver(const string &name) : CartesianSolver(name) { }
};



/************************************************************************/
class SolverModule: public RFModule
{
protected:
    CartesianSolver *slv;

public:
    /************************************************************************/
    SolverModule()
    {
        slv=NULL;
    }

    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {                
        string part, slvName;
        if (rf.check("part"))
            part=rf.find("part").asString().c_str();
        else
        {
            cout<<"Error: part option is not specified"<<endl;
            return false;
        }

        Bottle &group=rf.findGroup(part.c_str());
        if (group.isNull())
        {
            cout<<"Error: unable to locate "<<part<<" definition"<<endl;
            return false;
        }

        if (group.check("name"))
            slvName=group.find("name").asString().c_str();
        else
        {
            cout<<"Error: name option is missing"<<endl;
            return false;
        }

        if (group.check("CustomKinFile"))
        {
            cout<<"Custom Cartesian Solver detected!"<<endl;

            ResourceFinder rf_kin;
            rf_kin.setVerbose(true);
            rf_kin.setDefaultContext(rf.getContext().c_str());
            rf_kin.configure(0,NULL);
            pathToCustomKinFile=rf_kin.findFileByName(group.find("CustomKinFile").asString()).c_str();

            slv=new CustomCartesianSolver(slvName);
        }
        else if ((part=="left_arm") || (part=="right_arm"))
            slv=new iCubArmCartesianSolver(slvName);
        else if ((part=="left_leg") || (part=="right_leg"))
            slv=new iCubLegCartesianSolver(slvName);
        else
        {
            cout<<"Error: "<<part<<" is invalid"<<endl;
            return false;
        }

        if (slv->open(group))
            return true;
        else
        {    
            delete slv;
            return false;
        }
    }

    /************************************************************************/
    bool interruptModule()
    {
        if (slv!=NULL)
            slv->interrupt();

        return true;
    }

    /************************************************************************/
    bool close()
    {
        delete slv;
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
        if (slv->isClosed())
            return false;

        if (slv->getTimeoutFlag())
        {
            slv->getTimeoutFlag()=false;
            slv->suspend();
        }

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

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cartesianSolver");
    rf.setDefaultConfigFile("cartesianSolver.ini");
    rf.configure(argc,argv);

    SolverModule mod;
    return mod.runModule(rf);
}



