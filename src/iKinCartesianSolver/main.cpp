/** 
\defgroup iKinCartesianSolver iKinCartesianSolver 
 
@ingroup icub_module  
 
Just a container which runs the \ref iKinSlv "Cartesian Solver" 
taking parameters from a configuration file. 
 
Copyright (C) 2009 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description

See \ref iKinSlv "Cartesian Solver" for documentation. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref iKin "iKin" library (it requires IPOPT lib: see 
  http://eris.liralab.it/wiki/Installing_IPOPT ).

\section parameters_sec Parameters
--part \e type [mandatory]
- select the part to run. \e type is the group name within the 
  configuration file (e.g. left_arm, right_arm, ...).

--context \e directory [optional]
- allow to specify a different path where to search the 
  configuration file in the form of \e
  $ICUB_ROOT/app/<directory>. Directory default value is \e
  default/conf.
 
--from \e file [optional]
- allow to specify a different configuration file from the 
  default one which is \e icub_iKinCartesianSolver.ini.
 
\section portsa_sec Ports Accessed
 
All ports which allow the access to motor interface shall be 
previously open. 

\section portsc_sec Ports Created 
 
- /<solverName>/in : for requests in streaming mode.
- /<solverName>/rpc : for requests and replies.
- /<solverName>/out : for output streaming. 
 
\note for a detailed description, see \ref iKinSlv "Cartesian 
      Solver".
 
\section conf_file_sec Configuration Files
 
Here's how the configuration file will look like: 
 
\code 
[left_arm]
robot       icub
name        CartesianSolver/left_arm
type        left
period      20
dof         (0 0 0 1 1 1 1 1 1 1)
pose        full
mode        shot
verbosity   off
maxIter     200
tol         0.001
xyzTol      0.000001
interPoints off
 
[right_arm]
robot       icub
name        CartesianSolver/right_arm
type        right
period      20
dof         (0 0 0 1 1 1 1 1 1 1)
pose        full
mode        shot
verbosity   off
maxIter     200
tol         0.001   
xyzTol      0.000001
interPoints off

[left_leg]
robot       icub
name        CartesianSolver/left_leg
type        left
period      20
dof         (1 1 1 1 1 1)
pose        full
mode        shot
verbosity   off
maxIter     200
tol         0.001   
xyzTol      0.000001
interPoints off

[right_leg]
robot       icub
name        CartesianSolver/right_leg
type        right
period      20
dof         (1 1 1 1 1 1)
pose        full
mode        shot
verbosity   off
maxIter     200
tol         0.001   
xyzTol      0.000001
interPoints off
\endcode 
 
\note for a detailed description of options, see \ref iKinSlv
      "Cartesian Solver".
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

#include <iCub/iKinSlv.h>

#include <iostream>
#include <iomanip>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace iKin;


class SolverModule: public RFModule
{
protected:
    CartesianSolver *slv;

public:
    SolverModule()
    {
        slv=NULL;
    }

    virtual bool configure(ResourceFinder &rf)
    {                
        string part, slvName;

        if (rf.check("part"))
            part=rf.find("part").asString();
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
            slvName=group.find("name").asString();            
        else
        {
            cout<<"Error: name option is missing"<<endl;
            return false;
        }

        if (part=="left_arm" || part=="right_arm")
            slv=new ArmCartesianSolver(slvName);
        else if (part=="left_leg" || part=="right_leg")
            slv=new LegCartesianSolver(slvName);
        else
        {
            cout<<"Error: "<<part<<" is invalid"<<endl;
            return false;
        }

        return slv->open(group);
    }

    virtual bool close()
    {
        if (slv!=NULL)
            delete slv;

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
    rf.setDefaultContext("default/conf");
    rf.setDefaultConfigFile("icub_iKinCartesianSolver.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    SolverModule mod;

    return mod.runModule(rf);
}



