/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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
\defgroup d4cServer d4cServer 
 
@ingroup icub_module
 
Just a container which runs the server part of the \ref d4c 
"Dynamic Force Field Controller" (D4C) framework. 
 
\section intro_sec Description 
See \ref d4c "D4C Library" for detailed documentation. 
 
\section lib_sec Libraries 
- YARP libraries. 
- \ref d4c "D4C" library.

\section parameters_sec Parameters
--robot \e robot
- select the robot to connect to.

--part \e part
- select the part to control. Available parts are 
  <i>left_arm</i>, <i>right_arm</i> and \e <i>both_arms</i>.
 
--name \e name
- specify the stem-name of the server. 
 
--device \e device
- select the cartesian device to rely on. Currently only the 
  \e cartesiancontrollerclient is supported.
 
--period \e Ts
- specify the server period given in [ms].
 
--verbosity \e level
- specify the verbosity level of the server print-outs. 
 
\section portsc_sec Ports Created 
Among all the ports created by the server, one deserves 
particular attention. In fact, the port /<name>/gui:o can be 
directly connected to the \ref icub_gui objects port for 
visualization purpose. 
 
\section tested_os_sec Tested OS
Windows, Linux
 
\author Ilaria Gori, Ugo Pattacini
*/ 

#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Log.h>
#include <yarp/dev/Drivers.h>

#include <iCub/d4c/d4c_server.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::d4c;


/************************************************************************/
class ServerModule: public RFModule
{
protected:
    D4CServer server;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        int verbosity=rf.check("verbosity",Value(0)).asInt();
        int period=rf.check("period",Value(20)).asInt();
        string device=rf.check("device",Value("cartesiancontrollerclient")).asString().c_str();
        string name=rf.check("name",Value("d4c_server")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        string part=rf.check("part",Value("both_arms")).asString().c_str();

        Property options;
        options.put("verbosity",verbosity);
        options.put("period",period);
        options.put("device",device.c_str());
        options.put("name",name.c_str());
        options.put("robot",robot.c_str());
        options.put("part",part.c_str());
        
        return server.open(options);
    }

    /************************************************************************/
    bool close()
    {
        server.close();
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
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    ServerModule mod;
    return mod.runModule(rf);
}



