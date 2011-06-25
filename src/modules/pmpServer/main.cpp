/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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
\defgroup pmpServer pmpServer 
 
@ingroup icub_contrib_modules
 
Launches the server part of the Passive Motion Paradigm (PMP) 
control. 
 
\author Ilaria Gori, Ugo Pattacini
*/ 

#include <string>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/Drivers.h>

#include <iCub/pmp/pmp_server.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace iCub::pmp;


class ServerModule: public RFModule
{
protected:
    PmpServer server;

public:
    bool configure(ResourceFinder &rf)
    {
        int verbosity=rf.check("verbosity",Value(0)).asInt();
        int period=rf.check("period",Value(20)).asInt();
        string device=rf.check("device",Value("cartesiancontrollerclient")).asString().c_str();
        string name=rf.check("name",Value("pmp_server")).asString().c_str();
        string robot=rf.check("robot",Value("icub")).asString().c_str();
        string part=rf.check("part",Value("right_arm")).asString().c_str();

        Property options;
        options.put("verbosity",verbosity);
        options.put("period",period);
        options.put("device",device.c_str());
        options.put("name",name.c_str());
        options.put("robot",robot.c_str());
        options.put("part",part.c_str());
        
        return server.open(options);
    }

    bool close()
    {
        server.close();
        return true;
    }

    double getPeriod()    { return 1.0;  }
    bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    ServerModule mod;

    return mod.runModule(rf);
}



