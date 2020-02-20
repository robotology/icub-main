/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <yarp/os/all.h>

#include "module.h"

using namespace yarp::os;


/****************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setDefaultContext("depth2kin");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("calibrationFile","calibration_data.ini");
    rf.configure(argc,argv);

    int test=rf.check("test",Value(-1)).asInt();
    if (test<0)
    {
        if (!yarp.checkNetwork())
        {
            yError("YARP server not available!");
            return 1;
        }
    }

    CalibModule mod;
    return mod.runModule(rf);
}



