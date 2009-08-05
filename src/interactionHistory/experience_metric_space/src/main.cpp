// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
 * email:   assif.mirza@robotcub.org
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

#include <iCub/iha/ExperienceMetricSpaceModule.h>

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

// class global statics
int Experience::numActions = 0;

/**
@ingroup icub_iha_module

\defgroup icub_iha_ExperienceMetricSpace Experience Metric Space Module (IHA)

\brief Experience Metric Space Module (part of the Interaction History Architecture)

 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Experience Metric Space Module\n");
    srand( time(NULL) );
    Network yarp;
    ExperienceMetricSpaceModule module;
    module.setName("/experience_metric_space"); // set default name of module

    // Get configuration
    if (!module.openFromCommand(argc,argv)) {
        ACE_OS::fprintf(stderr,"Module failed ot open.\n");
        return -1;
    }
    module.attachTerminal();

    module.initDataStore();
    if (!module.loadMetricSpace()) {
        ACE_OS::fprintf(stderr,"Error loading metric space\n");
        return -1;
    };

    int result =  module.runModule();

    //module.saveMetricSpace();

    module.close();
    
    return result;
}
