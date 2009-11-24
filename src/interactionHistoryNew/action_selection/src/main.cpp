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

#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/ActionSelectionModule.h>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;

/**
 * @ingroup icub_iha_module
 * 
 * \defgroup icub_iha_ActionSelection Action Selection (IHA)
 * 
 * \brief Action Selection based on nearest neighbour experiences in an \b Experience \b Metric \b Space, their quality value and temperature.

 */


int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Action Selection Module\n");
    srand( time(NULL) );
    Network yarp;
    ActionSelectionModule module;
    module.setName("/action_selection"); // set default name of module
    return module.runModule(argc,argv);
}
