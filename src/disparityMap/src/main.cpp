/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Harold Martinez
 * email:   martinez@ifi.uzh.ch
 *
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


// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/DisparityMapModule.h>



using namespace std;
using namespace yarp::os;
using namespace iCub::contrib;



int main(int argc, char *argv[]) {

    Network yarp;
    DisparityMapModule module;
    module.setName("/disparitymap"); // set default name of module
    return module.runModule(argc,argv);
}