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

#include <iCub/iha/IhaFaceDetectModule.h>

using namespace std;
using namespace yarp::os;

/**
 * @ingroup icub_iha2_module
 *
 * \defgroup icub_iha2_IhaFaceDetect Face Detection (IHA2)
 *
 * \brief An implementation of the OpenCV HAAR Face detector (optionally with Camshift tracking) for the IHA
 *
 *
 */

int main(int argc, char *argv[]) {

    IhaDebug::setLevel(DBGL_STATUS1);
    IhaDebug::pmesg(DBGL_INFO,"Iha Face Detect Module\n");
    Network yarp;
    IhaFaceDetectModule module;
    module.setName("/iha_face_detect"); // set default name of module
    return module.runModule(argc,argv);
}
