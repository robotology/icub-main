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

#ifndef __SOUNDSENSORMODULE__
#define __SOUNDSENSORMODULE__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <ace/OS.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class SoundSensorModule;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

/**
 *
 * Sound Sensor Module class
 *
 * \brief See \ref icub_iha_SoundSensor
 */
class iCub::contrib::SoundSensorModule : public yarp::os::Module {

private:

    // ports
    BufferedPort<yarp::os::Bottle> quitPort;
    BufferedPort<Sound> soundPort;
    Port sndSensorPort;

    int sndSensorRate;
    double sndGain;

public:

    SoundSensorModule();
    virtual ~SoundSensorModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    //virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
