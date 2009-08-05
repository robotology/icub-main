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

#ifndef __IHA_AC_SOUNDSENSORREADLOOP__
#define __IHA_AC_SOUNDSENSORREADLOOP__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class SoundSensorReadLoop;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

class iCub::contrib::SoundSensorReadLoop : public yarp::os::Thread
{
public:
	SoundSensorReadLoop(BufferedPort<Bottle> &_soundsensorport) :
		soundsensorport(&_soundsensorport)
	{ 
        currentSoundSensor=0.0;
    } 
	~SoundSensorReadLoop(){
        ACE_OS::fprintf(stderr,"SoundSensorReadLoop: destroyed\n");
    }
	void run();
	void onStop() {}
	void beforeStart() {}
	void afterStart() {}
	bool threadInit() { return true; }
	void threadRelease() {}

    double getCurrentSoundSensor();
        
private:
	BufferedPort<Bottle> *soundsensorport;
    double currentSoundSensor;
    Semaphore soundsensorMutex;
};

#endif
