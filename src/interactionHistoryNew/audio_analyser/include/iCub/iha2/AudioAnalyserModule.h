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

#ifndef __AUDIO_ANALYSERY_MODULE_H__
#define __AUDIO_ANALYSERY_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha2/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class AudioAnalyserModule;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha2;

#define AUDIO_WINDOW_SIZE 4
#define AUDIO_MAX_VAL 1000

/**
 *
 * Audio Analyser Module class
 *
 * \brief See \ref icub_iha2_AudioAnalyser
 */
class iCub::contrib::AudioAnalyserModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    BufferedPort<Sound> soundPort;
    Port sndSensorPort;

    // parameters read from ini file/command line
    int audio_analyser_int_param;

    int hmask[4];
    int lmask[4];
    double window[AUDIO_WINDOW_SIZE];
    double f;
    int z;
    int count, count1, count2, count3,step;
    double val;
    double high, low, avg;
    double vhigh, vlow;
    int H, L;
    double max1,max2,max3;
    int maxid1,maxid2,maxid3;
    int th1;
    int th;
    int beatNo;
    int top;
    time_t stime, ftime;
    time_t stime1, ftime1, ftime2;
    clock_t st, ft, st1;
    clock_t ss, fs;//session start and finish
    int total_time;//default value
    int duration;
    double dur;
    double durArr[100];
    int durArray[20];
    int mode;//1 play 0 listen
    int stop;
    Port dataPortOut;
    double Threshold1, Threshold2;//sends beats if human stays silent from 1 sec.
    double duration_double;


public:

    AudioAnalyserModule();
    virtual ~AudioAnalyserModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
