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

#ifndef __SHORT_TERM_MEMORYY_MODULE_H__
#define __SHORT_TERM_MEMORYY_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <map>
#include <vector>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
//using namespace std;

namespace iCub {
    namespace contrib {
        class ShortTermMemoryModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Short Term Memory Module class
 *
 * \brief See \ref icub_iha_ShortTermMemory
 */
class iCub::contrib::ShortTermMemoryModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    yarp::os::BufferedPort<yarp::os::Bottle> coordsPort;
    yarp::os::Port outPort;
   
    //internal data structures
    std::map< std::string, std::vector<double> > memory;
    //std::vector< std::vector<double> > memory;
    std::map< std::string, std::vector<int> > memory_process;
    std::map< std::string, int> memory_sum;

    // parameters read from ini file/command line
    //int short_term_memory_int_param;
    //the memory's resolution in samples/second
    int resolution;
    //the memory's length in seconds
    int mem_length;
    //threshold for drumming
    double sound_thresh;
    
    //keep track of how often to store new data in memory
    double lastTime, waitTime;

    //action indicies of the actions relevant to turn-taking
    int hide_act, drum_act;

    
public:

    ShortTermMemoryModule();
    virtual ~ShortTermMemoryModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
