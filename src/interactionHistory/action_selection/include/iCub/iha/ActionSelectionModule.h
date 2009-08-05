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

#ifndef __ACTION_SELECTION_MODULE_H__
#define __ACTION_SELECTION_MODULE_H__

#include <stdio.h>
#include <string>
#include <iostream>
#include <map>

#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/iha/debug.h>

#include <iCub/iha/ActionSelect.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
    namespace contrib {
        class ActionSelectionModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Action Selection Module class
 *
 * \brief See \ref icub_iha_ActionSelection
 */
class iCub::contrib::ActionSelectionModule : public yarp::os::Module {

private:

    // ports
    yarp::os::BufferedPort<yarp::os::Bottle> quitPort;
    map < int, yarp::os::BufferedPort< yarp::os::Bottle >* > currdistPorts;
    yarp::os::Port statusPort;

    yarp::os::Port actionOutPort;

    // parameters read from ini file/command line
    int max_repeat_zero;

    int* horizons;
    int numHorizons;
    int numActions;

    Bottle** neighbourlist; // Array of pointers to Bottles
    bool* data_read;        // Array of bools

    int current_action;
    int prev_act;
    bool robot_busy;
    int zero_action_count;

    bool test_mode;

    /**
     * ActionSelect is the probability based algorithm
     * to select an action based on a nearest-neighbour list
     */
    ActionSelect actionSelector;


public:

    ActionSelectionModule();
    virtual ~ActionSelectionModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

    /**
     * Send an action over a port and receive an acknowledgement
     */
    bool sendAction(int act);

    /**
     * To read one set of exp distances from the ports
     */
    void readDistances();


};


#endif
