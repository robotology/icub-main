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

#ifndef __IHA_AC_REWARDREADLOOP__
#define __IHA_AC_REWARDREADLOOP__

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
        class RewardReadLoop;
    }
}

using namespace iCub::contrib;
using namespace iCub::iha;

class iCub::contrib::RewardReadLoop : public yarp::os::Thread
{
public:
	RewardReadLoop(BufferedPort<Bottle> &_rewardPort, Port &_actionOutPort,
                   Port &_expressionRawPort,
                   bool _reward_display, 
                   int _action_ehi, int _action_elo, int _action_emid, 
                   double _th_ehi, double _th_elo ) :
		rewardPort(&_rewardPort), actionOutPort(&_actionOutPort),
        expressionRawPort(&_expressionRawPort),
        reward_display(_reward_display), 
        action_ehi(_action_ehi), action_elo(_action_elo), action_emid(_action_emid), 
        th_ehi(_th_ehi), th_elo(_th_elo) 

	{ 
        currentReward=0.0;
    } 
	~RewardReadLoop(){
        ACE_OS::fprintf(stderr,"RewardReadLoop: destroyed\n");
    }
	void run();
	void onStop() {}
	void beforeStart() {}
	void afterStart() {}
	bool threadInit() { return true; }
	void threadRelease() {}

    double getCurrentReward();
    void sendAction(int act);
    void sendExpression(int expr);

private:
	yarp::os::BufferedPort<Bottle> *rewardPort;
    yarp::os::Port *actionOutPort;
    yarp::os::Port *expressionRawPort;

    double currentReward;
    Semaphore rewardMutex;

    bool reward_display;
    int action_ehi;
    int action_elo;
    int action_emid;
    double th_ehi;
    double th_elo;
    int current_eout;
    int new_eout;

};

#endif
