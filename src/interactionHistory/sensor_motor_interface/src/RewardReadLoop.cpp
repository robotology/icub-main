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

#include <iCub/iha/RewardReadLoop.h>

void RewardReadLoop::sendAction(int act) {
    Bottle bot;
    bot.addInt(act);
    actionOutPort->write(bot);
}

void RewardReadLoop::sendExpression(int expr) {
    Bottle out;
    // hard coded for now
    if (expr==1) {
        out.add("L04");
        expressionRawPort->write(out);
        out.clear();
        out.add("R04");
        expressionRawPort->write(out);
        out.clear();
        out.add("M38");
        expressionRawPort->write(out);
        out.clear();
        out.add("S48");
        expressionRawPort->write(out);
        out.clear();
    }
    if (expr==2) {
        out.add("L02");
        expressionRawPort->write(out);
        out.clear();
        out.add("R02");
        expressionRawPort->write(out);
        out.clear();
        out.add("M08");
        expressionRawPort->write(out);
        out.clear();
        out.add("S40");
        expressionRawPort->write(out);
        out.clear();
    }
    if (expr==16) {
        out.add("L04");
        expressionRawPort->write(out);
        out.clear();
        out.add("R04");
        expressionRawPort->write(out);
        out.clear();
        out.add("M0B");
        expressionRawPort->write(out);
        out.clear();
        out.add("S3B");
        expressionRawPort->write(out);
        out.clear();
    }
    
}

void RewardReadLoop::run() {
    sendAction(current_eout);
	while (!isStopping()) {
		Bottle* bot = rewardPort->read(true);
        if (bot==NULL) break;
		rewardMutex.wait();
		currentReward = bot->get(0).asDouble();
		rewardMutex.post();

		// execute certain actions in response to reward value
		if (reward_display) {
			if (currentReward >= th_ehi) {
				new_eout=action_ehi;
			} else if (currentReward < th_elo) {
				new_eout=action_elo;
			} else {
				new_eout=action_emid;
			}
			
			if (current_eout != new_eout) {
				IhaDebug::pmesg(DBGL_STATUS1,"============ Emote action %d\n",new_eout);
				//sendAction(new_eout);
				sendExpression(new_eout);
				IhaDebug::pmesg(DBGL_DEBUG1,"============ Emote action %d done\n",new_eout);
			}
			current_eout = new_eout;
		}
	}
    fprintf(stderr,"RewardReadLoop ended\n");
}

double RewardReadLoop::getCurrentReward() 
{
    double retval;
    rewardMutex.wait();
    retval=currentReward;
    rewardMutex.post();

    return retval;
}
