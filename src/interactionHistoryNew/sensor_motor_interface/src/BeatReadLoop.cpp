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

#include <iCub/iha/BeatReadLoop.h>

void BeatReadLoop::run() {
	currentBeat=0;

	while (!isStopping()) {
        //IhaDebug::pmesg(DBGL_STATUSLINE,"In beat read loop\n");
		Bottle* bot = beatport->read(true);
		if (bot==NULL) {
            break;
			//IhaDebug::pmesg(DBGL_DEBUG1,"Error: Sound data null\n");
			//Time::delay(0.01);
			//continue;
		}

		Value val=bot->get(0);
        IhaDebug::pmesg(DBGL_DEBUG1,"Beats: %s\n",bot->toString().c_str());
		if (!val.isNull() && val.isInt()) {
			beatMutex.wait();
			currentBeat = val.asInt();
			beatMutex.post();
		} else {
			IhaDebug::pmesg(DBGL_DEBUG1,"Error: Beat data not valid\n");
		}
	}
    fprintf(stderr,"BeatReadLoop ended\n");
}


int BeatReadLoop::getCurrentBeat() 
{
    int retval=0;
    beatMutex.wait();
    retval=currentBeat;
    beatMutex.post();
    //IhaDebug::pmesg(DBGL_DEBUG1,"Beats: %d\n",currentBeat);
    
    return retval;
}
