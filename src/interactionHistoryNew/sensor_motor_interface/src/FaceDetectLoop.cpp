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

#include <iCub/iha/FaceDetectLoop.h>

void FaceDetectLoop::run() {
	while (!isStopping()) {
		// try to keep the current pointer valid for as long as possible
		if (coordsport->getPendingReads()>0) {
			coordsMutex.wait();
			Vector* pc = coordsport->read(true);
            if (pc==NULL) break;
			p_currentCoords = pc;
			if ( (*pc)[0]>0 ) 
                faceDetected=true;
			else 
                faceDetected=false;
			coordsMutex.post();
		} else {
			Time::delay(0.001);
		}

	}
    fprintf(stderr,"FaceDetectLoop ended\n");
}

bool FaceDetectLoop::getFaceState() 
{
    return faceDetected;
}
