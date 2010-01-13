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

#include <iCub/iha/GazeReadLoop.h>

void GazeReadLoop::run() {
	currentGaze=0.0;
	while (!isStopping()) {
		Bottle* bot = gazeport->read(true);
		if (bot==NULL) {
            break;
			//IhaDebug::pmesg(DBGL_DEBUG1,"Error: Sound data null\n");
			//Time::delay(0.01);
			//continue;
		}

        //classify the gaze data into an output state
        //reading format is gaze x, gaze y, 
        //face bounding box x start, y start, x end, y end
        
        //states are rectangular image regions around the face bounding box
        double time = bot->get(0).asDouble();
        int gx = bot->get(1).asInt();
        int gy = bot->get(2).asInt();
        int lx = bot->get(3).asInt();
        int ly = bot->get(4).asInt();
        int ux = bot->get(5).asInt();
        int uy = bot->get(6).asInt();

        IhaDebug::pmesg(DBGL_DEBUG1,"gx %d gy %d lx %d ly %d  ux %d uy %d\n",gx,gy,lx,ly,ux,uy);

        double gazestate;

        if ((gx < 0) || (gy < 0) || (lx < 0) || (ly < 0) || (ux < 0) || (uy < 0)) {
            //there is an error in part of the gaze reading
            gazestate = -1.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: err\n");
        } else if ((gx > lx) && (gx < ux) && (gy > ly) && (gy < uy)) {
            //the gaze is inside the face bounding box
            gazestate = 1.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: inside\n");
        } else if ((gx > lx) && (gx < ux) && (gy < ly)) {
            //the gaze is below the face (at the body)
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: body\n");
        } else if ((gx < lx) && (gy < ly)) {
            //the gaze is to the lower left of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: lower left\n");
        } else if ((gx > ux) && (gy < ly)) {
            //the gaze is to the lower right of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: lower right\n");
        } else if ((gx > lx) && (gx < ux) && (gy > uy)) {
            //the gaze is above the face 
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: above\n");
        } else if ((gx < lx) && (gy > uy)) {
            //the gaze is to the upper left of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: upper left\n");
        } else if ((gx > ux) && (gy > uy)) {
            //the gaze is to the upper right of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: upper right\n");
        } else if ((gx < lx) && (gy > ly) && (gy < uy)) {
            //gaze is to the left of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: left\n");
        } else if ((gx > ux) && (gy > ly) && (gy < uy)) {
            //gaze is to the right of the face
            gazestate = 0.0;
            //IhaDebug::pmesg(DBGL_DEBUG1,"State: right\n");
        }

        
        gazeMutex.wait();
        currentGaze = gazestate;
        gazeMutex.post();
        
        //IhaDebug::pmesg(DBGL_DEBUG1,"Error: Gaze data not valid\n");
        
	}
    fprintf(stderr,"GazeReadLoop ended\n");
}


double GazeReadLoop::getCurrentGaze() 
{
    double retval=0.0;
    gazeMutex.wait();
    retval=currentGaze;
    gazeMutex.post();
    IhaDebug::pmesg(DBGL_DEBUG2,"Mutual Gaze: %f\n",currentGaze);
    
    return retval;
}
