/* 
 * Copyright (C) <2008> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Hatice Kose-Bagci (University of Hertfordshire)
 * email:   h.kose-bagci@herts.ac.uk
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

/*
 *  Takes the patterns via messaging bottles from the audioAnalyser and displays.
 *
 */

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp;


int main() {
    time_t _stime, ftime;
	Network::init();
	Port input;
	int total_time=60;//default play time
	int i;
	int beat=0, duration;
	double dtime, timeDiff;
	input.open("/uh/Drum/pattern:i");
	time(&_stime);
	time(&ftime);
	while(difftime(ftime, _stime)<total_time){
		//the program would run for total_time
		Bottle bot;
		input.read(bot);
		//pattern is taken from audioAnalyser
		beat = bot.get(0).asVocab();
		printf("Got message: %d\n", beat);
		for (i=1; i<beat; i++){
			duration = bot.get(i).asVocab();
			printf("Got duration: %d\n", duration);
		}
	time(&ftime);
	}
	input.close();
    Network::fini();
    return 0;
}

