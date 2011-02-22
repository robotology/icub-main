// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Martin Peniak, Vadim Tikhanoff
* email:   martin.peniak@plymouth.ac.uk, vadim.tikhanoff@iit.it
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

#include "EyeLidsController.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

EyeLids::EyeLids()
{
    
}

EyeLids::~EyeLids()
{
    ClosePort();
}

void EyeLids::ClosePort()
{
   port.close();
}

void EyeLids::setName(string name) {
    this->portName = name;
    OpenPort();
}

bool EyeLids::OpenPort()
{
    string eyelidsName = "/face/eyelids";
    string eyelicsPort = this->portName + eyelidsName;
    port.open( eyelicsPort.c_str() );
	return true;
}


void EyeLids::checkPort()
{
   if(port.getInputCount()>0)
	{
        Bottle *bot = port.read(false);
		if (bot!=NULL){
	        eyeLidsRotation = (float)bot->get(0).asDouble();
            bot->clear();
		    printf("Message received: %s\n",bot->toString().c_str());
        }    
    }
}
