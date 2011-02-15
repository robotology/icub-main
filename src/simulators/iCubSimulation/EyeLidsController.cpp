// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file EyeLidsController.cpp
 * \brief This file initializes the port to be used in order to control the eyeLids
 * \author Martin Peniak, Vadim Tikhanoff
 * \date 2008
 * \note Released under the terms of the GNU GPL v2.0.
 **/
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
