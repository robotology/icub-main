// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \file EyeLidsController.cpp
 * \brief This file initializes the port to be used in order to control the eyeLids
 * \author Martin Peniak, Vadim Tikhanoff
 * \date 2008
 * \note Release under GNU GPL v2.0
 **/
#include "EyeLidsController.h"

using namespace yarp::os;
using namespace yarp::sig;

EyeLids::EyeLids()
{
    OpenPort();
}

EyeLids::~EyeLids()
{
    ClosePort();
}

void EyeLids::ClosePort()
{
   port.close();
}

bool EyeLids::OpenPort()
{
    port.open("/icubSim/face/eyelids");
	if(port.getInputCount()>0)
	{
		    Bottle *bot = port.read(false);
			if (bot!=NULL) 
            {
			    eyeLidsRotation=bot->get(0).asDouble();
			    bot->clear();
			    printf("Message received: %s\n",bot->toString().c_str());
            }
    }
	
	return true;
}
