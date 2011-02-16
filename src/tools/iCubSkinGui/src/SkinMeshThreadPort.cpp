// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it, Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/SkinMeshThreadPort.h"

#include <yarp/os/Time.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

bool SkinMeshThreadPort::threadInit()
{
	printf("SkinMeshThreadPort initialising...\n");
    printf("... done!\n");

	printf("Waiting for port connection\n");
    return true;
}

void SkinMeshThreadPort::run()
{	
    mutex.wait();

	Bottle *input=0;

    input = skin_port.read(false);
    
    if (input==0)
    {
        mutex.post();
        return;
    }

	yarp::sig::Vector skin_value;
	int sensorId =0;
	skin_value.resize(input->size());
	for (int i=0; i<input->size(); i++)
	{
		skin_value[i] = input->get(i).asDouble();

		sensorId=i/12;
		if (sensorId<16 && sensor[sensorId])
			sensor[sensorId]->setActivationFromPortData(skin_value[i],i%12);
	}

    mutex.post();
}

void SkinMeshThreadPort::threadRelease()
{
	printf("SkinMeshThreadPort releasing...\n");	

    printf("... done.\n");
}
