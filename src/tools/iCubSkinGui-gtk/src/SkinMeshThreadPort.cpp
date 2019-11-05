// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Randazzo, Marco Maggiali, Alessandro Scalzo
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
    std::lock_guard<std::mutex> lck(mtx);

    for (Bottle *input=NULL; input=skin_port.read(false);) 
    {    
        yarp::sig::Vector skin_value;
        skin_value.resize(input->size());
        for (int i=0; i<input->size(); i++)
        {
            skin_value[i] = input->get(i).asDouble();
        }

        for (int sensorId=0; sensorId<MAX_SENSOR_NUM; sensorId++)
        {
            if (sensor[sensorId]==0) continue;

            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                int curr_tax = i-sensor[sensorId]->min_tax;
            
                sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
            }
        }
    }
}

void SkinMeshThreadPort::threadRelease()
{
    printf("SkinMeshThreadPort releasing...\n");	
    printf("... done.\n");
}
