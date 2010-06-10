// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/SkinMeshThreadPort.h>

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

	Bottle *input = skin_port.read();
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