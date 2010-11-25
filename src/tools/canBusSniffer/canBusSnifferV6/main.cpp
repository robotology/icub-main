// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
*
Connect to the canbus, read messages and dump to file. Based on 
code by Lorenzo Natale adn Alberto Parmiggiani.

\author Unknown

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/canBusSniffer/main.cpp.
**/

#include <stdio.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <bitset>

YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

using namespace std;

const int SNIFFER_THREAD_RATE=50;
const int CAN_DRIVER_BUFFER_SIZE=2047;
const int localBufferSize=2048;
bool done=false;


bool log_start=true;
class SnifferThread: public RateThread
{
    PolyDriver driver;
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    CanBuffer messageBuffer;
	unsigned long int cnt;    
	FILE *fp;

	/* dimension of local buffer, number of recieved messages */
    unsigned int messages, readMessages;

	/* variables to be sniffed from the Can Bus */
	signed int position[2];
	signed short speed[2];
	signed short pwm[2];
	signed short pid[2];
	signed short sin_frequency[2];
	signed short sin_amplitude[2];
	signed short dutyCycle[2];
	signed short torque[2];
	signed short commut[2];

	unsigned short unsigned_gaugeData[6];
	signed short signed_gaugeData[6];
	signed short dataBuffer[6];

public:
    SnifferThread(int r=SNIFFER_THREAD_RATE): RateThread(r)
    {
		messages = localBufferSize;

		for(int i=0; i<6; i++)
		{
			unsigned_gaugeData[i] = 0;
			signed_gaugeData[i] = 0;
			dataBuffer[i] = 0;
		}

		for(int i=0; i<2; i++)
		{
			position[i] = 0;
			speed[i] = 0;
			pwm[i] = 0;
			pid[i] = 0;
			dutyCycle[i] = 0;
			torque[i] = 0;
			commut[i] = 0;
			sin_frequency[i] = 0;
			sin_amplitude[i] = 0;
		}
		
	}

    bool threadInit()
    {
		// load configuration parameters into the options property collector
        Property prop;

		// set driver properties
		prop.put("device", "ecan");

        prop.put("CanTxTimeout", 500);
        prop.put("CanRxTimeout", 500);
        prop.put("CanDeviceNum", 0);

		prop.put("CanTxQueue", CAN_DRIVER_BUFFER_SIZE);
        prop.put("CanRxQueue", CAN_DRIVER_BUFFER_SIZE);

		// open driver
        driver.open(prop);

        if (!driver.isValid())
        {
            fprintf(stderr, "Error opening PolyDriver check parameters\n");
            return false;
        }

        driver.view(iCanBus);
        driver.view(iBufferFactory);

		// set the baud rate (0 is defaul for 1Mb/s) 
		if (!iCanBus->canSetBaudRate(0))
			fprintf(stderr, "Error setting baud rate\n");

		// add the id of can messages to be read
		iCanBus->canIdAdd(0x11A);
        messageBuffer=iBufferFactory->createBuffer(localBufferSize);

		cnt = 0;
		fp = fopen("output.dat","w");
    }

    void run()
    {
		readMessages = 0; 
		// read from the Can Bus messages with the id that has been specified
        bool res=iCanBus->canRead(messageBuffer, messages, &readMessages);
		
		// parse the messages 
		for(int i=0; i<readMessages; i++)
		{
			/*
			if (messageBuffer[i].getId() == 0x35a)
			{
				unsigned_gaugeData[0] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[1] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[2] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[0] = unsigned_gaugeData[0]-0x7fff;
				signed_gaugeData[1] = unsigned_gaugeData[1]-0x7fff;
				signed_gaugeData[2] = unsigned_gaugeData[2]-0x7fff;
			}
			if (messageBuffer[i].getId() == 0x35b)
			{
				unsigned_gaugeData[3] = (messageBuffer[i].getData()[1]<<8)|messageBuffer[i].getData()[0];
				unsigned_gaugeData[4] = (messageBuffer[i].getData()[3]<<8)|messageBuffer[i].getData()[2];
				unsigned_gaugeData[5] = (messageBuffer[i].getData()[5]<<8)|messageBuffer[i].getData()[4];
				signed_gaugeData[3] = unsigned_gaugeData[3]-0x7fff;
				signed_gaugeData[4] = unsigned_gaugeData[4]-0x7fff;
				signed_gaugeData[5] = unsigned_gaugeData[5]-0x7fff;
			}
			*/

			if (messageBuffer[i].getId() == 0x11B) 
			{
				sin_frequency[0] = messageBuffer[i].getData()[0];
				sin_amplitude[0] = messageBuffer[i].getData()[1];
			}

			if (messageBuffer[i].getId() == 0x11A) 
			{
				position[0]  = (messageBuffer[i].getData()[1]<<8) | messageBuffer[i].getData()[0];
				speed[0]	 = (messageBuffer[i].getData()[3]<<8) | messageBuffer[i].getData()[2];
				pid[0]		 = (messageBuffer[i].getData()[5]<<8) | messageBuffer[i].getData()[4];
				torque[0]    = (messageBuffer[i].getData()[7]<<8) | messageBuffer[i].getData()[6];
			
			/*	if (sin_frequency[0]==1000)
				{
					this->stop();
					fclose(fp);
					exit(0);
				};*/
						
				if(log_start) fprintf(fp,"%d %d %d %d %d %d %d\n",cnt,sin_frequency[0],sin_amplitude[0],position[0],speed[0],pid[0],torque[0]);
				cnt++;
				if ((cnt % 1000) == 0)
				{
					fprintf(stdout,"%d %d %d %d %d %d %d\n",cnt,sin_frequency[0],sin_amplitude[0],position[0],speed[0],pid[0],torque[0]);
				}
			}
		}
	
/*
			if (cnt==50000) 
			{
				this->stop();
				done =true;
			}
*/
		
	/*	
		cout<<setiosflags(ios::fixed)
			<<setw(10)<<"commut:"
			<<setw(8)<<setprecision(3)<<commut[0]
			<<" duty cycle:"
			<<setw(8)<<setprecision(3)<<dutyCycle[0]
			<<" gauge 5:"
			<<setw(8)<<setprecision(3)<<signed_gaugeData[5]
			<<" dsp torque:"
			<<setw(8)<<setprecision(3)<<torque[0]
			<<" pid:"
			<<setw(8)<<setprecision(3)<<pid[0]
			<<" kp:"
			<<setw(8)<<setprecision(3)<<kp[0]
			<<"\r";
		*/	
		/*
		cout<<setiosflags(ios::fixed)
			<<" "
			<<setw(8)<<setprecision(3)<<cnt
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[0]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[1]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[2]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[3]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[4]
			<<" "
			<<setw(8)<<setprecision(3)<<gaugeData[5]<<"\r";
		*/
    }

    void threadRelease()
    {
        iBufferFactory->destroyBuffer(messageBuffer);
        driver.close();
		fclose(fp);
    }
};

int main(int argc, char *argv[]) 
{
	YARP_REGISTER_DEVICES(icubmod)

    SnifferThread thread;
    thread.start();

    std::string input;
    while(!done)
    {
/*        std::cin>>input;
        if (input=="quit")
            done=true;*/
    }

    thread.stop();
}