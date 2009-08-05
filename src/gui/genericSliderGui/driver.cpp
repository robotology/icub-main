// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "driver.h"
#include <stdio.h>
#include <ace/OS.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/String.h>
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

///*****************************************************************/
cDriver::cDriver ()
{

}


//*****************************************************************/
int cDriver::init (Searchable &config)
{
    bool ret;

    ret=dd.open(config);
    if (!ret)
        return -1;
    
    dd.view(iCanBus);
    dd.view(iFactory);
    
    if (iCanBus==0)
        return -1;
    if (iFactory==0)
        return -1;

    int i;
	for (i = 1700; i < 2047; i++) iCanBus->canIdAdd (i);
	for (i = 0x200; i < 0x2FF; i++) iCanBus->canIdAdd (i); //for strain board (polling messages used for calibration)

    iCanBus->canSetBaudRate(0); //0=1Mbit/s

    return 0;
}

int cDriver::uninit ()
{
    dd.close();
    return true;
}

//*****************************************************************/
int cDriver::receive_message(CanBuffer &messages, int howMany, double TIMEOUT)
{
    bool  ret;
    unsigned int how_many_messages=howMany;

    int read=0;
    int count=0;

    double start=Time::now();
    double now=start;
    bool done=false;
    while(!done)
        {
            ret=iCanBus->canRead(messages, MAX_READ_MSG, &how_many_messages, false);
            now=Time::now();
            read+=how_many_messages;
            
            if (read>=howMany)
                done=true;
            if ( (now-start)>TIMEOUT)
                done=true;
            //  Time::delay(0.0);
        }

    if(!ret) 
        return 0;

    return read;
}

//*****************************************************************/
int cDriver::send_message(CanBuffer &message, int messages)
{
    unsigned int sent=0;

	bool ret = iCanBus->canWrite(message, messages, &sent);
	
	if(!ret)
        return 0;
	else
		return sent;
}

yarp::dev::CanBuffer cDriver::createBuffer(int m)
{
    return iFactory->createBuffer(m);
}


void cDriver::destroyBuffer(yarp::dev::CanBuffer &buff)
{
    iFactory->destroyBuffer(buff);
}
