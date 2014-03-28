// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

///
/// $Id: SocketCan.cpp,v 0.9 2010/12/05 18:00:00 randaz Exp $
///

#include "SocketCan.h"
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <sys/types.h>
#include <yarp/os/Time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif
 
#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

#define SOCK_DEBUG 0

using namespace yarp::dev;
using namespace yarp::os;

const int TX_QUEUE_SIZE=2047;
const int RX_QUEUE_SIZE=2047;

SocketCan::SocketCan()
{
    skt = 0;
}

SocketCan::~SocketCan()
{
 
}

bool SocketCan::canSetBaudRate(unsigned int rate)
{ 
    //not yet implemented
    return true;
}

bool SocketCan::canGetBaudRate(unsigned int *rate)
{
    //not yet implemented
    return true;
}

bool SocketCan::canIdAdd(unsigned int id)
{
    //not yet implemented
    return true;
}

bool SocketCan::canIdDelete(unsigned int id)
{
    //not yet implemented 
    return true;
}

bool SocketCan::canRead(CanBuffer &msgs,
                     unsigned int size, 
                     unsigned int *readout,
                     bool wait)
{
/*
    int res;
    #ifdef WIN32
        long lRead=size;
    #else
        int32_t lRead=size;
    #endif

    CMSG *tmp=reinterpret_cast<CMSG *>(msgs[0].getPointer());
    if (wait)
        res=::canRead(*handle, tmp, &lRead, 0);
    else
        res=::canTake(*handle, tmp, &lRead);

    *readout=lRead;

    if ( (res==NTCAN_SUCCESS)||(res==NTCAN_RX_TIMEOUT))
        return true;

    fprintf(stderr, "Error: canRead returned with code:%.8X\n", res);*/

    int i=0;
    int bytes_read=0;
    #if SOCK_DEBUG
        printf("Asked for %d messages\n", size);
    #endif
    for (i=0; i<size; i++)
    {
        can_frame *frm=reinterpret_cast<can_frame *>(msgs[i].getPointer());
        //printf("start reading\n");
        bytes_read = 0;
        bytes_read = read( skt, frm, sizeof(*frm) );
        #if SOCK_DEBUG
            printf("finished reading. read %d bytes\n",bytes_read);
        #endif
        if (bytes_read<=0) break;

        #if SOCK_DEBUG
            printf("Read: %d \n ",bytes_read);
            printf("len %d ", frm->can_dlc);
            printf("id %d ", frm->can_id);
            printf("data: ");
            for(int j=0;j<frm->can_dlc;j++)
                printf("%2x ", frm->data[j]);
            printf("\n");
        #endif

        //yarp::os::Time::delay(0.010); //test only
    }
    *readout=i;
    #if SOCK_DEBUG
        printf("Read %d messages\n", *readout);
    #endif
    return true;

	/*
	//debug
	//struct can_frame frame;
    can_frame *frame=reinterpret_cast<can_frame *>(msgs[0].getPointer());
    bytes_read = read( skt, frame, sizeof(*frame) );
    *readout=1;

    printf("Read: %d \n ",bytes_read);

    printf("len %d ", frame->can_dlc);
    printf("id %d ", frame->can_id);
    printf("data: ");
    for(i=0;i<frame->can_dlc;i++)
        printf("%2x ", frame->data[i]);
    printf("\n");

    return true;
	*/
}

bool SocketCan::canWrite(const CanBuffer &msgs,
                      unsigned int size,
                      unsigned int *sent,
                      bool wait)
{
    int res;
    int32_t lRead=size;
    int bytes_sent=0;

/*
    //debug
    struct can_frame frame2;
    frame2.can_id = 0x123;
    strcpy( (char*)(&frame2.data), "YAY" );
    frame2.can_dlc = strlen( (char*)(&frame2.data) );
    bytes_sent = write( skt, &frame2, sizeof(frame2) );
*/
/*
    //debug
    can_frame frame3;
    frame3.can_id = 0x123;
    frame3.data[0] = 'Y';
    frame3.can_dlc = 1;
    bytes_sent = write( skt, &frame3, sizeof(frame3) );
*/

    //debug
    //fprintf(stderr, "id:%d data:%d size:%d\n", tmp->can_id, tmp->data[0], tmp->can_dlc );
   
    //@@@ IMPORTANT (RANDAZ): I'm putting here a delay of one millisecond.
	//I noticed that without this delay a lot CAN messages are lost when iCubInterface starts and
	//sends the configuration parameters (PIDs etc.) to the control boards.
	//Further investigation is required in order to understand how the internal buffer is handled 
	//when the function write( skt, tmp, sizeof(*tmp) ); is called.
	Time::delay(0.001);
	
    int i=0;
	(*sent)=0;
	for (i=0; i<size; i++)
    {
		CanBuffer &buffer=const_cast<CanBuffer &>(msgs);
		const struct can_frame *tmp=reinterpret_cast<const struct can_frame*>(buffer[i].getPointer());   
		bytes_sent = write( skt, tmp, sizeof(*tmp) );
		if (bytes_sent>0) 
		{
			(*sent)++;
		}
        else
		{
			fprintf(stderr, "Error: SocketCan::canWrite() was unable to send message.\n");
			//break;
		}
	}
	
    if (*sent <size)
       {
           fprintf(stderr, "Error: SocketCan::canWrite() not all messages were sent.\n");
           return false;
       }
   
    return true;
}

bool SocketCan::open(yarp::os::Searchable &par)
{
    int canTxQueue=TX_QUEUE_SIZE;
    int canRxQueue=RX_QUEUE_SIZE;
    int netId =-1;
    int txTimeout=500;
    int rxTimeout=500;

                         netId=par.check("CanDeviceNum", Value(-1), "numeric identifier of the can device").asInt();
    if  (netId == -1)    netId=par.check("canDeviceNum", Value(-1), "numeric identifier of the can device").asInt();
    
                           txTimeout=par.check("CanTxTimeout", Value(500), "timeout on transmission [ms]").asInt();
    if  (txTimeout == 500) txTimeout=par.check("canTxTimeout", Value(500), "timeout on transmission [ms]").asInt();
    
                           rxTimeout=par.check("CanRxTimeout", Value(500), "timeout on receive when calling blocking read [ms]").asInt() ;
    if  (rxTimeout == 500) rxTimeout=par.check("canRxTimeout", Value(500), "timeout on receive when calling blocking read [ms]").asInt() ;

                                      canTxQueue=par.check("CanTxQueue", Value(TX_QUEUE_SIZE), "length of tx buffer").asInt();
    if  (canTxQueue == TX_QUEUE_SIZE) canTxQueue=par.check("canTxQueue", Value(TX_QUEUE_SIZE), "length of tx buffer").asInt();
    
                                      canRxQueue=par.check("CanRxQueue", Value(RX_QUEUE_SIZE), "length of rx buffer").asInt() ;
    if  (canRxQueue == RX_QUEUE_SIZE) canRxQueue=par.check("canRxQueue", Value(RX_QUEUE_SIZE), "length of rx buffer").asInt() ;

   int so_timestamping_flags = 0;
   /* Create the socket */
   skt = socket( PF_CAN, SOCK_RAW, CAN_RAW );
 
   /* Locate the interface you wish to use */
   struct ifreq ifr;
   sprintf (ifr.ifr_name, "can%d",netId);
   ioctl(skt, SIOCGIFINDEX, &ifr); // ifr.ifr_ifindex gets filled with that device's index
 
   /* Select that CAN interface, and bind the socket to it. */
   struct sockaddr_can addr;
   addr.can_family = AF_CAN;
   addr.can_ifindex = ifr.ifr_ifindex;
   bind( skt, (struct sockaddr*)&addr, sizeof(addr) );

    int flags;
    if (-1 == (flags = fcntl(skt, F_GETFL, 0))) flags = 0;
    fcntl(skt, F_SETFL, flags | O_NONBLOCK);


   return true;
}

bool SocketCan::close()
{
/*
    int res;
    if (!handle)
        return false;

    res=::canClose (*handle);

    if (res!=NTCAN_SUCCESS)
        return false;

    delete handle;
    handle=0;
*/

    if (!skt)
        return false;

    //not yet implemented 
    return true;
}
