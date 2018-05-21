// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Alberto Cardellino, Marco Accame
 * email:   alberto.cardellino@iit.it, marco.accame@iit.it
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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "ethReceiver.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------

#include <yarp/os/Network.h>
#include <yarp/os/NetType.h>

//#include <yarp/os/SystemClock.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
using yarp::os::Log;

#include "ethManager.h"
#include "ethResource.h"


// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


// - class eth::EthReceiver

using namespace eth;



EthReceiver::EthReceiver(int raterx): RateThread(raterx)
{
    rateofthread = raterx;
    yDebug() << "EthReceiver is a RateThread with rxrate =" << rateofthread << "ms";
    // ok, and now i get it from xml file ... if i find it.

//    std::string tmp = NetworkBase::getEnvironment("ETHSTAT_PRINT_INTERVAL");
//    if (tmp != "")
//    {
//        statPrintInterval = (double)NetType::toInt(tmp);
//    }
//    else
//    {
//        statPrintInterval = 0.0;
//    }
}

void EthReceiver::onStop()
{
    // in here i send a small packet to ... myself ?
    uint8_t tmp = 0;
    ethManager->sendPacket( &tmp, 1, ethManager->getLocalIPV4addressing());
}

EthReceiver::~EthReceiver()
{

}

bool EthReceiver::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    recv_socket = pSocket;
    ethManager  = _ethManager;

    ACE_HANDLE sockfd = pSocket->get_handle();
    int retval;
    int32_t mysize = 1024*1024;     // 1Mb note:actually kernel uses memory with size doblem of mysize
                                    // with this size i'm sure ems pkts are not lost
    int len = sizeof(mysize);

    // the user can change buffer size by environment variable ETHRECEIVER_BUFFER_SIZE
    std::string _dgram_buffer_size = NetworkBase::getEnvironment("ETHRECEIVER_BUFFER_SIZE");
    if (_dgram_buffer_size!="")
        mysize = NetType::toInt(_dgram_buffer_size);

    retval = ACE_OS::setsockopt  (sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&mysize, sizeof(mysize));
    if (retval != 0)
    {
        int myerr = errno;
        yError()<< "ERROR in SetSockOpt SO_RCVBUF";
    }

    int32_t sock_input_buf_size = 0;
    retval = ACE_OS::getsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, (char *)&sock_input_buf_size, &len);
    if (retval != 0)
    {
        int myerr = errno;
        yError() << "ERROR inGetSockOpt SO_RCVBUF";
    }

    yWarning() << "in EthReceiver::config() the config socket has queue size = "<< sock_input_buf_size<< "; you request ETHRECEIVER_BUFFER_SIZE=" << _dgram_buffer_size;

    return true;
}


bool EthReceiver::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#if defined(__unix__)
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (< 50)
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2; // = 49
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif

    return true;
}


uint64_t getRopFrameAge(char *pck)
{
    return(eo_ropframedata_age_Get((EOropframeData*)pck));
}



void EthReceiver::run()
{
    ssize_t       incoming_msg_size = 0;
    ACE_INET_Addr sender_addr;
    uint64_t      incoming_msg_data[TheEthManager::maxRXpacketsize/8];   // 8-byte aligned local buffer for incoming packet: it must be able to accomodate max size of packet
    const ssize_t incoming_msg_capacity = TheEthManager::maxRXpacketsize;

    int flags = 0;
#ifndef WIN32
    flags |= MSG_DONTWAIT;
#endif

    static uint8_t earlyexit_prev = 0;
    static uint8_t earlyexit_prevprev = 0;

    // marco.accame: set maxUDPpackets as a fixed minimum number (2) + the number of boards. all is multipled by rateofthread and by a gain which depends on past activity
    // the gain on past activity is usually 1. if maxUDPpackets is not enough the gain becomes (1+f1). if again it is not enough, the gain becomes 1+f1+f2+f3 and stays as
    // such until it is enough. at this time it becomes 1+f2 and then 1 again
    const double f1 = 0.5;
    const double f2 = 0.5;
    const double f3 = 8.0;
    double gain = 1.0 + f1*(1-earlyexit_prev) + f2*(1-earlyexit_prevprev) + f3*(1-earlyexit_prev)*(1-earlyexit_prevprev);
    int maxUDPpackets = (2 + ethManager->getNumberOfResources()) * EthReceiver::rateofthread *  gain;


    earlyexit_prevprev = earlyexit_prev;    // save previous early exit
    earlyexit_prev = 0;                     // consider no early exit this time

    for(int i=0; i<maxUDPpackets; i++)
    {
        incoming_msg_size = recv_socket->recv((void *) incoming_msg_data, incoming_msg_capacity, sender_addr, flags);
        if(incoming_msg_size <= 0)
        { // marco.accame: i prefer using <= 0.
            earlyexit_prev = 1; // yes, we have an early exit
            break; // we break and do not return because we want to be sure to execute what is after the for() loop
        }

        // we have a packet ... we give it to the ethmanager for it parsing
        //bool collectStatistics = (statPrintInterval > 0) ? true : false;
        ethManager->Reception(ethManager->toipv4addr(sender_addr), incoming_msg_data, incoming_msg_size);
    }

    // execute the check on presence of all eth boards.
    ethManager->CheckPresence();
}



// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





