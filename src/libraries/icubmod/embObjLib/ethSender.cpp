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

#include "ethSender.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


//#include <yarp/os/SystemClock.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
using yarp::os::Log;

#include "ethManager.h"


// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


// - class eth::EthSender

// -- class EthSender
// -- here is it code

using namespace eth;

EthSender::EthSender(int txrate) : RateThread(txrate)
{
    rateofthread = txrate;
    yDebug() << "EthSender is a RateThread with txrate =" << rateofthread << "ms";
    yTrace();
}

EthSender::~EthSender()
{

}

bool EthSender::config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager)
{
    yTrace();
    send_socket = pSocket;
    ethManager  = _ethManager;

    return true;
}

bool EthSender::threadInit()
{
    yTrace() << "Do some initialization here if needed";

#if defined(__unix__)
    /**
     * Make it realtime (works on both RT and Standard linux kernels)
     * - increase the priority upto the system IRQ's priorities (50) and less than the receiver thread
     * - set the scheduler to FIFO
     */
    struct sched_param thread_param;
    thread_param.sched_priority = sched_get_priority_max(SCHED_FIFO)/2 - 1; // = 48
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
#endif

    return true;
}



void EthSender::run()
{
    // by calling this metod of ethManager, we put protection vs concurrency internal to the class.
    // for tx we must protect the EthResource not being changed. they can be changed by a device such as
    // embObjMotionControl etc which adds or releases its resources.
    ethManager->Transmission();
}


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





