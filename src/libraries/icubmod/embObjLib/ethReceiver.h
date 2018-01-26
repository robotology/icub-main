// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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

// - include guard ----------------------------------------------------------------------------------------------------

#ifndef _ETHRECEIVER_H_
#define _ETHRECEIVER_H_

// -- class EthReceiver
// -- it is a rate thread created by singleton TheEthManager.
// -- it regularly wakes up to see if a packet is in its listening socket and it parses that with methods made available by TheEthManager.

//#include <ethManager.h>

#include <ace/SOCK_Dgram.h>

#include <yarp/os/RateThread.h>


namespace eth {

    class TheEthManager;

    class EthReceiver : public yarp::os::RateThread
    {
    private:
        int rateofthread;

        ACE_SOCK_Dgram *recv_socket;
        eth::TheEthManager *ethManager;
        double statPrintInterval;


    public:

        enum { EthReceiverDefaultRate = 5, EthReceiverMaxRate = 20 };

        EthReceiver(int rxrate);
        ~EthReceiver();
        bool config(ACE_SOCK_Dgram *pSocket, eth::TheEthManager* _ethManager);
        bool threadInit();
        void run();
        void onStop();
    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







