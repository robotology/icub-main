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

#ifndef _ETHSENDER_H_
#define _ETHSENDER_H_

// -- class EthSender
// -- it is a rate thread created by singleton TheEthManager. it regularly transmits packets (if any available) to the eth boards.
// -- it uses methods made available by TheEthManager.

//#include <ethManager.h>

#include <ace/SOCK_Dgram.h>

#include <yarp/os/RateThread.h>


namespace eth {

    class TheEthManager;

    class EthSender : public yarp::os::RateThread
    {
    private:
        int rateofthread;

        uint8_t *p_sendData;
        TheEthManager *ethManager;
        ACE_SOCK_Dgram *send_socket;
        void run();


    public:

        enum { EthSenderDefaultRate = 1, EthSenderMaxRate = 20 };

        EthSender(int txrate);
        ~EthSender();
        bool config(ACE_SOCK_Dgram *pSocket, TheEthManager* _ethManager);
        bool threadInit();

    };

} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







