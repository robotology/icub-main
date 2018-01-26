// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
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

#ifndef _THENVMANAGER_H_
#define _THENVMANAGER_H_


#include <vector>
#include <cstdint>

#include "EoProtocol.h"
#include <hostTransceiver.hpp>

namespace eth {
           
    class theNVmanager
    {
    public:
        static theNVmanager& getInstance();
                
    public:

        enum class ropCode { sig = eo_ropcode_sig, say = eo_ropcode_say };
        
        // value and values[i] must point to memory with enough space to host the reply. the ask() functions will just copy the reply
        // into these memory locations, hence the user must pre-allocate enough memory before calling ask()
        // but what must be the size of the memory? well, it depends on {ipv4, id32}. in any case, it is upper bounded.

        // tells if a given ip address is supported
        bool supported(const eOprotIP_t ipv4);

        // tells if a given ip address + network variable is supported
        bool supported(const eOprotIP_t ipv4, const eOprotID32_t id32);

        // tells the size in bytes of a given network variable. its size is independent from the ip address
        size_t sizeOfNV(const eOprotID32_t id32);

        // it returns true only when the remote board replies to the request of a given network variable.
        // the varible has id32 = eoprot_ID_get(eoprot_endpoint_management, eoprot_entity_mn_comm, 0, eoprot_tag_mn_comm_status_managementprotocolversion)
        bool ping(const eOprotIP_t ipv4, eoprot_version_t &mnprotversion, const double timeout = 0.5, const unsigned int retries = 20);


        // management of a single variable at a time: ask(), set(), check(), setcheck()

        // it sends a ask<> ROP to a single network variable and waits the say<> reply ROP until timeout.
        // result is in value, which must be a buffer with at least sizeOfNV(id32) bytes
        bool ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, const double timeout = 0.5);
        bool ask(eth::HostTransceiver *t, const eOprotID32_t id32, void *value, const double timeout = 0.5);
        // imposes a value to a given network variable. it does not wait nor verify
        bool set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value);
        bool set(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value);
        // it asks the value of a single network variable and checks vs a given value which points to a buffer of at least sizeOfNV(id32) bytes
        bool check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const double timeout = 0.5, const unsigned int retries = 0);
        bool check(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const double timeout = 0.5, const unsigned int retries = 0);
        // it sends set<> ROP to a given varaible and it checks that the value is really written. it repeats this cycle until done, at most retries + 1 times.
        bool setcheck(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const unsigned int retries = 10, double waitbeforecheck = 0.001, double timeout = 0.5);
        bool setcheck(eth::HostTransceiver *t, const eOprotID32_t id32, const void *value, const unsigned int retries = 10, double waitbeforecheck = 0.001, double timeout = 0.5);

        // function which must be placed in the reception handlers to unblock the waiting of replies from a given board
        bool onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature);

        // function used to wait for a given ROP: ropcode<ipv4, id32, value>
        // its main use is: set(id32command); wait(sig); read(value)
        // we could use a compact function such as; command(ipv4, setid32, setvalue, sigid32, sigvalue, timeout);
        // maybe we also use signature? with: bool wait(const ropCode ropcode, const std::uint32_t signature, const double timeout = 0.5);
//        bool wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout);
        // function used to simply read the locally cached network variable.
//        bool read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value);

        // it sends a set<id32com, command>, waits for a sig<id32rep, reply> with a timeout, reads reply.
        bool command(const eOprotIP_t ipv4, const eOprotID32_t id32cmd, const void *cmd, const eOprotID32_t id32rep, void *rep, double timeout = 0.5);


        // for future use: ask of multiple values on the same ipv4 board.


        // sends read parallel requests for many network variables to the same ip address and waits
        bool ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout = 0.5);
        bool ask(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values, const double timeout = 0.5);


        // tobedone: i want to group several requests before i start to wait.
        // i need:
        // - group_start() which tells the nvmanager to use a given signature for all successive group_add_ask() until group_add_stop()
        // - group_ask() which adds to the nvmanager a vector of ask rops identified by the same signature.
        //   this function can be repeated as many times one want. typically by different devices but by the same thread.
        // - group_stop() which starts the wait for the replies. when this function returns, the various values vector will contain
        //   the replies.
        // some more explanation:
        // we shall use the same signature if the calling thread is the same as the one which called group_start().
        // in this way, we can target the operation to a given thread without blocking other threads using normal ask() requests. .
        // i would say however, that if another thread starts a group_start() during an active session .... it musts wait.
        // also let's give some limitations:
        // start of proper ask sending is done by group_stop() and not directly by group_ask().
        // we can have at most one active session at any time.
        // we can call group_ask() at most ... 8 times (boh, maybe it is not necessary to give a limit).
        // the calling thread will be retrieved inside.
        // i use a ACE_Recursive_Thread_Mutex to perform synch amongst different threads.

        bool group_start();
        bool group_ask(eth::HostTransceiver *t, const std::vector<eOprotID32_t> &id32s, const std::vector<void*> &values);
        bool group_stop(const double timeout = 0.5);

    private:
        theNVmanager(); 

    public:
        // remove copy constructors and copy assignment operators
        theNVmanager(const theNVmanager&) = delete;
        theNVmanager(theNVmanager&) = delete;
        void operator=(const theNVmanager&) = delete;
        void operator=(theNVmanager&) = delete;

    private:    
        struct Impl;
        Impl *pImpl;        
    };       


} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


