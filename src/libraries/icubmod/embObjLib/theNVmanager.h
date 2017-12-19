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

namespace tbd {
           
    class theNVmanager
    {
    public:
        static theNVmanager& getInstance();
                
    public:
        struct Config
        {
            std::uint8_t    param1;
            Config() :  
                param1(0)
                {}
            Config(std::uint8_t _p1) : 
                param1(_p1)
                {}    
        }; 

        enum class ropCode { sig = eo_ropcode_sig, say = eo_ropcode_say };
        
        
        bool initialise(const Config &config);

        bool ask(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size, const double timeout = 0.5);

        bool ask(const eOprotIP_t ipv4, const std::vector<eOprotID32_t> &id32s, std::vector<void*> &values, std::vector<std::uint16_t> &sizes , const double timeout = 0.5);

        bool set(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size);

        bool setuntilverified(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, int retries = 10, double waitbeforeverification = 0.001, double verificationtimeout = 0.050, int verificationretries = 2);

        bool verify(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, const double timeout = 0.5, const int retries = 3);


        bool onarrival(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const std::uint32_t signature);

        //bool wait(const ropCode ropcode, const std::uint32_t signature, const double timeout = 0.5);
        bool wait(const ropCode ropcode, const eOprotIP_t ipv4, const eOprotID32_t id32, const double timeout = 0.5);

        bool read(const eOprotIP_t ipv4, const eOprotID32_t id32, void *value, std::uint16_t &size);

        // the thread locks the object
        bool parallel_ask_start();
        // it adds as many requests are it likes. it adds ip, vector<id>, vector<data*> which are added in vector<ips> and vector<vector<id>> vector<vector<data*>>
        bool parallel_ask_add();
        // and finally the request all start. they all have the same signature. the thread is locked until
        // it receives all the replies. results are in previous vectors
        bool parallel_ask_wait(const double timeout = 0.5);


        bool check(const eOprotIP_t ipv4, const eOprotID32_t id32, const void *value, const std::uint16_t size, const double timeout = 0.5);

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


} // namespace tbd


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------


