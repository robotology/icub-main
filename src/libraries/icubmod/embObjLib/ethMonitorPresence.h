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

#ifndef _ETHMONITORPRESENCE_H_
#define _ETHMONITORPRESENCE_H_


#include <string>


namespace eth {
           
    class EthMonitorPresence
    {
    public:

        struct Config
        {
            bool    enabled;
            double  timeout;                // in seconds
            double  periodmissingreport;    // in seconds
            std::string name;
            Config() : enabled(true), timeout(0.020), periodmissingreport(30.0), name("generic") {}
            Config(bool en, double t, double p, const std::string& na) : enabled(en), timeout(t), periodmissingreport(p), name(na) {}
        };

        EthMonitorPresence();
        ~EthMonitorPresence();

        void config(const Config &cfg);
        void enable(bool en);
        bool isenabled();

        void tick();
        bool check();   // returns true if ok, false if missing

    private:

        Config configuration;

        double lastTickTime;
        double lastMissingReportTime;
        double lastHeardTime;
        bool reportedMissing;
    };


} // namespace eth


#endif  // include-guard


// - end-of-file (leave a blank line after)----------------------------------------------------------------------------







