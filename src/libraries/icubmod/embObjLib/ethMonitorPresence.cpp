// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


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


// --------------------------------------------------------------------------------------------------------------------
// - public interface
// --------------------------------------------------------------------------------------------------------------------

#include "ethMonitorPresence.h"



// --------------------------------------------------------------------------------------------------------------------
// - external dependencies
// --------------------------------------------------------------------------------------------------------------------


#include <yarp/os/SystemClock.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
using yarp::os::Log;


// --------------------------------------------------------------------------------------------------------------------
// - pimpl: private implementation (see scott meyers: item 22 of effective modern c++, item 31 of effective c++
// --------------------------------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------------------------------
// - the class
// --------------------------------------------------------------------------------------------------------------------


// - class eth::EthMonitorPresence

eth::EthMonitorPresence::EthMonitorPresence()
{
    lastTickTime = 0;
    lastMissingReportTime = 0;
    lastHeardTime = 0;
    reportedMissing = false;
}


eth::EthMonitorPresence::~EthMonitorPresence()
{

}


void eth::EthMonitorPresence::config(const Config &cfg)
{
    configuration = cfg;

    if(true == configuration.enabled)
    {
        yDebug() << "eth::EthMonitorPresence::config(): monitoring of presence is ON for BOARD" << configuration.name << "with timeout =" << configuration.timeout << "sec and period of missing report =" << configuration.periodmissingreport << "sec";
    }
    else
    {
        yDebug() << "eth::EthMonitorPresence::config(): monitoring of presence is OFF for BOARD" << configuration.name;
    }
}


void eth::EthMonitorPresence::enable(bool en)
{
    configuration.enabled = en;
}


bool eth::EthMonitorPresence::isenabled()
{
    return configuration.enabled;
}


void eth::EthMonitorPresence::tick()
{
    lastTickTime = yarp::os::SystemClock::nowSystem();
}


bool eth::EthMonitorPresence::check()
{

    if(false == configuration.enabled)
    {
        return true;
    }


    double tnow = yarp::os::SystemClock::nowSystem();

    if((true == reportedMissing) && (lastTickTime > 0))
    {
        yDebug() << "eth::EthMonitorPresence: BOARD" << configuration.name << "has shown after being lost for" << tnow - lastHeardTime << "sec";
        reportedMissing = false;
        lastHeardTime = tnow;
    }

    if((true == reportedMissing) && (configuration.periodmissingreport > 0))
    {
        // i report the board is still missing. but at a smaller rate
        if((tnow - lastMissingReportTime) >= configuration.periodmissingreport)
        {
            yDebug() << "eth::EthMonitorPresence: BOARD" << configuration.name << "has been silent for another" << tnow - lastMissingReportTime << "sec, for a total of" << tnow - lastHeardTime << "sec";
            lastMissingReportTime = tnow;
        }
        return false;
    }


    // check vs the target timeout
    double delta = tnow - lastTickTime;

    if(delta > configuration.timeout)
    {
        yDebug() << "eth::EthMonitorPresence: BOARD" << configuration.name << "has been silent for" << delta << "sec (its timeout is" << configuration.timeout << "sec)";

        // also: mark the board as lost.
        lastMissingReportTime = tnow;
        reportedMissing = true;
        lastHeardTime = lastTickTime;
        lastTickTime = -1;

        return false;
    }

    // we have the board and we hard of it by its timeout
    return true;
}



// - end-of-file (leave a blank line after)----------------------------------------------------------------------------





