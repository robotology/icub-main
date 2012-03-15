// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup TheEthManager TheEthManager
 *
 * Implements <a href="http://eris.liralab.it/yarpdoc/d3/d5b/classyarp_1_1dev_1_1ICanBus.html" ICanBus interface <\a> for a cfw2 can bus device (cfw2 pc104 card). This is the eth2ems module
 * device.
 *
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/....h
 *
 */

//
// $Id: TheEthManager.h,v 1.5 2008/06/25 22:33:53 nat Exp $
//
//

#ifndef __ethManager__
#define __ethManager__

using namespace std;
//#include <stdlib.h>

#include <iostream>				// string type

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>

// ACE stuff
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

// debug with workstation
#define DEBUG_LAPTOP_IP			"10.255.37.155" 	// <- dhcp;
#define DEBUG_WORKSTATION_IP 	"10.255.37.24" // ip della workstation qui dietro.

// to use inside the robot
#define DEFAULT_PC104_IP		"10.0.0.1" 			// <- fix;
#define DEBUG_PC104_IP			"1.1.1.1" 			// <- fix;
#define DEBUG_EMS_IP			"10.0.0.2" 			// <- fix;

#define PC104_IP				DEBUG_LAPTOP_IP
#define	EMS_IP					DEBUG_EMS_IP
#define DEFAULT_PORT			33333


namespace yarp{
    namespace dev{
        class TheEthManager;
        class ethResources;
    }
}

class yarp::dev::ethResources: public DeviceDriver	// needed if I want to create this interface through the polydriver open (I think)
{
private:
	char msg[126];
	TheEthManager  *handle;
    static int		i;

    ACE_INET_Addr	remote_dev;
    ACE_INET_Addr	remote_broadcast;

public:
	ethResources();
	//ethResources(ACE_UINT16 loc_port, ACE_UINT32 loc_ip, ACE_UINT16 rem_port, ACE_UINT32 rem_ip);
    ~ethResources();
    virtual bool open(yarp::os::Searchable &par);
};


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

class yarp::dev::TheEthManager: public DeviceDriver
{
private:
    TheEthManager();
	static TheEthManager *handle;
    ACE_SOCK_Dgram_Bcast* mSocket;
    static int	i;

protected:
    ACE_INET_Addr	local_addr;


public:
    ~TheEthManager();

	char 		*data;
    PolyDriver polyDriver;
    static TheEthManager* instance();
    static TheEthManager* instance(ACE_INET_Addr local_addr);

    //Device Driver
    virtual bool open(yarp::os::Searchable &par);
    virtual bool initialize(yarp::os::Searchable &par);
    virtual bool close();
};



#endif
