// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 Robotcub Consortium
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */
///
/// $Id: TheEthManager.cpp,v 1.9 2008/03/08 10:07:01 babybot Exp $
///

//#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Bottle.h>
#include <string.h>
#include <iostream>
#include <stdio.h>

#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>

#include "ethManager.h"

#include <yarp/os/Log.h>
#include <yarp/os/impl/Logger.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

TheEthManager* TheEthManager::handle = 0x00;
int TheEthManager::i = 0;
int ethResources::i = 0;

ethResources::ethResources()
{
	char tmp[126];
	sprintf(tmp, "Error!!! Called constructor of ethResources without parameters!!!");
//	YARP_FAIL(Logger::get(),tmp, Logger::get().log_files.f3);
}

/*
ethResources::ethResources(ACE_UINT16 loc_port, ACE_UINT32 loc_ip, ACE_UINT16 rem_port, ACE_UINT32 rem_ip)
{
	char tmp[126];
	sprintf(tmp, "\n\n ethResources::ethResources() 1 handle= 0x%06X - i=%d, this: 0x%06X", handle, i, this);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f3);

    remote_dev.set(rem_port, rem_ip);
    remote_broadcast.set(rem_port, rem_ip | 0x0000FFFF);

    ACE_INET_Addr loc_dev(loc_port, loc_ip);
	handle = TheEthManager::instance(loc_dev);
}
*/

ethResources::~ethResources()
{
	char tmp[126];
	sprintf(tmp, "\nethResources::~ethResources() 1 handle= 0x%06X - i=%d", handle, i);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);

}

bool ethResources::open(yarp::os::Searchable &config)
{
	char tmp[126], address[16];;
	Bottle xtmp, xtmp2;
	string str=config.toString().c_str();
	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	sprintf(tmp, "ethResources open - handle= 0x%06X - i=%d", handle, i);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);

	YARP_INFO(Logger::get(), config.toString().c_str(), Logger::get().log_files.f3);

	// Get EMS ip address from config file
	xtmp = Bottle(config.findGroup("ETH"));

	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	YARP_INFO(Logger::get(), String("IpAddress:\t") + address, Logger::get().log_files.f3);
	// ACE format
	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);
	sprintf(tmp,"remote01.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), rem_ip1,rem_ip2,rem_ip3,rem_ip4);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f4);

	// Get EMS CmdPort from config file
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();
	remote_dev.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

	// Get PC104 ip address from config file
	xtmp2 = config.findGroup("PC104IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	YARP_INFO(Logger::get(), String("PC104IpAddress:\t") + address, Logger::get().log_files.f3);
	// ACE format
	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
	sprintf(tmp,"pc104.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), loc_ip1,loc_ip2,loc_ip3,loc_ip4);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f4);

	// Get EMS CmdPort from config file
//	xtmp2 = xtmp.findGroup("CmdPort");
//	rem_port = xtmp2.get(1).asInt();
	ACE_INET_Addr loc_dev(rem_port, (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4);


/*
	sprintf(tmp, "%s:%d", xtmp2.get(0).asString().c_str(), port);
	YARP_INFO(Logger::get(), tmp, Logger::get().log_files.f4);

	sscanf(loc_ip_string.c_str(),"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
	sprintf(tmp, "local.address: %s, %d:%d:%d:%d\n", loc_ip_string.c_str(), loc_ip1,loc_ip2,loc_ip3,loc_ip4);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f4);
	ACE_UINT32 loc_ip_ACE = (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4;
*/


	// Init transceiver -- nel pc104 quale utilità hanno gli indirizzi ip utilizzati qui??
	//hostTransceiver_Init(eo_common_ipv4addr(10,0,0,1), eo_common_ipv4addr(rem_ip1,rem_ip2,rem_ip3,rem_ip4), port, 512);

	//	udppkt_data = NULL;
	//	udppkt_size = 0;

	// Now this will raise an error if the ethResources constructor is called without the parameters
	//handle = new ethResources;

	//	Create the ethResources class which will call the TheEthManager singleton and store it's handler
	//resource = new ethResources(loc_port,	loc_ip_ACE, rem_port, rem_ip_ACE);


	// Get the pointer to the actual Singleton ethManager, or create it if it's the first time.
//	ACE_INET_Addr loc_dev(loc_port, loc_ip);
	handle = TheEthManager::instance(loc_dev);
	//handle = TheEthManager::instance();

	return true;
}

// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

TheEthManager *TheEthManager::instance(ACE_INET_Addr local_addr)
{
	char tmp[126];
	sprintf(tmp, "\n\nTheEthManager::instance() handle= 0x%06X - i=%d", handle, i);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f3);

	if (i == 0)
	{
		handle = new TheEthManager;
//		i++;
	}
	i++;

	return handle;
}

TheEthManager *TheEthManager::instance()
{
	return handle;
}

TheEthManager::TheEthManager()
{
	char tmp[126];
	int i;

	sprintf(tmp, "TheEthManager::TheEthManager() 1 handle= 0x%06X - i=%d, this: 0x%06X", handle, i, this);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);


// not needed here. This place knows only about ace and nothing about the payload. It can be EmbObj or everything else!
//	eOipv4addr_t pc104_addr = eo_common_ipv4addr(loc_ip1,loc_ip2,loc_ip3,loc_ip4);
//	eOipv4port_t port = DEFAULT_EMS_PORT;

	//  CREATE THE SOCKET - Si può definire/aprire più volte? -> no!  usare singleton
#warning  " How to check if address has been correctly assigned?? Annoying error from ACE"
 //   ACE_INET_Addr ace_addr(loc_port, loc_ip_ACE);
//	ACE_INET_Addr ace_addr(loc_port, loc_ip_string.c_str());
//    mSocket=new ACE_SOCK_Dgram_Bcast(ace_addr);
}

TheEthManager::~TheEthManager()
{
	char tmp[126];
	sprintf(tmp, "TheEthManager::~TheEthManager() 1 handle= 0x%06X - i=%d", handle, i);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
	i--;
	if (i == 0 )
	{
		sprintf(tmp, "TheEthManager::~TheEthManager() 2 handle= 0x%06X - i=%d", handle, i);
		YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
		delete handle;
	}
}

bool TheEthManager::open(yarp::os::Searchable &par)
{
	char tmp[126];
	sprintf(tmp, "TheEthManager open - handle= 0x%06X - i=%d", handle, i);

	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
	//new TheEthManager();

	return true;
}

bool TheEthManager::initialize(yarp::os::Searchable &par)
{
	YARP_INFO(Logger::get(),"TheEthManager::initialize()", Logger::get().log_files.f3);
	return true;
}

bool TheEthManager::close()
{
	YARP_INFO(Logger::get(),"TheEthManager::close()", Logger::get().log_files.f3);
	fflush(stdout);
	return true;
}
