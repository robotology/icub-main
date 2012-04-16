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
int ethResources::how_many = 0;

ethResources::ethResources()
{
	char tmp[126];
	transceiver = 0x00;
}

ethResources::~ethResources()
{
	char tmp[126];
	sprintf(tmp, "\nethResources::~ethResources() 1 handle= %p - i=%d", theEthManager_h, how_many);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);

}

bool ethResources::open(yarp::os::Searchable &config)
{
	ACE_TCHAR tmp[126], address[64];
	Bottle xtmp, xtmp2;
//	string str=config.toString().c_str();
	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	sprintf(tmp, "ethResources open\n");
	YARP_DEBUG(Logger::get(), tmp, Logger::get().log_files.f3);
	YARP_DEBUG(Logger::get(), config.toString().c_str(), Logger::get().log_files.f3);

    //
    // Get EMS ip addresses and port from config file, in order to correctly configure the transceiver.
    //
	// extract eth group info
	xtmp = Bottle(config.findGroup("ETH"));

	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	YARP_INFO(Logger::get(), String("IpAddress:\t") + address, Logger::get().log_files.f3);

	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);
	sprintf(tmp,"remote01.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), rem_ip1,rem_ip2,rem_ip3,rem_ip4);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f4);
	// Get EMS CmdPort from config file
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();
	// ACE format
	remote_dev.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

    //
	// Get PC104 ip address from config file
    //
	xtmp2 = config.findGroup("PC104IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	YARP_INFO(Logger::get(), String("PC104IpAddress:\t") + address, Logger::get().log_files.f3);
	// ACE format
	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
	sprintf(tmp,"pc104.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), loc_ip1,loc_ip2,loc_ip3,loc_ip4);
	YARP_INFO(Logger::get(),tmp, Logger::get().log_files.f4);

	ACE_INET_Addr loc_dev(rem_port, (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4);

	// Fill debug 'info' fields
	sprintf(info, "ethResources - referred to EMS: %d.%d.%d.%d\n", rem_ip1,rem_ip2,rem_ip3,rem_ip4);

	theEthManager_h = TheEthManager::instance(loc_dev);
	remote_dev.addr_to_string(address, 64);
	theEthManager_h->register_device(address, this);

	// Get the pointer to the actual Singleton ethManager, or create it if it's the first time.

	//
	//	EMBOBJ INIT
	//
	// Init  -- nel pc104 quale utilità hanno gli indirizzi ip utilizzati qui??
	//transceiver= new hostTransceiver;
	createProtocolHandler.open("hostTransceiver");
	createProtocolHandler.view(transceiver);
	transceiver->init(eo_common_ipv4addr(loc_ip1,loc_ip2,loc_ip3,loc_ip4), eo_common_ipv4addr(rem_ip1,rem_ip2,rem_ip3,rem_ip4), rem_port, EOK_HOSTTRANSCEIVER_capacityofpacket);

	// look through the config to know which features -E.P.- are required: motionControl, skin, analog... and create them
	// Clean device and subdevice fileds
	Property prop;
	string str=config.toString().c_str();
	xtmp = Bottle(config.findGroup("FEATURES"));
	prop.fromString(str.c_str());
	prop.unput("device");
	prop.unput("subdevice");
	// look for ethernet device driver to use and put it into the "device" field.
	Value &motionControl=xtmp.find("motionControl");
	strcpy(tmp, motionControl.asString().c_str());
	prop.put("device", motionControl.asString().c_str());
	createMotionControlHandler.open(prop);
	createMotionControlHandler.view(motionCtrl);
	motionCtrl->configureTransceiver(transceiver);


	return true;
}

int ethResources::send(void *data, size_t len)
{
	return TheEthManager::instance()->send(data, len, remote_dev);
}

void ethResources::getPack(uint8_t **pack, uint16_t *size)
{
	if (0 != transceiver)
		transceiver->getTransmit(pack, size);
}

void ethResources::setCalibrator(ICalibrator *icalibrator)
{
	motionCtrl->setCalibrator(icalibrator);
}

void ethResources::getControlCalibration(IControlCalibration2 **icalib)
{
	createMotionControlHandler.view(*icalib);

}

void ethResources::onMsgReception(uint8_t *data, uint16_t size)
{
	transceiver->onMsgReception(data, size);
}


ACE_INET_Addr	ethResources::getRemoteAddress()
{
	return	remote_dev;
}




// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


TheEthManager::TheEthManager() : RateThread(10 * 1000)
{
	char tmp[126];
	sprintf(tmp, "Error!!! Called constructor of ethResources without parameters!!! BTW, how can this be possible ???");
	YARP_FAIL(Logger::get(),tmp, Logger::get().log_files.f3);
}


TheEthManager::TheEthManager(ACE_INET_Addr local_addr) : RateThread(10)
{
	char tmp[126];
	sprintf(info, "TheEthManager");
	test = 0;
	socket = 0;
	deviceNum = 0;
	sprintf(tmp, "TheEthManager::TheEthManager()");
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);

	socket = new ACE_SOCK_Dgram();
	if (-1 == socket->open(local_addr) )
	{
		sprintf(tmp, "\n/---------------------------------------------------\\"
					 "\n|eStiketzi pensa che qualcosa non abbia funzionato!!|"
					"\n\\---------------------------------------------------/");
		YARP_ERROR(Logger::get(),tmp, Logger::get().log_files.f3);
		YARP_FAIL(Logger::get(),tmp);
	}
	//socket->enable(ACE_NONBLOCK);
}

TheEthManager *TheEthManager::instance(ACE_INET_Addr local_addr)
{
	if (i == 0)
	{
		handle = new TheEthManager(local_addr);
		// move the start during / right before the calibration command??
		handle->start();
	}
	i++;
	return handle;
}

TheEthManager *TheEthManager::instance()
{
	return handle;
}

TheEthManager::~TheEthManager()
{
	char tmp[126];
	sprintf(tmp, "TheEthManager::~TheEthManager() destructor call; handle= %p - i=%d", handle, i);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
	i--;
	if (i == 0 )
	{
		sprintf(tmp, "TheEthManager::~TheEthManager() - real destruction happens here handle= %p - i=%d", handle, i);
		YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
		socket->close();
		delete handle;
	}
}

bool TheEthManager::register_device(ACE_TCHAR *new_dev_id, ethResources *new_dev_handler)
{
	// put this device handler in a list somehow
	DeviceEntry	dev;
	strcpy(dev.id, "0");
	dev.resource = 0x00;


	deviceList.resize(deviceNum +1);
	int t=deviceList.size();
	strcpy(deviceList[deviceNum].id, new_dev_id);
	deviceList[deviceNum].resource = new_dev_handler;
	deviceNum++;


	for (int i=0; i<t; i++)
		dev=deviceList[i];
	return true;
}

int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
	return socket->send(data,len,remote_addr);
}


void TheEthManager::run()
{
	size_t 			n = MAX_RECV_SIZE;
	ACE_TCHAR 		address[64];
	ethResources	*ethRes;

	// per ogni msg ricevuto
	recv_size = socket->recv((void *) incoming_msg, n, sender_addr, 0);

    sender_addr.addr_to_string(address, 64);
	printf("Received new packet from address %s, size = %d\n", address, recv_size);
	int nDev = deviceList.size();
	if( recv_size > 0)
	{
		for( int i=0; i<nDev; i++)
		{
			if(strcmp(deviceList[i].id, address) == 0)
			{
				ethRes = deviceList[i].resource;
				// come fare queste chiamate in parallelo, non "bloccanti" e magari togliere la memcpy?
				memcpy(ethRes->recv_msg, incoming_msg, recv_size);
				ethRes->onMsgReception(ethRes->recv_msg, recv_size);
				//continue; // to skip remaining part of the for cycle
			}
		}
	}

	//if( (test % 1000) == 0)
	{
		YARP_DEBUG(Logger::get(), "TheEthManager:run()" , Logger::get().log_files.f3);
		test = 0;

		for( int i=0; i<deviceList.size(); i++)
		{
			data = 0;
			size = 0;
			deviceList[i].resource->getPack(&data, &size);
			ACE_INET_Addr addr = deviceList[i].resource->getRemoteAddress();
			this->send(data, size, addr);
		}
	}
	test++;


}

// Probably useless
bool TheEthManager::open()
{
	char tmp[126];
	sprintf(tmp, "TheEthManager open - handle= %p - i=%d", handle, i);

	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
	return true;
}

// Probably useless
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
