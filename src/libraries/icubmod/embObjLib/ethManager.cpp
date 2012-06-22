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

bool keepGoingOn2 = true;



ethResCreator* ethResCreator::handle = 0x00;
bool ethResCreator::initted = false;

TheEthManager* TheEthManager::handle = 0x00;
int TheEthManager::i = false;


ethResources::ethResources()
{
	YARP_INFO(Logger::get(), "ethResources::ethResources()", Logger::get().log_files.f3);
	char tmp[126];
	transceiver = 0x00;
	theEthManager_h = NULL;

	udppkt_data = 0x00;
	udppkt_size = 0;

	// Protocol handlers
	//createProtocolHandler = 0x00;					// for some reason polydrivers cannot be initialized
	transceiver = 0x00;

	// Motion control handlers
	//createMotionControlHandler = 0x00;
	//motionCtrl = 0x00;

	//createSkinHandler = 0x00;
	//createAnalogHandler = 0x00;
}

ethResources::~ethResources()
{
	char tmp[126];
	sprintf(tmp, "\nethResources::~ethResources() 1 handle= %p ", theEthManager_h);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
}



bool ethResources::open(yarp::os::Searchable &config)
{
#warning "pick up the right board number: now using 4th number of ip address"
	uint8_t board_n = -1;
	ACE_TCHAR tmp[126]; //, address[64];
	Bottle xtmp, xtmp2;
	//	string str=config.toString().c_str();
	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	YARP_INFO(Logger::get(), "ethResources::open()", Logger::get().log_files.f3);

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
	// Fill 'info' and ID fields
	//
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "ethResources - referred to EMS: %d.%d.%d.%d", rem_ip1,rem_ip2,rem_ip3,rem_ip4);
	id.ip1=rem_ip1;
	id.ip2=rem_ip2;
	id.ip3=rem_ip3;
	id.ip4=rem_ip4;
	strcpy(id.name, "Dunno");


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

	theEthManager_h = TheEthManager::instance(loc_dev);
	remote_dev.addr_to_string(address, 64);
	theEthManager_h->register_device(address, this);


	// Get the pointer to the actual Singleton ethManager, or create it if it's the first time.

	//
	//	EMBOBJ INIT
	//
	// Init  -- nel pc104 quale utilità hanno gli indirizzi ip utilizzati qui??
	//transceiver= new hostTransceiver;
	transceiver= new hostTransceiver;
	//	createProtocolHandler.open("hostTransceiver");
	//	createProtocolHandler.view(transceiver);

	transceiver->init(eo_common_ipv4addr(loc_ip1,loc_ip2,loc_ip3,loc_ip4), eo_common_ipv4addr(rem_ip1,rem_ip2,rem_ip3,rem_ip4), rem_port, EOK_HOSTTRANSCEIVER_capacityofpacket, rem_ip4);

	// look through the config to know which features -E.P.- are required: motionControl, skin, analog... and create them
	// Clean device and subdevice fileds
#if 0
	Property prop;
	string str=config.toString().c_str();
	xtmp = Bottle(config.findGroup("FEATURES"));
	prop.fromString(str.c_str());
	prop.unput("device");
	prop.unput("subdevice");
	// look for Ethernet device driver to use and put it into the "device" field.
	Value &motionControl=xtmp.find("motionControl");
	strcpy(tmp, motionControl.asString().c_str());
	prop.put("device", motionControl.asString().c_str());

	createMotionControlHandler.open(prop);
	createMotionControlHandler.view(motionCtrl);
	motionCtrl->configureTransceiver(transceiver);

	//motionCtrl.open(prop);


	IPidControl       		*pipid;
	IPositionControl       	*popod;
	IVelocityControl		*vevel;
	// motionCtrl->view(pipid);
	// motionCtrl->view(popod);
	// motionCtrl->view(vevel);

	pipid = (IPidControl *) motionCtrl;
	Pid p;
	view(popod);
	view(vevel);
#endif
	return true;
}

// ???? da eliminare...
int ethResources::send(void *data, size_t len)
{
	return TheEthManager::instance()->send(data, len, remote_dev);
}

void ethResources::getPack(uint8_t **pack, uint16_t *size)
{
	if (0 != transceiver)
		transceiver->getTransmit(pack, size);
}


#if 0
void ethResources::setCalibrator(ICalibrator *icalibrator)
{
	motionCtrl->setCalibrator(icalibrator);
}

void ethResources::getControlCalibration(IControlCalibration2 **icalib)
{
	createMotionControlHandler.view(*icalib);

}

void ethResources::getMotorController(DeviceDriver **iMC)
{
	*iMC =  this->motionCtrl;
}
#endif

void ethResources::onMsgReception(uint8_t *data, uint16_t size)
{
	transceiver->onMsgReception(data, size);
}

ACE_INET_Addr	ethResources::getRemoteAddress()
{
	return	remote_dev;
}

// -------------------------------------------------------------------\\
//            ethResCreator   Singleton
// -------------------------------------------------------------------\\

ethResCreator::ethResCreator()
{
	how_many_boards = 0;
}

ethResCreator::~ethResCreator()
{
	ethResIt iterator = this->begin();
	while(iterator != this->end())
	{
		delete (*iterator);
		iterator++;
	}
}

ethResCreator *ethResCreator::instance()
{
	if (initted == false)
		handle = new ethResCreator();

	initted = true;
	return handle;
}

ethResources* ethResCreator::getResource(yarp::os::Searchable &config)
{
	ACE_TCHAR tmp[126], address[64];
	Bottle xtmp, xtmp2;
	//	string str=config.toString().c_str();
	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	YARP_INFO(Logger::get(), " ethResCreator::getResource", Logger::get().log_files.f3);

	//
	// Get EMS ip addresses from config file, to see if we need to instantiate a new Resources or simply return
	//	a pointer to an already existing object
	//

	xtmp = Bottle(config.findGroup("ETH"));
	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	sprintf(tmp,"EMS IpAddress %s", address);
	YARP_INFO(Logger::get(), tmp, Logger::get().log_files.f3);

	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);
	sprintf(tmp,"remote01.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), rem_ip1,rem_ip2,rem_ip3,rem_ip4);
	// Get EMS CmdPort from config file
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();

	// ACE format
	ACE_INET_Addr tmp_addr;
	tmp_addr.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

	EMS_ID id;
	id.ip1=rem_ip1;
	id.ip2=rem_ip2;
	id.ip3=rem_ip3;
	id.ip4=rem_ip4;


	ethResources *newRes = NULL;
	ethResIt iterator = this->begin();

	while(iterator!=this->end())
	{
		if(this->compareIds(id, (*iterator)->id))
			//if(tmp_addr == (*iterator)->getRemoteAddress() )
		{
			// device already exist.
			YARP_INFO(Logger::get(), String("device already exist\n") + address, Logger::get().log_files.f3);

			newRes = (*iterator);
		}
		iterator++;
	}

	if ( NULL == newRes)
	{
		// device doesn't exist yet, create it
		YARP_INFO(Logger::get(), String("device doesn't exist yet, create it\n") + address, Logger::get().log_files.f3);
		newRes = new ethResources;
		newRes->open(config);
		how_many_boards++;
		this->push_back(newRes);
	}
	return newRes;
}

bool	ethResCreator::compareIds(EMS_ID id2beFound, EMS_ID nextId)
{
	if( (id2beFound.ip1 == nextId.ip1) && (id2beFound.ip2 == nextId.ip2) && (id2beFound.ip3 == nextId.ip3) && (id2beFound.ip4 == nextId.ip4) )
		return true;
	else
		return false;
}


uint8_t* ethResCreator::find(EMS_ID &id)
{
	YARP_INFO(Logger::get(), " ethResCreator::find", Logger::get().log_files.f3);

	ethResources *res = NULL;
	ethResIt iterator = this->begin();

	while(iterator!=this->end())
	{
		if(this->compareIds(id, (*iterator)->id))
		{
			// device found
			res = (*iterator);
			break;
		}
		iterator++;
	}
	//res->
}


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


SendThread::SendThread() : RateThread(1)

{
	YARP_INFO(Logger::get(), "SendThread::SendThread()", Logger::get().log_files.f3);
}

SendThread::~SendThread()
{

}

bool SendThread::threadInit()
{
	ethResList = ethResCreator::instance();
	return true;
}


void SendThread::run()
{
	iterator = ethResList->begin();

	while(iterator!=ethResList->end())
	{
		data = 0;
		size = 0;
		(*iterator)->getPack(&data, &size);
		ACE_INET_Addr addr = (*iterator)->getRemoteAddress();
		TheEthManager::instance()->send(data, (size_t)size, addr);
		iterator++;
	}
}



#ifdef _SEPARETED_THREADS_
TheEthManager::TheEthManager()
#else
TheEthManager::TheEthManager() : RateThread(1000)
#endif
{
	char tmp[126];
	sprintf(tmp, "Error!!! Called constructor of ethResources without parameters!!! BTW, how can this be possible ???");
	YARP_FAIL(Logger::get(),tmp, Logger::get().log_files.f3);
}

#ifdef _SEPARETED_THREADS_
TheEthManager::TheEthManager(ACE_INET_Addr local_addr)
#else
TheEthManager::TheEthManager(ACE_INET_Addr local_addr) : RateThread(1)
#endif
{
	char tmp[SIZE_INFO];
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "TheEthManager");

	test = 0;
	deviceNum = 0;
	//_socket = 0;
	sprintf(tmp, "TheEthManager::TheEthManager()");

	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);
	local_addr.addr_to_string(tmp, 64);
	YARP_DEBUG(Logger::get(),tmp, Logger::get().log_files.f3);

	createSocket(local_addr);

	//socket->enable(ACE_NONBLOCK);
}

bool TheEthManager::createSocket(ACE_INET_Addr local_addr)
{
	char tmp[SIZE_INFO];
	_socket = new ACE_SOCK_Dgram();
	if (-1 == _socket->open(local_addr) )
	{
		sprintf(tmp, "\n/---------------------------------------------------\\"
				"\n|eStiketzi pensa che qualcosa non abbia funzionato!!|"
				"\n\\---------------------------------------------------/");
		YARP_ERROR(Logger::get(),tmp, Logger::get().log_files.f3);
		YARP_FAIL(Logger::get(),tmp);
		return false;
	}
	ACE_thread_t id_recvThread;
	if(ACE_Thread::spawn((ACE_THR_FUNC)recvThread, (void*) _socket, THR_CANCEL_ENABLE, &id_recvThread )==-1)
		printf((LM_DEBUG,"Error in spawning recvThread\n"));


	return true;
}

TheEthManager *TheEthManager::instance(ACE_INET_Addr local_addr)
{
	if (i == 0)
	{
		handle = new TheEthManager(local_addr);
		// move the start during / right before the calibration command??

#ifdef _SEPARETED_THREADS_

		handle->sendThread.start();
#else
		handle->start();
#endif
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
		_socket->close();
		delete handle;
	}
}

bool TheEthManager::register_device(ACE_TCHAR *new_dev_id, ethResources *new_dev_handler)
{
	//	// put this device handler in a list somehow
	//		deviceList.resize(deviceNum +1);
	//
	//#ifdef _DEBUG_
	//	DeviceEntry	dev;
	//	strcpy(dev.id, "0");
	//	dev.resource = 0x00;
	//	int t=deviceList.size();
	//	void *p=&deviceList;
	//#endif
	//	strcpy(deviceList[deviceNum].id, new_dev_id);
	//	deviceList[deviceNum].resource = new_dev_handler;
	//	deviceNum++;
	//
	//#ifdef _DEBUG_
	//	for (int i=0; i<t; i++)
	//		dev=deviceList[i];
	//#endif
	return true;
}

int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
	return _socket->send(data,len,remote_addr);
}

#ifdef _SEPARETED_THREADS_
void *recvThread(void * arg)
{
	size_t 						n = MAX_RECV_SIZE;
	ACE_TCHAR 					address[64];
	ethResources				*ethRes;
	ACE_SOCK_Dgram				*_socket = (ACE_SOCK_Dgram*)arg;
	ACE_UINT16 					recv_size;
	ACE_INET_Addr				sender_addr;
	char 						incoming_msg[MAX_RECV_SIZE];


	while (keepGoingOn2)
	{
		// per ogni msg ricevuto
		recv_size = _socket->recv((void *) incoming_msg, n, sender_addr, 0);

		sender_addr.addr_to_string(address, 64);
		//	printf("Received new packet from address %s, size = %d\n", address, recv_size);

		if( recv_size > 0)
		{
//			check_received_pkt(&sender_addr, (void *) incoming_msg, recv_size);

			ethResIt iterator = ethResCreator::instance()->begin();
			while(iterator!=ethResCreator::instance()->end())
			{
				if(strcmp((*iterator)->address, address) == 0)
				{
					ethRes = (*iterator);
					memcpy(ethRes->recv_msg, incoming_msg, recv_size);
					ethRes->onMsgReception(ethRes->recv_msg, recv_size);
					//continue; // to skip remaining part of the for cycle
				}
				iterator++;
			}
		}
		else
			printf("Received weird msg of size %d\n", recv_size);
	}
}

#else
void TheEthManager::run()
{
	size_t 			n = MAX_RECV_SIZE;
	ACE_TCHAR 		address[64];
	ethResources	*ethRes;

	// per ogni msg ricevuto
	recv_size = _socket->recv((void *) incoming_msg, n, sender_addr, 0);

	sender_addr.addr_to_string(address, 64);
	//	printf("Received new packet from address %s, size = %d\n", address, recv_size);

	if( recv_size > 0)
	{
		ethResIt iterator = ethResCreator::instance()->begin();

		while(iterator!=ethResCreator::instance()->end())
		{
			if(strcmp((*iterator)->address, address) == 0)
			{
				// come fare queste chiamate in parallelo, non "bloccanti" e magari togliere la memcpy?
				ethRes = (*iterator);
				memcpy(ethRes->recv_msg, incoming_msg, recv_size);
				(*iterator)->onMsgReception(ethRes->recv_msg, recv_size);
				//continue; // to skip remaining part of the for cycle
			}
			iterator++;
		}
	}

}

#endif

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
#ifdef _SEPARETED_THREADS_
	keepGoingOn2 = false;
	print_data();
	sleep(1);
	ACE_Thread::cancel(id_recvThread);

#else
	handle->stop();
	handle->sendThread.stop();
#endif
	YARP_INFO(Logger::get(),"TheEthManager::close()", Logger::get().log_files.f3);
	fflush(stdout);
	return true;
}
