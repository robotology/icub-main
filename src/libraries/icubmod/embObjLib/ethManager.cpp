// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
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

#include "Debug.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;

bool keepGoingOn2 = true;


ethResCreator* ethResCreator::handle = NULL;
bool ethResCreator::initted = false;

TheEthManager* TheEthManager::handle = NULL;
int TheEthManager::_deviceNum = false;
yarp::os::Semaphore TheEthManager::_mutex = 1;


ethResources::ethResources()
{
	transceiver = NULL;
	theEthManager_h = NULL;
}

ethResources::~ethResources()
{
	// Really nothing to do here??
	delete transceiver;
}


bool ethResources::open(yarp::os::Searchable &config)
{
	yTrace();
	ACE_TCHAR tmp[126]; //, address[64];
	Bottle xtmp, xtmp2;
	Value val;
	//	string str=config.toString().c_str();
	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	yDebug() << "\nEthResources open parameters: " << config.toString().c_str() << "\n";

	//
	// Get EMS ip addresses and port from config file, in order to correctly configure the transceiver.
	//
	// extract eth group info
	xtmp = Bottle(config.findGroup("ETH"));

	xtmp2 = xtmp.findGroup("IpAddress");
    if (xtmp2.isNull())
    {
        yError() << "EMS Ip Address not found\n";
        return false;
    }
	strcpy(address, xtmp2.get(1).asString().c_str());
	yDebug() << "Ems ip address " << address;

	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);
	sprintf(tmp,"remote01.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), rem_ip1,rem_ip2,rem_ip3,rem_ip4);

	// Get EMS CmdPort from config file
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();
	// ACE format
	remote_dev.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

	int boardNum=255;
	xtmp2 = xtmp.findGroup("Ems");
    if (xtmp2.isNull())
    {
        yError() << "EMS Board number identifier not found\n";
        return false;
    }

	boardNum = xtmp2.get(1).asInt();
	printf("Ems %d\n", boardNum);
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
	printf("%s\n", info);

	//
	// Get PC104 ip address from config file
	//
	xtmp2 = config.findGroup("PC104IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());

	// ACE format
	sscanf(xtmp2.get(1).asString().c_str(),"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
	sprintf(tmp,"pc104.address: %s, %d:%d:%d:%d\n", xtmp2.get(1).asString().c_str(), loc_ip1,loc_ip2,loc_ip3,loc_ip4);


	ACE_INET_Addr loc_dev(rem_port, (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4);

	// Get the pointer to the actual Singleton ethManager, or create it if it's the first time.
	theEthManager_h = TheEthManager::instance(loc_dev);
	remote_dev.addr_to_string(address, 64);
//	theEthManager_h->register_device(address, this);


	//
	//	EMBOBJ INIT
	//
	if ( ( boardNum >= FIRST_BOARD) && ( boardNum <= LAST_BOARD) )
	{
		transceiver= new hostTransceiver;
		transceiver->init(eo_common_ipv4addr(loc_ip1,loc_ip2,loc_ip3,loc_ip4), eo_common_ipv4addr(rem_ip1,rem_ip2,rem_ip3,rem_ip4), rem_port, EOK_HOSTTRANSCEIVER_capacityofpacket, boardNum);
	}

	yDebug() << "Transceiver succesfully initted.";

	return true;
}

// Per inviare pacchetti in maniera asincrona rispetto al ciclo da 1 ms... anche robaccia che non sia eoStuff
// Eliminare ???
int ethResources::send(void *data, size_t len)
{
	return TheEthManager::instance()->send(data, len, remote_dev);
}

void ethResources::getPack(uint8_t **pack, uint16_t *size)
{
	if (0 != transceiver)
		transceiver->getTransmit(pack, size);
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

	// AC_YARP_INFO(Logger::get(), " ethResCreator::getResource", Logger::get().log_files.f3);

	//
	// Get EMS ip addresses from config file, to see if we need to instantiate a new Resources or simply return
	//	a pointer to an already existing object
	//

	xtmp = Bottle(config.findGroup("ETH"));
	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(address, xtmp2.get(1).asString().c_str());
	sprintf(tmp,"EMS IpAddress %s", address);
	// AC_YARP_INFO(Logger::get(), tmp, Logger::get().log_files.f3);

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
			// AC_YARP_INFO(Logger::get(), String("device already exist\n") + address, Logger::get().log_files.f3);

			newRes = (*iterator);
		}
		iterator++;
	}

	if ( NULL == newRes)
	{
		// device doesn't exist yet, create it
		// AC_YARP_INFO(Logger::get(), String("device doesn't exist yet, create it\n") + address, Logger::get().log_files.f3);
		newRes = new ethResources;
		newRes->open(config);
		how_many_boards++;
		this->push_back(newRes);
	}
	return newRes;
}

bool ethResCreator::compareIds(EMS_ID id2beFound, EMS_ID nextId)
{
	if( (id2beFound.ip1 == nextId.ip1) && (id2beFound.ip2 == nextId.ip2) && (id2beFound.ip3 == nextId.ip3) && (id2beFound.ip4 == nextId.ip4) )
		return true;
	else
		return false;
}

#warning " vecchia interfaccia, eliminare??"
uint8_t* ethResCreator::find(EMS_ID &id)
{
	// AC_YARP_INFO(Logger::get(), " ethResCreator::find", Logger::get().log_files.f3);

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

void ethResCreator::addLUTelement(FEAT_ID id)
{
	ethResCreator::class_lut.insert(std::pair<uint8_t, FEAT_ID>(id.ep, id));
	//ethResCreator::class_lut[id.ep] = id;		// does the same??
}

void *ethResCreator::getHandleFromEP(eOnvEP_t ep)
{
	return ethResCreator::class_lut[ep].handle;
}

FEAT_ID ethResCreator::getFeatInfoFromEP(uint8_t ep)
{
	return ethResCreator::class_lut[ep];
}


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

void *recvThread(void * arg)
{
	size_t 			n = MAX_RECV_SIZE;
	ACE_TCHAR 		address[64];
	ethResources	*ethRes;
	ACE_UINT16 		recv_size;
	ACE_INET_Addr	sender_addr;
	char 			incoming_msg[MAX_RECV_SIZE];

    ACE_SOCK_Dgram *pSocket = (ACE_SOCK_Dgram*) arg;
	while(keepGoingOn2)
	{
		// per ogni msg ricevuto
		recv_size = pSocket->recv((void *) incoming_msg, n, sender_addr, 0);

		sender_addr.addr_to_string(address, 64);
		//	printf("Received new packet from address %s, size = %d\n", address, recv_size);

		if( recv_size > 0)
		{
			ethResIt iterator = ethResCreator::instance()->begin();
			//check_received_pkt(&sender_addr, (void *) incoming_msg, recv_size);

			while(iterator!=ethResCreator::instance()->end())
			{
				if(strcmp((*iterator)->address, address) == 0)
				{
					// fare queste chiamate in parallelo, non "bloccanti" e magari togliere la memcpy?
					ethRes = (*iterator);
					memcpy(ethRes->recv_msg, incoming_msg, recv_size);
					ethRes->onMsgReception(ethRes->recv_msg, recv_size);
					//continue; // to skip remaining part of the for cycle
				}
				iterator++;
			}
		}
		else
		{
			printf("Received weird msg of size %d\n", recv_size);
		}
		// old debug stuff
		//	if(counter++ >= 10*1000)
		//	{
		//		print_data();
		//		fflush(stdout);
		//		counter = 0;
		//	}
	}
	return NULL;
}

// EthManager uses 2 threads, one for sending and one for receiving.
// The one embedded here is the sending thread because it will use the ACE timing feature.
TheEthManager::TheEthManager() : RateThread(1)
{
	char tmp[126];
	sprintf(tmp, "Error!!! Called constructor of ethResources without parameters!!! BTW, how can this be possible ???");
}

TheEthManager::TheEthManager(ACE_INET_Addr local_addr) : RateThread(1)
{
	char tmp[SIZE_INFO];
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "TheEthManager");

	_socket_initted = false;

	local_addr.addr_to_string(tmp, 64);

	//TheEthManager::createSocket(local_addr);

	_socket = new ACE_SOCK_Dgram();
	if (-1 == _socket->open(local_addr) )
	{
		fprintf(stderr, "\n/---------------------------------------------------\\"
						"\n|eStiketzi pensa che qualcosa non abbia funzionato!!|"
						"\n\\---------------------------------------------------/");
		delete _socket;
		_socket = NULL;
		_socket_initted = false;
	}
	else
		_socket_initted = true;


	// If something went wrong, clean everything
	printf("\ninitted = %s\n", _socket_initted ? "true" : "false");
	if(!_socket_initted)
	{
		if(NULL !=handle)
			delete handle;
		handle = NULL;

		if(NULL != _socket)
			delete _socket;
		_socket = NULL;
	}
	else
	{
		ACE_thread_t id_recvThread;
		if(ACE_Thread::spawn((ACE_THR_FUNC)recvThread, (void*) _socket, THR_CANCEL_ENABLE, &id_recvThread )==-1)
			printf(("Error in spawning recvThread\n"));
	}
}

TheEthManager *TheEthManager::instance(ACE_INET_Addr local_addr)
{
	TheEthManager::_mutex.wait();
	if (_deviceNum == 0)
	{
		handle = new TheEthManager(local_addr);
		printf(" \n\nCalling EthManager Constructor\n\n");
		// Start sending thread
		handle->start();
	}
	_deviceNum++;

	_mutex.post();
	return handle;
}

TheEthManager *TheEthManager::instance()
{
	return handle;
}

TheEthManager::~TheEthManager()
{
	keepGoingOn2 = false;
	Time::delay(1);
	ACE_Thread::cancel(id_recvThread);
	_socket->close();
	delete _socket;
}


int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
	return _socket->send(data,len,remote_addr);
}

bool TheEthManager::threadInit()
{
	ethResList = ethResCreator::instance();
	return true;
}


void TheEthManager::run()
{
	// send
	ethResIt iterator = ethResList->begin();
	ethResources *ethRes;
	while(iterator!=ethResList->end())
	{
		p_to_data = 0;
		bytes_to_send = 0;
		ethRes = (*iterator);
		ethRes->getPack(&p_to_data, &bytes_to_send);
		if((bytes_to_send > 20) && (0 != p_to_data))
		{
			ACE_INET_Addr addr = ethRes->getRemoteAddress();
			int ret = TheEthManager::instance()->send(p_to_data, (size_t)bytes_to_send, addr);
			//printf("sent package of byte %d to %s\n", ret, (*iterator)->address);
		}
		iterator++;
	}
}



// Probably useless
bool TheEthManager::open()
{
	char tmp[126];
	sprintf(tmp, "TheEthManager open - handle= %p - i=%d", handle, _deviceNum);
	return true;
}


bool TheEthManager::close()
{
	_deviceNum--;

	if(0 == _deviceNum)
		delete handle;

	return true;
}
