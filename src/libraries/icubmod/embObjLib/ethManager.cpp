// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

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

static bool keepGoingOn2 = true;


ethResCreator* ethResCreator::handle = NULL;
yarp::os::Semaphore ethResCreator::_creatorMutex = 1;
bool ethResCreator::initted = false;

TheEthManager* TheEthManager::handle = NULL;
yarp::os::Semaphore TheEthManager::_mutex = 1;
int TheEthManager::_deviceNum = 0;

// ethResources stuff
yarp::os::Semaphore ethResources::_mutex = 1;

ethResources::ethResources()
{
	how_many_features 	= 0;
	transceiver 		= NULL;
	theEthManager_h 	= NULL;
}

ethResources::~ethResources()
{
	// What to do here??
	if(NULL != transceiver)
		delete transceiver;
}


bool ethResources::open(yarp::os::Searchable &config)
{
	yTrace();
	ACE_TCHAR tmp[126], address_tmp_string[64];
	Bottle xtmp2;
	Value val;

	ACE_UINT16 loc_port, rem_port;
	ACE_UINT32 loc_ip1,loc_ip2,loc_ip3,loc_ip4;
	ACE_UINT32 rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	yTrace() << "\n\nEthResources open parameters: " << config.toString().c_str() << "\n";

	//
	// Get EMS ip addresses and port from config file, in order to correctly configure the transceiver.
	//

	// extract eth group info
	Bottle &xtmp = config.findGroup("ETH");

	xtmp2 = xtmp.findGroup("IpAddress");
    if (xtmp2.isNull())
    {
        yError() << "EMS Ip Address not found\n";
        return false;
    }
    // identify ip address
	strcpy(address_tmp_string, xtmp2.get(1).asString().c_str());
	yDebug() << "Ems ip address " << address_tmp_string;
	sscanf(address_tmp_string,"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);

	// identify port
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();

	// Build ip addess/port in ACE format
	remote_dev.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

	// identify EMS board number
	int boardNum=255;
	xtmp2 = xtmp.findGroup("Ems");
    if (xtmp2.isNull())
    {
        yError() << "[ethResources] EMS Board number identifier not found\n";
        return false;
        boardNum = 2;
    }
    else
    	boardNum = xtmp2.get(1).asInt();

	// Fill 'info' field with human friendly string
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "ethResources - referred to EMS: %d.%d.%d.%d", rem_ip1,rem_ip2,rem_ip3,rem_ip4);

	//
	// Get PC104 ip address from config file
	//
	xtmp2 = config.findGroup("PC104IpAddress");
	strcpy(address_tmp_string, xtmp2.get(1).asString().c_str());

	// ACE format
	sscanf(address_tmp_string,"%d.%d.%d.%d",&loc_ip1, &loc_ip2, &loc_ip3, &loc_ip4);
	ACE_INET_Addr loc_dev(rem_port, (loc_ip1<<24)|(loc_ip2<<16)|(loc_ip3<<8)|loc_ip4);

	// Get the pointer to the actual Singleton ethManager, or create it if it's the first time.
	theEthManager_h = TheEthManager::instance(loc_dev);
	if(theEthManager_h == NULL)
	{
		yError() 	<< "\n----------------------------"  \
							<< "\n while creating ETH MANAGER "  \
							<< "\n----------------------------";
		return false;
	}

	//
	//	EMBOBJ INIT
	//
	if ( ( boardNum >= FIRST_BOARD) && ( boardNum <= LAST_BOARD) )
	{
		transceiver= new hostTransceiver;
		transceiver->init(eo_common_ipv4addr(loc_ip1,loc_ip2,loc_ip3,loc_ip4), eo_common_ipv4addr(rem_ip1,rem_ip2,rem_ip3,rem_ip4), rem_port, EOK_HOSTTRANSCEIVER_capacityofpacket, boardNum);
	}

	how_many_features = 1;
	yDebug() << "Transceiver succesfully initted.";

	return true;
}

bool ethResources::registerFeature(yarp::os::Searchable &config)
{
	how_many_features++;
	return true;
}

//bool ethResources::mayclose()
//{
//	how_many_features--;
//	if(how_many_features <= 0)
//	return true;
//}

// Per inviare pacchetti in maniera asincrona rispetto al ciclo da 1 ms... anche roba che non sia eoStuff
// Eliminare ???
int ethResources::send(void *data, size_t len)
{
	return TheEthManager::instance()->send(data, len, remote_dev);
}

void ethResources::getPack(uint8_t **pack, uint16_t *size)
{
	if (0 != transceiver)
	{
		_mutex.wait();
		transceiver->getTransmit(pack, size);
		_mutex.post();
	}
}

ACE_UINT16	ethResources::getBufferSize()
{
	return (ACE_UINT16) RECV_BUFFER_SIZE;
}

void ethResources::onMsgReception(uint8_t *data, uint16_t size)
{
	if (0 != transceiver)
	{
		_mutex.wait();
		transceiver->onMsgReception(data, size);
		_mutex.post();
	}
}

ACE_INET_Addr	ethResources::getRemoteAddress()
{
	return	remote_dev;
}


bool ethResources::goToConfig(void)
{
    yTrace();
    eOnvID_t nvid;
    EOnv tmp;

    // attiva il loop di controllo
    eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used but there for API compatibility
    nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);

    EOnv  *nvRoot   = transceiver->getNVhandler(endpoint_mn_appl, nvid, &tmp);
    if(NULL == nvRoot)
    {
        yError () << "NV pointer not found at line" << __LINE__;
        return false;
    }

    eOmn_appl_state_t  desired  = applstate_config;

    if( !transceiver->nvSetData(nvRoot, &desired, eobool_true, eo_nv_upd_dontdo))
        return false;

    // tell agent to prepare a rop to send
    if( !transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid) )
        return false;
}

bool ethResources::goToRun(void)
{
    yTrace();
    eOnvID_t nvid;
    EOnv tmp;

    // attiva il loop di controllo
    eOcfg_nvsEP_mn_applNumber_t dummy = 0;  // not used but there for API compatibility
    nvid = eo_cfg_nvsEP_mn_appl_NVID_Get(endpoint_mn_appl, dummy, applNVindex_cmmnds__go2state);

    EOnv  *nvRoot   = transceiver->getNVhandler(endpoint_mn_appl, nvid, &tmp);
    if(NULL == nvRoot)
    {
        yError () << "NV pointer not found at line" << __LINE__;
        return false;
    }

    eOmn_appl_state_t  desired  = applstate_running;

    if( !transceiver->nvSetData(nvRoot, &desired, eobool_true, eo_nv_upd_dontdo))
        return false;

    // tell agent to prepare a rop to send
    if( !transceiver->load_occasional_rop(eo_ropcode_set, endpoint_mn_appl, nvid) )
        return false;
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
	if(this->size() != 0)
	{
		ethResIt iterator = this->begin();
		while(iterator != this->end())
		{
			delete (*iterator);
			iterator++;
		}
	}
}

ethResCreator *ethResCreator::instance()
{
	if( !initted && keepGoingOn2)
	{
		_creatorMutex.wait();
		handle = new ethResCreator();
		initted = true;
		_creatorMutex.post();
	}
	return handle;
}

void ethResCreator::close()
{
//	ethResCreator::~ethResCreator();
}

ethResources* ethResCreator::getResource(yarp::os::Searchable &config)
{
	ACE_TCHAR 	remote_address[64];
	Bottle 		xtmp2;

	ACE_UINT16 	rem_port;
	ACE_UINT32 	rem_ip1,rem_ip2,rem_ip3,rem_ip4;

	std::string str=config.toString().c_str();
	yTrace() << "\n\n EthResCreator parameters\n" << str;
	
	//
	// Get EMS ip addresses from config file, to see if we need to instantiate a new Resources or simply return
	//	a pointer to an already existing object
	//

	_creatorMutex.wait();
	Bottle xtmp = Bottle(config.findGroup("ETH"));
	xtmp2 = xtmp.findGroup("IpAddress");
	strcpy(remote_address, xtmp2.get(1).asString().c_str());


	sscanf(remote_address,"%d.%d.%d.%d",&rem_ip1, &rem_ip2, &rem_ip3, &rem_ip4);

	// Get EMS CmdPort from config file
	xtmp2 = xtmp.findGroup("CmdPort");
	rem_port = xtmp2.get(1).asInt();

	// ACE format
	ACE_INET_Addr remote_addr_tmp;
	remote_addr_tmp.set(rem_port, (rem_ip1<<24)|(rem_ip2<<16)|(rem_ip3<<8)|rem_ip4);

	ethResources *newRes = NULL;
	ethResIt iterator = this->begin();

	while(iterator!=this->end())
	{
		if((*iterator)->getRemoteAddress() == remote_addr_tmp)
		{
			// device already exist.
			yDebug() << "Returning a pointer to an alreasy existing device.";
			newRes = (*iterator);
			break;
		}
		iterator++;
	}

	if ( NULL == newRes)
	{
		// device doesn't exist yet, create it
		yDebug() << "device doesn't exist yet, creating it.";
		newRes = new ethResources;
		if(!newRes->open(config))
		{
			printf("Error opening new EMS!!");
			if(NULL != newRes)
				delete newRes;

			newRes = NULL;
		}
		else
		{
			how_many_boards++;
			this->push_back(newRes);
		}
	}
	_creatorMutex.post();
	return newRes;
}

bool ethResCreator::removeResource(ethResources* to_be_removed)
{
	_creatorMutex.wait();
	to_be_removed->close();
	if(this->size() != 0)
	{
		ethResIt iterator = this->begin();
		while(iterator != this->end())
		{
			if((*iterator)->getRemoteAddress() == to_be_removed->getRemoteAddress())
			{
				delete (*iterator);
				this->remove(*iterator);
				_creatorMutex.post();
				return true;
			}
			iterator++;
		}
		yError() << "[ethResCreator] Asked to remove an entry but it wan not found!\n";
	}
	_creatorMutex.post();
	yError() << "[ethResCreator] Asked to remove an entry in an empty list!\n";
	return false;
}

void ethResCreator::addLUTelement(FEAT_ID id)
{
    _creatorMutex.wait();
    bool addLUT_res;
    addLUT_res = ethResCreator::class_lut.insert(std::pair<uint8_t, FEAT_ID>(id.ep, id)).second;
//     addLUT_res ? printf("ok add lut element") : printf("NON ok add lut element");
    _creatorMutex.post();
}

void *ethResCreator::getHandleFromEP(eOnvEP_t ep)
{
	_creatorMutex.wait();
	void * ret =ethResCreator::class_lut[ep].handle;
	_creatorMutex.post();
	return ret;
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
	ACE_TCHAR 		address[64];
	ethResources	*ethRes;
	ACE_UINT16 		recv_size;
	ACE_INET_Addr	sender_addr;
	char 			incoming_msg[RECV_BUFFER_SIZE];

    ACE_SOCK_Dgram *pSocket = (ACE_SOCK_Dgram*) arg;

    Time::delay(5);

    printf("Starting udp RECV thread\n");
	while(keepGoingOn2)
	{
		// per ogni msg ricevuto
		recv_size = pSocket->recv((void *) incoming_msg, RECV_BUFFER_SIZE, sender_addr, 0);

		sender_addr.addr_to_string(address, 64);
//     printf("Received new packet from address %s, size = %d\n", address, recv_size);

		if( (recv_size > 0) && (keepGoingOn2) )
		{
			ethResIt iterator = ethResCreator::instance()->begin();
			//check_received_pkt(&sender_addr, (void *) incoming_msg, recv_size);

			while(iterator!=ethResCreator::instance()->end())
			{
				if((*iterator)->getRemoteAddress() == sender_addr)
				{
					ethRes = (*iterator);
					if(recv_size > ethRes->getBufferSize() || (recv_size <=0) )
					{
						printf("Error, received a message of wrong size ( received %d bytes while buffer is %d bytes long)\n", recv_size, ethRes->getBufferSize());
					}
					else
					{
						memcpy(ethRes->recv_msg, incoming_msg, recv_size);
						ethRes->onMsgReception(ethRes->recv_msg, recv_size);
					}
					//continue; 	// to skip remaining part of the for cycle // crea loop infinito perch`e prima del iterator++
				}
				iterator++;
			}
		}
		else
		{
			printf("Received weird msg of size %d\n", recv_size);
		}
	}
	return NULL;
}

// EthManager uses 2 threads, one for sending and one for receiving.
// The one embedded here is the sending thread because it will use the ACE timing feature.
TheEthManager::TheEthManager() : RateThread(1)
{
	memset(info, 0x00, SIZE_INFO);
	sprintf(info, "TheEthManager");
	ethResList = ethResCreator::instance();

	p_to_data = NULL;
	_socket_initted = false;
	_socket = NULL;
	id_recvThread = -1;
}

bool TheEthManager::createSocket(ACE_INET_Addr local_addr)
{
	_socket = new ACE_SOCK_Dgram();
	yDebug() << "PC104 Address " << local_addr.get_host_addr();
	if (-1 == _socket->open(local_addr) )
	{
		yError() <<  "\n/---------------------------------------------------\\" \
				 <<		"\n|eStiketzi pensa che qualcosa non abbia funzionato!!|" \
				 <<		"\n\\---------------------------------------------------/";
		delete _socket;
		_socket = NULL;
		_socket_initted = false;
	}
	else
	{
		_socket_initted = true;

		ACE_thread_t id_recvThread;
		if(ACE_Thread::spawn((ACE_THR_FUNC)recvThread, (void*) _socket, THR_CANCEL_ENABLE, &id_recvThread )==-1)
			printf(("Error in spawning recvThread\n"));

		// Start sending thread
		handle->start();
	}
	return _socket_initted;
}

bool TheEthManager::isInitted(void)
{
	return _socket_initted;
}

TheEthManager *TheEthManager::instance(ACE_INET_Addr local_addr)
{
	TheEthManager::_mutex.wait();
	bool ret = false;
	if (_deviceNum == 0)
	{
		yTrace() << "Calling EthManager Constructor";

		handle = new TheEthManager();
		ret = handle->createSocket(local_addr);
		if(!ret )
		{
			delete handle;
			handle = NULL;
		}
	}

	if(handle)
		_deviceNum++;

	TheEthManager::_mutex.post();
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
	if(id_recvThread != -1)
		ACE_Thread::cancel(id_recvThread);

	if(isInitted())
	{
		_socket->close();
		delete _socket;
	}
}


int TheEthManager::send(void *data, size_t len, ACE_INET_Addr remote_addr)
{
	if(_socket_initted)
		return _socket->send(data,len,remote_addr);
	else
		yError() << "Local ETH Socket NOT correctly initted!";

	return -1;
}

bool TheEthManager::threadInit()
{
	ethResList = ethResCreator::instance();
	return true;
}


void TheEthManager::run()
{
	// send
	ACE_TCHAR		address_tmp[64];
	ethResIt 		iterator = ethResList->begin();
	ethResources 	*ethRes;
	uint16_t 		bytes_to_send = 0;

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
//			(*iterator)->getRemoteAddress().addr_to_string(address_tmp, 64);
//			printf("sent package of byte %d to %s\n", ret, address_tmp);
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
