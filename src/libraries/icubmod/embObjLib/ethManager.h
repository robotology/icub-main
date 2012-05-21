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
#include <vector>
#include <list>
#include <string>

#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>

// _AC_
// to clean creating an Interface with some common method for ethResources and something like that for the canBus
//#include "../ethManager/iCubDeviceInterface.h"
//#include "../embObjMotionControl/embObjMotionControl.h"
#include "hostTransceiver.hpp"
#include "transceiverInterface.h"



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
#define DEFAULT_PORT			3333

#define	MAX_RECV_SIZE			512
#define	SIZE_INFO				126

#define _DEBUG_


namespace yarp{
    namespace dev{
        class TheEthManager;
        class ethResources;
        class ethResCreator;
    }
}

using namespace yarp::os;
using namespace yarp::dev;


class yarp::dev::ethResources:  //public DeviceDriver,	// needed if I want to create this interface through the polydriver open (I guess)
								public PolyDriver
								//public iCubDeviceInterface
{
private:
	char 						info[SIZE_INFO];
	static int					how_many_boards;


	TheEthManager  				*theEthManager_h;
//	embObjMotionControl			*controller;

    ACE_INET_Addr				remote_dev;
    ACE_INET_Addr				remote_broadcast;

    uint8_t 					*udppkt_data;
   	uint16_t 					udppkt_size;

   	// Protocol handlers
   	PolyDriver					createProtocolHandler;


   	// Motion control handlers
   	PolyDriver					createMotionControlHandler;

   	PolyDriver					createSkinHandler;
   	PolyDriver					createAnalogHandler;

public:

   	hostTransceiver				*transceiver;
	ACE_TCHAR					address[64];
   	/*embObjMotionControl*/ DeviceDriver			*motionCtrl;
	ethResources();

    ~ethResources();

    uint8_t 					recv_msg[512];
	ACE_UINT16 					recv_size;

	ethResources* already_exists(yarp::os::Searchable &config);
    virtual bool open(yarp::os::Searchable &par);

    int send(void *data, size_t len);
    ACE_INET_Addr	getRemoteAddress();
    void getPack(uint8_t **pack, uint16_t *size);
//    void setCalibrator(ICalibrator *icalibrator);
//    void getControlCalibration(IControlCalibration2 **icalib);
//    void getMotorController(DeviceDriver **iMC);
    void onMsgReception(uint8_t *data, uint16_t size);
};


class yarp::dev::ethResCreator:public std::list<ethResources *>
{
	private:
	static ethResCreator 		*handle;
	static int					how_many;

	public:
	static ethResources* getResource(yarp::os::Searchable &config);
	ethResCreator *find(const std::string &id);
};

typedef std::list<ethResources *>::iterator ethResIt;

static ethResCreator ethResList;





// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

class SendThread:	public RateThread
{
	public:
	SendThread();
	~SendThread();

	private:
    uint8_t						*data;
    uint16_t					size;

	virtual void run(void);
};

class yarp::dev::TheEthManager: public DeviceDriver,
								public RateThread
{
private:
    TheEthManager();
    TheEthManager(ACE_INET_Addr local_addr);

    static int					i;
	static TheEthManager 		*handle;
	char 						info[SIZE_INFO];

	ethResources				*res;
    ACE_SOCK_Dgram				*_socket;
    int							deviceNum;

protected:
    ACE_INET_Addr				local_addr;
	ACE_UINT16 					recv_size;
    // Thread
    virtual void run(void);
    SendThread					sendThread;

// recv phase
    ACE_INET_Addr				sender_addr;
	char 						incoming_msg[MAX_RECV_SIZE];


public:
    ~TheEthManager();
    PolyDriver 					polyDriver;
	int							test;

    // Singleton access
    static TheEthManager* instance();
    static TheEthManager* instance(ACE_INET_Addr local_addr);
    bool   register_device(ACE_TCHAR *new_dev_id, ethResources *new_dev_handler);

    //Device Driver
    virtual bool open();
    virtual bool initialize(yarp::os::Searchable &par);
    virtual bool close();

    //	Ethernet business
    int send(void *data, size_t len, ACE_INET_Addr remote_addr);
};

#endif
