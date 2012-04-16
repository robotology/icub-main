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

#include <string>

#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>

// _AC_
// to clean creating an Interface with some common method for ethResources and something like that for the canBus
#include "../ethManager/iCubDeviceInterface.h"
#include "../embObjMotionControl/embObjMotionControl.h"
//#include "../embObjLib/hostTransceiver.hpp"
#include "../embObjLib/transceiverInterface.h"

// Boards configurations
#include "eOcfg_EPs_rem_board.h"




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

namespace yarp{
    namespace dev{
        class TheEthManager;
        class ethResources;
    }
}

typedef struct
{
	ACE_TCHAR					id[64];
	yarp::dev::ethResources 	*resource;
}DeviceEntry;

class yarp::dev::ethResources:  public DeviceDriver,	// needed if I want to create this interface through the polydriver open (I guess)
								public iCubDeviceInterface
{
private:
    static int					how_many;
	char 						info[126];
	TheEthManager  				*theEthManager_h;
//	embObjMotionControl			*controller;

    ACE_INET_Addr				remote_dev;
    ACE_INET_Addr				remote_broadcast;

    uint8_t 					*udppkt_data;
   	uint16_t 					udppkt_size;

   	// uint8_t 					*recv_data;

   	// Protocol handlers
   	PolyDriver					createProtocolHandler;
   	ITransceiver 				*transceiver;

   	// Motion control handlers
   	PolyDriver					createMotionControlHandler;
   	embObjMotionControl			*motionCtrl;

   	PolyDriver					createSkinHandler;
   	PolyDriver					createAnalogHandler;

public:
	ethResources();
	//ethResources(ACE_UINT16 loc_port, ACE_UINT32 loc_ip, ACE_UINT16 rem_port, ACE_UINT32 rem_ip);
    ~ethResources();

    uint8_t 					recv_msg[512];
	ACE_UINT16 					recv_size;

    virtual bool open(yarp::os::Searchable &par);

    int send(void *data, size_t len);
    ACE_INET_Addr	getRemoteAddress();
    void getPack(uint8_t **pack, uint16_t *size);
    void setCalibrator(ICalibrator *icalibrator);
    void getControlCalibration(IControlCalibration2 **icalib);
    void onMsgReception(uint8_t *data, uint16_t size);
};


// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\

class yarp::dev::TheEthManager: public DeviceDriver,
								public os::RateThread
{
private:
    TheEthManager();
    TheEthManager(ACE_INET_Addr local_addr);

    static int					i;
	static TheEthManager 		*handle;
	char 						info[126];

	ethResources				*res;
    ACE_SOCK_Dgram				*socket;
    std::vector<DeviceEntry> 	deviceList;
    int							deviceNum;

    uint8_t						*data;
    uint16_t					size;
protected:
    ACE_INET_Addr				local_addr;
    // Tread
    virtual void run(void);

// recv phase
    ACE_INET_Addr				sender_addr;
	char 						incoming_msg[512];
	ACE_UINT16 					recv_size;

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
