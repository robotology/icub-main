// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup TheEthManager TheEthManager
 *
*/
/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
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
#include "embObjLibInterface.h"
#include "debugFunctions.h"


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

//#define	MAX_RECV_SIZE			512
#define	MAX_RECV_SIZE			1500
#define	SIZE_INFO				126

#define _DEBUG_
#define _SEPARETED_THREADS_

namespace yarp{
    namespace dev{
        class TheEthManager;
        class ethResources;
        class ethResCreator;
    }
}

using namespace yarp::os;
using namespace yarp::dev;

#ifdef _SEPARETED_THREADS_
	void *recvThread(void * arg);
	#include "ace/Thread.h"
#endif

class yarp::dev::ethResources:  //public DeviceDriver,	// needed if I want to create this interface through the polydriver open (I guess)
								public PolyDriver
								//public iCubDeviceInterface
{
private:
	char 						info[SIZE_INFO];

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

	EMS_ID						id;
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


// -------------------------------------------------------------------\\
//            ethResCreator   Singleton
// -------------------------------------------------------------------\\

class yarp::dev::ethResCreator: public std::list<ethResources *>,
								public IEmbObjResList
{
	private:
		ethResCreator();
		~ethResCreator();
		static ethResCreator 		*handle;
		static bool					initted;
		int							how_many_boards;
		bool	compareIds(EMS_ID id2beFound, EMS_ID comparingId);

	public:
		static ethResCreator* 		instance();
		ethResources* 				getResource(yarp::os::Searchable &config);
		virtual uint8_t*			find(EMS_ID &id);
};

typedef std::list<ethResources *>::iterator ethResIt;






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
    ethResCreator 				*ethResList;
    ethResIt 					iterator;

    virtual bool threadInit();
	virtual void run(void);
};

class yarp::dev::TheEthManager: public DeviceDriver
#ifndef _SEPARETED_THREADS_
,public RateThread
#endif
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
    SendThread					sendThread;


#ifdef _SEPARETED_THREADS_

	ACE_thread_t 				id_recvThread;
#else
    // Thread
    virtual void run(void);
	ACE_UINT16 					recv_size;
	// recv phase
	ACE_INET_Addr				sender_addr;
	char 						incoming_msg[MAX_RECV_SIZE];
#endif



public:
    ~TheEthManager();
    PolyDriver 					polyDriver;
	int							test;

    // Singleton access
    static TheEthManager* instance();
    static TheEthManager* instance(ACE_INET_Addr local_addr);
    bool   register_device(ACE_TCHAR *new_dev_id, ethResources *new_dev_handler);
    bool  createSocket(ACE_INET_Addr local_addr);

    //Device Driver
    virtual bool open();
    virtual bool initialize(yarp::os::Searchable &par);
    virtual bool close();

    //	Ethernet business
    int send(void *data, size_t len, ACE_INET_Addr remote_addr);
};

#endif
