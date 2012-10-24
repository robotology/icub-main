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
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>

// _AC_
// to clean creating an Interface with some common method for ethResources and something like that for the canBus
//#include "../ethManager/iCubDeviceInterface.h"
//#include "../embObjMotionControl/embObjMotionControl.h"
#include "hostTransceiver.hpp"
//#include "embObjLibInterface.h"
#include "debugFunctions.h"
#include "FeatureInterface.h"
#include <map>

// ACE stuff
#include <ace/ACE.h>
#include "ace/Thread.h"
#include <ace/SOCK_Dgram_Bcast.h>


//#define _ICUB_CALLBACK_

#include "Debug.h"



//// debug with workstation
//#define DEBUG_LAPTOP_IP			"10.255.37.155" 	// <- dhcp;
//#define DEBUG_WORKSTATION_IP 	"10.255.37.24" // ip della workstation qui dietro.
//
//// to use inside the robot
//#define DEFAULT_PC104_IP		"10.0.0.1" 			// <- fix;
//#define DEBUG_PC104_IP			"1.1.1.1" 			// <- fix;
//#define DEBUG_EMS_IP			"10.0.0.2" 			// <- fix;
//
//#define PC104_IP				DEBUG_LAPTOP_IP
//#define	EMS_IP					DEBUG_EMS_IP
//#define DEFAULT_PORT			3333

//#define	MAX_RECV_SIZE			512
#define	MAX_RECV_SIZE			1500
#define	SIZE_INFO				126
#define MAX_ICUB_EP				32

namespace yarp{
    namespace dev{
        class TheEthManager;
        class ethResources;
        class ethResCreator;
    }
}

typedef struct
{
		char 		name[64];
		uint8_t		ip1;
		uint8_t		ip2;
		uint8_t		ip3;
		uint8_t		ip4;
}EMS_ID;

using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

// Thread function used to receive messages from EMSs
void *recvThread(void * arg);


class yarp::dev::ethResources:  public PolyDriver
{
private:
	char 						info[SIZE_INFO];
	int							how_many_features;

	TheEthManager  				*theEthManager_h;

    ACE_INET_Addr				remote_dev;
    ACE_INET_Addr				remote_broadcast;


public:
	EMS_ID						id;				// to be removed
   	hostTransceiver				*transceiver;

	ethResources();
    ~ethResources();

    uint8_t 					recv_msg[512];
	ACE_UINT16 					recv_size;

	ethResources* already_exists(yarp::os::Searchable &config);
    bool open(yarp::os::Searchable &par);
    bool ethResources::registerFeature(yarp::os::Searchable &config);

    int send(void *data, size_t len);
    ACE_INET_Addr	getRemoteAddress();
    void getPack(uint8_t **pack, uint16_t *size);

    void onMsgReception(uint8_t *data, uint16_t size);
};


// -------------------------------------------------------------------\\
//            ethResCreator   Singleton
// -------------------------------------------------------------------\\


class yarp::dev::ethResCreator: public std::list<ethResources *>
{
	private:
		static yarp::os::Semaphore 	_mutex;
		static ethResCreator 		*handle;
		static bool					initted;
		int							how_many_boards;
		map 						<eOnvEP_t, FEAT_ID> class_lut;

		ethResCreator();
		~ethResCreator();
		bool						compareIds(EMS_ID id2beFound, EMS_ID comparingId);

	public:
		void						close(void);
		static ethResCreator* 		instance();
		ethResources* 				getResource(yarp::os::Searchable &config);
		bool 						removeResource(ethResources* to_be_removed);
		void 						addLUTelement(FEAT_ID id);
		void *						getHandleFromEP(eOnvEP_t ep);
		FEAT_ID 					getFeatInfoFromEP(uint8_t ep);
};

typedef std::list<ethResources *>::iterator ethResIt;




// -------------------------------------------------------------------\\
//            TheEthManager   Singleton
// -------------------------------------------------------------------\\


class RecvThread:	public RateThread
{
	public:
	RecvThread();
	~RecvThread();

	private:


    ethResIt 					iterator;

    virtual bool threadInit();
	virtual void run(void);
};

class yarp::dev::TheEthManager: public DeviceDriver,
								public RateThread
{
private:
    TheEthManager();

    static int					_deviceNum;
	static TheEthManager 		*handle;
	bool 						_socket_initted;
	static yarp::os::Semaphore 	_mutex;
	char 						info[SIZE_INFO];

//	ethResources				*res;
    ACE_SOCK_Dgram				*_socket;

	ACE_thread_t 				id_recvThread;
    ethResCreator 				*ethResList;
    uint8_t						*p_to_data;

protected:
    ACE_INET_Addr				local_addr;

    // Thread
    virtual void run(void);


public:
    bool threadInit();
    ~TheEthManager();
    PolyDriver 					polyDriver;

    // Singleton access
    static 	TheEthManager* instance();
    static 	TheEthManager* instance(ACE_INET_Addr local_addr);

    bool   	register_device(ACE_TCHAR *new_dev_id, ethResources *new_dev_handler);
    bool  	createSocket(ACE_INET_Addr local_addr);
    bool	isInitted(void);

    //Device Driver
    virtual bool open();
    virtual bool close();

    //	Ethernet business
    int 	send(void *data, size_t len, ACE_INET_Addr remote_addr);
};

#endif
