// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup iCubDeviceInterface iCubDeviceInterface
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

#ifndef __iCubDeviceInterface__
#define __iCubDeviceInterface__

using namespace std;

#include <iostream>				// string type
#include <vector>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>

//#include "../embObjMotionControl/embObjMotionControl.h"
//#include "../ethManager/ethManager.h"
//#include "../embObjLib/hostTransceiver.hpp"
//#include "../embObjLib/transceiverInterface.h"

namespace yarp{
    namespace dev{
			class iCubDeviceInterface;
    }
}

class yarp::dev::iCubDeviceInterface
{
private:
    static int					i;
	char 						info[126];
//	TheEthManager  				*theEthManager_h;
//	embObjMotionControl			*controller;

//    uint8_t 					*udppkt_data;
//   	uint16_t 					udppkt_size;

//   	PolyDriver					createProtocolHandler;
//   	ITransceiver 				*transceiver;

//   	PolyDriver					createMotionControlHandler;
//   	embObjMotionControl			*motionCtrl;

//   	PolyDriver					createSkinHandler;
//   	PolyDriver					createAnalogHandler;


public:

    virtual bool open(yarp::os::Searchable &par) =0;
    virtual void setCalibrator(ICalibrator *icalibrator) =0;
    virtual void getControlCalibration(IControlCalibration2 **icalib) =0;
};


#endif
