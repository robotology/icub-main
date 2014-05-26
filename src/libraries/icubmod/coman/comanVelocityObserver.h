// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


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

// update comment hereafter

/**
 * @ingroup coman_hardware_modules
 *
 * Copyright (C) 2012 iCubFacility - Istituto Italiano di Tecnologia
 *
 * Author: Alberto Cardellino
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/modules/libraries/icubmod/coman.h
 *
 */

//
// $Id: comanMotionControl.h,v 1.0 2013/02/5 $
//

#ifndef __comanVelocityObserver_h__
#define __comanVelocityObserver_h__

using namespace std;
using namespace yarp::os;

/////  Yarp stuff
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/dev/DeviceDriver.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>

#include <yarp/dev/IAnalogSensor.h>
///////////////////

#include <iCub/FactoryInterface.h>


// ACE udp socket
#include <ace/ACE.h>
#include <ace/SOCK_Dgram_Bcast.h>

#include <robolli/DSP_board.h>
#include <robolli/Boards_iface.h>

#include <comanDevicesHandler.hpp>

#define SIZE_INFO 128


/*
 * simple helper template to alloc memory.
 */
#define _YARP_ASSERT(x) { if (!(x)) { printf("memory allocation failure\n"); exit(1); } }

template <class T>
inline T* allocAndCheck(int size)
{
    T* t = new T[size];
    _YARP_ASSERT (t != 0);
    memset(t, 0, sizeof(T) * size);
    return t;
}

/*
 *
 */
template <class T>
inline void checkAndDestroy(T* &p) {
    if (p!=0) {
        delete [] p;
        p = 0;
    }
}

namespace yarp {
    namespace dev {
                    class comanVelocityObserver;
    }
}

class yarp::dev::comanVelocityObserver:    public DeviceDriver,
                                            public IAnalogSensor
{
private:

    ///////////// coman specific  ///////////////
    int32_t                 *trq_array;
    int32_t                 *off_array;           // pid offset, used as a feedforward term in control
    // handling classes
    comanDevicesHandler     *_comanHandler;
    Boards_ctrl             *_boards_ctrl;
    Boards_ctrl::mcs_map_t  _mcs;

    ////////  canonical
    yarp::os::Semaphore     _mutex;

    int                     *_axisMap;                          /** axis remapping lookup-table */
    int                     *_bIdMap;                           /* conversion from joint number to bId */
    int                     *_inv_bIdMap;                       /* conversion back from bId to joint number */
    double                  *_angleToEncoder;                   /** mRads to degrees conversion factors */

    // basic knowledge of my joints
    int                     _nChannels;                         // Number of channels handled by this class; this values will be extracted by the config file

private:
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    McBoard *getMCpointer(int j);
    int bId2Joint(int j);
    uint8_t jointTobId(int j);

    uint16_t strtouli(ConstString asString, int arg2, int arg3);
public:
    comanVelocityObserver();
    ~comanVelocityObserver();

    char                        info[SIZE_INFO];
    yarp::os::Semaphore         semaphore;

    /*Device Driver*/
    virtual bool open(yarp::os::Searchable &par);
    virtual bool close();
    bool fromConfig(yarp::os::Searchable &config);

    bool alloc(int njoints);
    bool init(void);


    //
    // IAnalogSensor Interface
    //

    virtual int read(yarp::sig::Vector &out);

    virtual int getState(int ch);

    virtual int getChannels();

    /* Calibrates the whole sensor.
     * @return status.
     */
    virtual int calibrateSensor();

    /* Calibrates the whole sensor, using an vector of calibration values.
     * @param value: a vector of calibration values.
     * @return status.
     */
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    /* Calibrates one single channel.
     * @param ch: channel number.
     * @return status.
     */
    virtual int calibrateChannel(int ch);

    /* Calibrates one single channel, using a calibration value.
     * @param ch: channel number.
     * @param value: calibration value.
     * @return status.
     */
    virtual int calibrateChannel(int ch, double value);
};

#endif // include guard __comanVelocityObserver_h__
