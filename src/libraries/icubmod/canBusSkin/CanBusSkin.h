// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __SKIN_MESH_THREAD_H__
#define __SKIN_MESH_THREAD_H__

#include <string>
#include <vector>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Value.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class CanBusSkin : public RateThread, public yarp::dev::IAnalogSensor, public DeviceDriver 
{
private:
    /* *************************************************************************************** */
    // CAN Message parameters
    // 4C Message
    Bottle msg4C_Timer;
    Bottle msg4C_CDCOffsetL;
    Bottle msg4C_CDCOffsetH;
    Bottle msg4C_TimeL;
    Bottle msg4C_TimeH;
    
    //4E Message
    Bottle msg4E_Shift;
    Bottle msg4E_Shift3_1;
    Bottle msg4E_NoLoad;
    Bottle msg4E_Param;
    Bottle msg4E_EnaL;
    Bottle msg4E_EnaH;
    /* *************************************************************************************** */
    
protected:
    PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer inBuffer;
    CanBuffer outBuffer;
   
    yarp::os::Semaphore mutex;

    yarp::sig::VectorOf<int> cardId;
    int sensorsNum;

    yarp::sig::Vector data;

public:
    CanBusSkin(int period=20) : RateThread(period), mutex(1) {}

    ~CanBusSkin() {}

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
   
    
    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& v);
    virtual int calibrateChannel(int ch);
    virtual int calibrateChannel(int ch, double v);

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:
    /**
     * Checks that the given parameter list, extracted from the configuration file, is of the same lenght as the number of cards on the CAN bus.
     * If thins is not the case then the missing parameters in the list are initialised with default values.
     *
     * \param i_paramName The parameter name to be used in the debug message
     * \param i_paramList The parameter list
     * \param i_length The number of cards on the CAN bus
     * \param i_defaultValue The default value for the given parameter
     */
    void checkParameterListLength(const std::string &i_paramName, Bottle &i_paramList, const int &i_length, const Value &i_defaultValue);

    /**
     * Sends the CMD_TACT_SETUP 0x4C CAN message to the MTB boards.
     */
    void sendCANMessage4C(void);
    /**
     * Sends the CMD_TACT_SETUP2 0x4E CAN message to the MTB boards.
     */
    void sendCANMessage4E(void);
};

#endif
