// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale, Francesco Giovannini
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __CANBUSSKIN_H__
#define __CANBUSSKIN_H__

#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>


#include "SkinConfigReader.h"
#include <SkinDiagnostics.h>


class CanBusSkin : public yarp::os::RateThread, public yarp::dev::IAnalogSensor, public yarp::dev::DeviceDriver 
{
private:

    bool _verbose;


    /* *************************************************************************************** */
    // CAN Message parameters
    // 4C Message
    yarp::os::Bottle msg4C_Timer;
    yarp::os::Bottle msg4C_CDCOffsetL;
    yarp::os::Bottle msg4C_CDCOffsetH;
    yarp::os::Bottle msg4C_TimeL;
    yarp::os::Bottle msg4C_TimeH;
    
    //4E Message
    yarp::os::Bottle msg4E_Shift;
    yarp::os::Bottle msg4E_Shift3_1;
    yarp::os::Bottle msg4E_NoLoad;
    yarp::os::Bottle msg4E_Param;
    yarp::os::Bottle msg4E_EnaL;
    yarp::os::Bottle msg4E_EnaH;
    /* *************************************************************************************** */

    /** Output port for skin diagnostics. */
    yarp::os::BufferedPort<yarp::sig::Vector> portSkinDiagnosticsOut;
    
    /****************** new cfg **********************************/
    SkinBoardCfgParam       _brdCfg;
    SkinTriangleCfgParam    _triangCfg;
    bool                    _newCfg;
    SkinConfigReader        _cfgReader;
    int                     _canBusNum;
    /*************************************************************/

    /****************** diagnostic********************************/
    bool _isDiagnosticPresent;       // is the diagnostic available from the firmware
    /*************************************************************/

protected:
    yarp::dev::PolyDriver driver;
    yarp::dev::ICanBus *pCanBus;
    yarp::dev::ICanBufferFactory *pCanBufferFactory;
    yarp::dev::CanBuffer inBuffer;
    yarp::dev::CanBuffer outBuffer;

    yarp::os::Semaphore mutex;

    /** The CAN net ID. */
    int netID;

    yarp::sig::VectorOf<int> cardId;
    int sensorsNum;

    yarp::sig::Vector data;

    /** The detected skin errors. These are used for diagnostics purposes. */
    yarp::sig::VectorOf<iCub::skin::diagnostics::DetectedError> errors;

public:
    CanBusSkin();
    ~CanBusSkin() {}

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();
   
    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
	virtual int calibrateSensor();
    virtual int calibrateChannel(int ch, double v);

    virtual int calibrateSensor(const yarp::sig::Vector& v);
    virtual int calibrateChannel(int ch);

private:
    /**
     * Extracts the detected errors and prints them out on a dedicated YARP port.
     *
     * \return true/false upon success/fail
     */
    bool diagnoseSkin(void);

    /**
     * Checks that the given parameter list, extracted from the configuration file, is of the same lenght as the number of cards on the CAN bus.
     * If thins is not the case then the missing parameters in the list are initialised with default values.
     *
     * \param i_paramName The parameter name to be used in the debug message
     * \param i_paramList The parameter list
     * \param i_length The number of cards on the CAN bus
     * \param i_defaultValue The default value for the given parameter
     */
    void checkParameterListLength(const std::string &i_paramName, yarp::os::Bottle &i_paramList, const int &i_length, const yarp::os::Value &i_defaultValue);
    
    /**
     * Sends the CMD_TACT_SETUP 0x4C CAN message to the MTB boards.
     */
    bool sendCANMessage4C(void);
    bool checkFirmwareVersion(void);

    /**
     * Sends the CMD_TACT_SETUP2 0x4E CAN message to the MTB boards.
     */
    bool sendCANMessage4E(void);

    /**
     * Sends the message of class polling with command @command and data @data.
     */
    bool sendCANMessage(uint8_t destAddr, uint8_t command, void *data);

    /**
     * Read configuration old style.
     */
    bool readOldConfiguration(yarp::os::Searchable& config);

    /**
     * Read configuration new style.
     */
    bool readNewConfiguration(yarp::os::Searchable& config);

    /**
     * Read special new configuration. It is used to configure board and/or triangles with values different from others
     */
    bool readNewSpecialConfiguration(yarp::os::Searchable& config);
};

#endif
