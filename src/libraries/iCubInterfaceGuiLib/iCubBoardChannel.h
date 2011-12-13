// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_BOARD_CHANNEL_H__
#define __GTKMM_ICUB_BOARD_CHANNEL_H__

#include <string>
#include <vector>
#include <stdlib.h>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <iCub/LoggerInterfaces.h>

////////////////////////////////////

#define ONLY_DATA_FLAG 0
#define CONFIG_FLAG    1

#include "RawData.h"

////////////////////////////////////

class iCubBoardChannel
{
public:
    enum { ALARM_NONE,ALARM_HIGH,ALARM_LOW };

    iCubBoardChannel(int channel)
    {
        mChannel=channel;
        mData=NULL;
    }

public:
    virtual ~iCubBoardChannel()
    {
        if (mData) delete mData;
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data)=0;

    virtual yarp::os::Bottle getConfig()
    {
        return mData->getConfig();
    }

    /*
    virtual void setConfig(yarp::os::Bottle& bot)
    {
        mData->setConfig(bot);
    }
    */

    virtual yarp::os::Bottle toBottle()
    {
        return mData->toBottle();
    }

    virtual bool hasAlarm(){ return false; }

protected:
    int mChannel;
    RawDataArray *mData;
};

class iCubBLLChannel : public iCubBoardChannel
{
public:
    enum Index
    {
        // interface generated
        INT_Channel,	            // The channel (boards can have up to 2 channels)
        INT_Joint,	                // Corresponding joint (for readability)
        
        DOUBLE_Status_messages_latency,       // Keep track of the time the last status message has been received (seconds)
        BOOL_Status_messages_latency_timeout, // If status messages latency > threshold (5s) raise an error
        DOUBLE_Encoder_latency,               // Keep track of the time the last encoder reading has been received        
        BOOL_Encoder_latency_timeout,         // If encoder latency > threshold (5s) raise an error

        // device generated
        BOOL_Is_Fault_Ok,               // Status of the fault pin, general error
        BOOL_Fault_undervoltage,        // Power supply voltage is below minimum
        BOOL_Fault_overload,            // Hardware fault triggered by the operational amplifier
        BOOL_Fault_overcurrent,	        // Current exceeds maximum value
        BOOL_Fault_external,            // External fault button is pressed
        BOOL_Hall_sensor_error,	        // Brushless hall effect sensor error
        BOOL_Absolute_encoder_error,    // Read error in absolute position sensor
        BOOL_BusOff,
        INT_Can_Tx_Error_counter,	
        INT_Can_Rx_Error_counter,
        BOOL_Can_Tx_Overflow,	        // Canbus Tx Buffer overflow (firmware)
        BOOL_Can_Rx_Overrun,            // Canbus Rx buffer overflow (firmware)
        BOOL_Main_loop_overflow,        // Main loop exceeded requested period (>1ms, typically)
        BOOL_Over_temperature,	
        BOOL_Temp_sensor_error,         // Read error in temperature sensor
        INT_Control_mode,               // Status of the controller. This enumeration is illustrated below.
        NUM_ROWS
    };

    enum ControlMode
    { 
        MODE_IDLE,                  // no pwm enabled
        MODE_POSITION,              // position control
        MODE_VELOCITY,              // velocity control
        MODE_TORQUE,                // torque control (generates a given joint level torque) 
        MODE_IMPEDANCE,             // impedance control (generates a given stiffness around an equilibrium point)
        MODE_CALIB_ABS_POS_SENS,    // calibrate joint using an absolute position sensor
        MODE_CALIB_HARD_STOPS,      // calibrate joint using hardware limit
        MODE_HANDLE_HARD_STOPS,     // trying to get out from an hardware limit
        MODE_MARGIN_REACHED,        // trying to get within limits when the margin to limits is too small
        MODE_OPENLOOP               // receiving PWM values via canbus
    };

    /*
    iCubBLLChannel() : iCubBoardChannel(),mJoint(-1)
    {
        mData.setConfig(mRowNames);
    }
    */

    iCubBLLChannel(int channel,int joint) : iCubBoardChannel(channel),mJoint(joint)
    {
        mData=new RawDataArray(mRowNames);

        mData->findAndWrite(INT_Channel,yarp::os::Value(channel));
        mData->findAndWrite(INT_Joint,yarp::os::Value(joint));

        for (int i=DOUBLE_Status_messages_latency; i<(int)NUM_ROWS; ++i)
        {
            mData->findAndWrite(i,yarp::os::Value(0));
        }
    }

    virtual ~iCubBLLChannel()
    {
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);

protected:
    int mJoint;
    static const char *mRowNames[];
};

class iCubAnalogChannel : public iCubBoardChannel
{
public:
    enum Index
    {
        // interface generated
        INT_Channel,	            // The channel (boards can have up to 6 channels)
        
        DOUBLE_Status_messages_latency,       // Keep track of the time the last status message has been received (seconds)
        BOOL_Status_messages_latency_timeout, // If status messages latency > threshold (5s) raise an error
        
        // device generated
        BOOL_Is_Fault_Ok,               // Status of the fault pin, general error
        BOOL_Fault_undervoltage,        // Power supply voltage is below minimum
        BOOL_Fault_overload,            // Hardware fault triggered by the operational amplifier
        BOOL_Fault_overcurrent,	        // Current exceeds maximum value
        BOOL_Fault_external,            // External fault button is pressed
        BOOL_BusOff,
        INT_Can_Tx_Error_counter,	
        INT_Can_Rx_Error_counter,
        BOOL_Can_Tx_Overflow,	        // Canbus Tx Buffer overflow (firmware)
        BOOL_Can_Rx_Overrun,            // Canbus Rx buffer overflow (firmware)
        BOOL_Main_loop_overflow,        // Main loop exceeded requested period (>1ms, typically)
        NUM_ROWS
    };

    /*
    iCubAnalogChannel() : iCubBoardChannel()
    {
        mData.setConfig(mRowNames);
    }
    */

    iCubAnalogChannel(int channel) : iCubBoardChannel(channel)
    {
        mData=new RawDataArray(mRowNames);

        mData->findAndWrite(INT_Channel,yarp::os::Value(channel));

        for (int i=DOUBLE_Status_messages_latency; i<(int)NUM_ROWS; ++i)
        {
            mData->findAndWrite(i,yarp::os::Value(0));
        }
    }

    virtual ~iCubAnalogChannel()
    {
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);

protected:
    static const char *mRowNames[];
};

#endif


