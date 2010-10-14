// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_CHANNEL_H__
#define __GTKMM_ICUB_BOARD_CHANNEL_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

#define ONLY_DATA_FLAG 0
#define CONFIG_FLAG    1

template<class T,int N> class RawData
{
public:
    RawData()
    {
        for (int i=0; i<N; ++i)
        {
            mFlag[i]=true;
        }
    }

    ~RawData()
    {
    }
    
    bool read(int index,T& d,bool rst=true)
    {
        if (index<0 || index>=N) return false;

        d=mData[index];

        bool tmp=mFlag[index];

        if (rst) mFlag[index]=false;

        return tmp;
    }

    bool write(int index,T d)
    {
        if (index<0 || index>=N) return false;

        if (mData[index]!=d)
        {
            mData[index]=d;
            mFlag[index]=true;
        }

        return true;
    }

protected:
    T mData[N];
    bool mFlag[N];
};

////////////////////////////////////

class iCubBoardChannel
{
public:
    iCubBoardChannel(int ch) : mChannel(ch)
    {
    }

    virtual ~iCubBoardChannel()
    {
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)=0;

    virtual yarp::os::Bottle toBottle(bool bConfig=false)=0;
    virtual void fromBottle(yarp::os::Bottle& bot)=0;

protected:
    int mChannel;
};

class iCubBLLChannel : public iCubBoardChannel
{
public:
    iCubBLLChannel(int ch,int j) : iCubBoardChannel(ch),mJoint(j)
    {
    }

    virtual ~iCubBLLChannel()
    {
    }    

    enum DoubleIndex
    {
        // interface generated
        DOUBLE_Status_messages_latency,       // Keep track of the time the last status message has been received (seconds)
        DOUBLE_Encoder_latency,               // Keep track of the time the last encoder reading has been received
        DOUBLE_NUM
    };

    enum BoolIndex
    {
        // interface generated
        BOOL_Status_messages_latency_timeout, // If status messages latency > threshold (5s) raise an error
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
        BOOL_CanTx_Overflow,	        // Canbus Tx Buffer overflow (firmware)
        BOOL_Can_Rx_Overrun,            // Canbus Rx buffer overflow (firmware)
        BOOL_Main_loop_overflow,        // Main loop exceeded requested period (>1ms, typically)
        BOOL_Over_temperature,	
        BOOL_Temp_sensor_error,         // Read error in temperature sensor
        BOOL_NUM
    };

    enum IntIndex
    {
        // device denerated
        INT_Can_Tx_Error_counter,	
        INT_Can_Rx_Error_counter,
        INT_Control_mode,               // Status of the controller. This enumeration is illustrated below.
        INT_NUM
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

    virtual yarp::os::Bottle toBottle(bool bConfig=false);
    virtual void fromBottle(yarp::os::Bottle& bot);

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt);

protected:
    int mJoint; // Corresponding joint (for readability)

    RawData<double,DOUBLE_NUM> mDoubleData;
    RawData<bool,BOOL_NUM> mBoolData;
    RawData<int,INT_NUM> mIntData;
};

#endif
