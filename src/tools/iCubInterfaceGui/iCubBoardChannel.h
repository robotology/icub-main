// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_CHANNEL_H__
#define __GTKMM_ICUB_BOARD_CHANNEL_H__

#include <string>
#include <vector>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

#define ONLY_DATA_FLAG 0
#define CONFIG_FLAG    1

class RawData
{
public:
    RawData()
    {
    }

    ~RawData()
    {
    }

    int size(){ return mData.size(); }

    yarp::os::Bottle toBottle(bool bConfig=false)
    {
        yarp::os::Bottle bot;

        for (int i=0; i<(int)mData.size(); ++i)
        {
            if (bConfig || mFlag[i])
            {
                mFlag[i]=false;
                bot.addInt(i);
                bot.add(mData[i]);
            }
        }

        return bot;
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        for (int i=0; i<bot.size(); i+=2)
        {
            int index=bot.get(i).asInt();
            mData[index]=bot.get(i+1);
            mFlag[index]=true;
        }
    }

    bool test(int index,bool reset=true)
    {
        if (index<0 || index>=(int)mData.size()) return false;

        bool tmp=mFlag[index];

        if (reset) mFlag[index]=false;

        return tmp;
    }

    std::string toString(int index)
    {
        if (index<0 || index>=(int)mData.size()) return std::string();

        return std::string(mData[index].toString().c_str());
    }

    bool read(int index,yarp::os::Value& data,bool rst=true)
    {
        if (index<0 || index>=(int)mData.size()) return false;

        data=mData[index];

        bool tmp=mFlag[index];

        if (rst) mFlag[index]=false;

        return tmp;
    }

    bool write(int index,yarp::os::Value& data)
    {
        if (index<0) return false;
        
        if (index>=(int)mData.size())
        {
            int oldSize=mData.size();
            mFlag.resize(index+1);
            for (int i=oldSize; i<index; ++i) mFlag[i]=true;

            mData.resize(index+1);
        }

        if (mData[index]!=data)
        {
            mData[index]=data;
            mFlag[index]=true;
        }

        return true;
    }

protected:
    std::vector<yarp::os::Value> mData;
    std::vector<bool> mFlag;
};

////////////////////////////////////

class iCubBoardChannel
{
public:
    iCubBoardChannel()
    {
    }

    virtual ~iCubBoardChannel()
    {
    }

    virtual bool findAndWrite(std::string addr,yarp::os::Value* data)=0;
    virtual yarp::os::Bottle toBottle(bool bConfig=false)=0;
    virtual void fromBottle(yarp::os::Bottle& bot)=0;
};

#define UNASSIGNED -1

class iCubBLLChannel : public iCubBoardChannel
{
public:
    iCubBLLChannel(int channel=UNASSIGNED,int joint=UNASSIGNED) : iCubBoardChannel(),mData(),mChannel(channel)
    {
        mData.write(INT_Channel,yarp::os::Value(channel));
        mData.write(INT_Joint,yarp::os::Value(joint));
    }

    virtual ~iCubBLLChannel()
    {
    }

    enum Index
    {
        // interface generated
        //STRING_Device_identifier,	// Name of the yarp can device: pcan/cfw2
        //INT_Board_ID,	            // The id with which the board is identified on the canbus
        
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
        INT_Control_mode               // Status of the controller. This enumeration is illustrated below.
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
    virtual bool findAndWrite(std::string addr,yarp::os::Value* data);

protected:
    int mChannel;
    RawData mData;
    static char *mRowNames[];
};

#endif
