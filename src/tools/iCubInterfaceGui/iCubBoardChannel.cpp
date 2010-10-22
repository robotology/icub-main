// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "iCubBoardChannel.h"

char* iCubBLLChannel::mRowNames[]=
{
    // interface generated
    "Status messages latency",       // Keep track of the time the last status message has been received (seconds)
    "Encoder latency",               // Keep track of the time the last encoder reading has been received

    // interface generated
    "Status messages latency timeout", // If status messages latency > threshold (5s) raise an error
    "Encoder latency timeout",         // If encoder latency > threshold (5s) raise an error

    // device generated
    "Is Fault Ok",               // Status of the fault pin, general error
    "Fault undervoltage",        // Power supply voltage is below minimum
    "Fault overload",            // Hardware fault triggered by the operational amplifier
    "Fault overcurrent",	        // Current exceeds maximum value
    "Fault external",            // External fault button is pressed
    "Hall sensor error",	        // Brushless hall effect sensor error
    "Absolute encoder error",    // Read error in absolute position sensor
    "BusOff",		
    "CanTx Overflow",	        // Canbus Tx Buffer overflow (firmware)
    "Can Rx Overrun",            // Canbus Rx buffer overflow (firmware)
    "Main loop overflow",        // Main loop exceeded requested period (>1ms, typically)
    "Over temperature",	
    "Temp sensor error",         // Read error in temperature sensor

    // device denerated
    "Can Tx Error counter",	
    "Can Rx Error counter",
    "Control mode",               // Status of the controller.
    NULL
};

yarp::os::Bottle iCubBLLChannel::toBottle(bool bConfig)
{
    yarp::os::Bottle bot;

    if (bConfig)
    {
        bot.addInt(CONFIG_FLAG);
        bot.addInt(mChannel);
        bot.addInt(mJoint);
    }
    else
    {
        bot.addInt(ONLY_DATA_FLAG);
    }

    yarp::os::Value data;
    for (int i=0; i<(int)mData.size(); ++i)
    {
        if (mData.read(i,data))
        {
            bot.addInt(i);
            bot.add(data);
        }
    }

    return bot;
}

void iCubBLLChannel::fromBottle(yarp::os::Bottle& bot)
{
    int i=1;

    if (bot.get(0).asInt()==CONFIG_FLAG)
    {
        i=3;
        mChannel=bot.get(1).asInt();
        mJoint=bot.get(2).asInt();
    }

    for (; i<bot.size(); i+=2)
    {
        int index=bot.get(i).asInt();
        yarp::os::Value data=bot.get(i+1);

        mData.write(index,data);
    }
}

bool iCubBLLChannel::findAndWrite(std::string addr,yarp::os::Value* data)
{
    int index=addr.find(",");

    std::string sCh=index<0?addr:addr.substr(0,index);

    if (sCh.length()==0) return false; // should never happen

    if (mChannel!=atoi(sCh.c_str())) return false;

    for (int i=0; i<(int)mData.size(); ++i)
    {
        mData.write(i,data[i]);
    }

    return true;
}
