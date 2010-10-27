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
    //"Device identifier",	// Name of the yarp can device: pcan/cfw2
    //"Board ID",	            // The id with which the board is identified on the canbus
    "Channel",	            // The channel (boards can have up to 2 channels)
    "Joint",	            // Corresponding joint (for readability)

    "Status_messages_latency",         // Keep track of the time the last status message has been received (seconds)
    "Status_messages_latency_timeout", // If status messages latency > threshold (5s) raise an error
    "Encoder_latency",                 // Keep track of the time the last encoder reading has been received        
    "Encoder_latency_timeout",         // If encoder latency > threshold (5s) raise an error

    // device generated
    "Is_Fault_Ok",               // Status of the fault pin, general error
    "Fault_undervoltage",        // Power supply voltage is below minimum
    "Fault_overload",            // Hardware fault triggered by the operational amplifier
    "Fault_overcurrent",	     // Current exceeds maximum value
    "Fault_external",            // External fault button is pressed
    "Hall_sensor_error",	     // Brushless hall effect sensor error
    "Absolute_encoder_error",    // Read error in absolute position sensor
    "BusOff",
    "Can_Tx_Error_counter",	
    "Can_Rx_Error_counter",
    "Can_Tx_Overflow",	         // Canbus Tx Buffer overflow (firmware)
    "Can_Rx_Overrun",            // Canbus Rx buffer overflow (firmware)
    "Main_loop_overflow",        // Main loop exceeded requested period (>1ms, typically)
    "Over_temperature",	
    "Temp_sensor_error",         // Read error in temperature sensor
    "Control_mode",              // Status of the controller. This enumeration is illustrated below.
    NULL
};

yarp::os::Bottle iCubBLLChannel::toBottle(bool bConfig)
{
    return mData.toBottle(bConfig);
}

void iCubBLLChannel::fromBottle(yarp::os::Bottle& bot)
{
    mData.fromBottle(bot);
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
