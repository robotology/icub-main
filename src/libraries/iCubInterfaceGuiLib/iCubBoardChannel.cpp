// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "iCubBoardChannel.h"

const char* iCubBLLChannel::mRowNames[]=
{
    // interface generated
    //"Device identifier",	// Name of the yarp can device: pcan/cfw2
    //"Board ID",	            // The id with which the board is identified on the canbus
    "Channel",	            // The channel (boards can have up to 2 channels)
    "Joint",	            // Corresponding joint (for readability)

    "Status_messages_latency",         // Keep track of the time the last status message has been received (seconds)
    "!Status_messages_latency_timeout", // If status messages latency > threshold (5s) raise an error
    "Encoder_latency",                 // Keep track of the time the last encoder reading has been received        
    "Encoder_latency_timeout",         // If encoder latency > threshold (5s) raise an error

    // device generated
    "!Is_Fault_Ok",               // Status of the fault pin, general error
    "!Fault_undervoltage",        // Power supply voltage is below minimum
    "!Fault_overload",            // Hardware fault triggered by the operational amplifier
    "!Fault_overcurrent",	     // Current exceeds maximum value
    "!Fault_external",            // External fault button is pressed
    "!Hall_sensor_error",	     // Brushless hall effect sensor error
    "!Absolute_encoder_error",    // Read error in absolute position sensor
    "!BusOff",
    "Can_Tx_Error_counter",	
    "Can_Rx_Error_counter",
    "!Can_Tx_Overflow",	         // Canbus Tx Buffer overflow (firmware)
    "!Can_Rx_Overrun",            // Canbus Rx buffer overflow (firmware)
    "!Main_loop_overflow",        // Main loop exceeded requested period (>1ms, typically)
    "!Over_temperature",	
    "!Temp_sensor_error",         // Read error in temperature sensor
    "Control_mode",              // Status of the controller. This enumeration is illustrated below.
    NULL
};

bool iCubBLLChannel::findAndWrite(std::string addr,const yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sJoint=addr.substr(0,index);
    if (sJoint.length()==0) return false; // should never happen
    if (mJoint!=atoi(sJoint.c_str())) return false;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    if (index>=mData->size()) return false;

    mData->findAndWrite(index,data);

    return true;
}

const char* iCubAnalogChannel::mRowNames[]=
{
    // interface generated
    "Channel",	            // The channel (boards can have up to 2 channels)

    "Status_messages_latency",         // Keep track of the time the last status message has been received (seconds)
    "!Status_messages_latency_timeout", // If status messages latency > threshold (5s) raise an error

    // device generated
    "!Is_Fault_Ok",               // Status of the fault pin, general error
    "!Fault_undervoltage",        // Power supply voltage is below minimum
    "!Fault_overload",            // Hardware fault triggered by the operational amplifier
    "!Fault_overcurrent",	     // Current exceeds maximum value
    "!Fault_external",            // External fault button is pressed
    "!BusOff",
    "Can_Tx_Error_counter",	
    "Can_Rx_Error_counter",
    "!Can_Tx_Overflow",	         // Canbus Tx Buffer overflow (firmware)
    "!Can_Rx_Overrun",            // Canbus Rx buffer overflow (firmware)
    "!Main_loop_overflow",        // Main loop exceeded requested period (>1ms, typically)
    NULL
};

bool iCubAnalogChannel::findAndWrite(std::string addr,const yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sCh=addr.substr(0,index);
    if (sCh.length()==0) return false; // should never happen
    if (mChannel!=atoi(sCh.c_str())) return false;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    if (index>=mData->size()) return false;

    mData->findAndWrite(index,data);

    return true;
}

/*
bool iCubBLLChannel::findAndRead(std::string addr,yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sJoint=addr.substr(0,index);
    if (sJoint.length()==0) return false; // should never happen
    if (mJoint!=atoi(sJoint.c_str())) return false;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    if (index>=mData.size()) return false;

    mData.read(index,data);

    return true;
}
*/

/*
yarp::dev::LoggerDataRef* iCubAnalogChannel::getDataReference(std::string addr)
{
    int index=addr.find(",");
    if (index<0) return NULL; // should never happen

    std::string sCh=addr.substr(0,index);
    if (sCh.length()==0) return NULL; // should never happen
    if (mChannel!=atoi(sCh.c_str())) return NULL;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    return mData.getDataReference(index);
}
*/

/*
bool iCubAnalogChannel::findAndRead(std::string addr,yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sCh=addr.substr(0,index);
    if (sCh.length()==0) return false; // should never happen
    if (mChannel!=atoi(sCh.c_str())) return false;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    if (index>=mData.size()) return false;

    mData.read(index,data);

    return true;
}
*/

/*
yarp::dev::LoggerDataRef* iCubBLLChannel::getDataReference(std::string addr)
{
    int index=addr.find(",");
    if (index<0) return NULL; // should never happen

    std::string sJoint=addr.substr(0,index);
    if (sJoint.length()==0) return NULL; // should never happen
    if (mJoint!=atoi(sJoint.c_str())) return NULL;

    ++index;
    index=atoi(addr.substr(index,addr.length()-index).c_str());

    return mData.getDataReference(index);
}
*/