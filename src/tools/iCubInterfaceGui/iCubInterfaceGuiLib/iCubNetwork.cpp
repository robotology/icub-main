// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include <stdlib.h>

#include "iCubNetwork.h"

////////////////////////////////////

const char* iCubNetwork::mRowNames[]=
{
        "Name",
        "Device identifier",// Name of the yarp can device: pcan/cfw2
        "Network id",	    // Usually a number for each device, from 0 to … n
        "Driver Rx ovf",	// Rx buffer overflow in device driver (# messages)
        "Driver Tx ovf",	// Tx buffer overflow in device driver (# messages)
        "Rx Can errors",	// Rx errors (can device)
        "Tx Can errors",	// Tx errors  (can device)
        "Rx buffer ovf", 	// Overflow Rx buffer (can device)
        "Tx buffer ovf", 	// Overflow Tx buffer (can device)
        
        "Bus off", //Bus off flag
 
        "Requested rate",         //Requested rate for the associated thread [ms]
        "Estimated average rate", // Estimated rate for the associated thread [ms]
        "Estimated std rate",	  // Same as before, standard deviation, ms
        NULL
};

bool iCubNetwork::findAndWrite(std::string address,const yarp::os::Value& data)
{
    int index=address.find(",");
    if (index<0) return false; // should never happen

    std::string netAddress=address.substr(0,index);

    if (netAddress.length()==0) return false; // should never happen

    if (netAddress!=mNetAddress) return false;

    ++index;
    address=address.substr(index,address.length()-index);
    index=address.find(",");

    // is the message for the network or for a board channel?
    if (index<0) // for the network
    {
        index=atoi(address.c_str());

        if (index>=mData.size()) return false;

        mData.write(index,data);

        return true;
    }

    // for a board channel
    for (int i=0; i<(int)mBoards.size(); ++i)
    {
        if (mBoards[i]->findAndWrite(address,data))
        {
            return true;
        }
    }

    return false;
}
/*
bool iCubNetwork::findAndRead(std::string addr,yarp::os::Value* data)
{
    int index=addr.find(",");

    std::string name=index<0?addr:addr.substr(0,index);

    if (name.length()==0) return false; // should never happen

    if (name!=mName) return false;

    // is the message for the network or for a board channel?
    if (index<0)
    {
        // for the network
        for (int i=0; i<(int)mData.size(); ++i)
        {
            mData.read(i,data[i]);
        }

        return true;
    }

    // for a board channel

    ++index;

    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<(int)mBoards.size(); ++i)
    {
        if (mBoards[i]->findAndRead(addr,data))
        {
            return true;
        }
    }

    return false;
}
*/