// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include "iCubNetwork.h"

////////////////////////////////////



bool iCubNetwork::findAndWrite(std::string addr,yarp::os::Value* data)
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
            mData.write(i,data[i]);
        }

        return true;
    }

    // for a board channel

    ++index;

    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<(int)mBoards.size(); ++i)
    {
        if (mBoards[i]->findAndWrite(addr,data))
        {
            return true;
        }
    }

    return false;
}
