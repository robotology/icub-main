// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include <stdlib.h>
#include "iCubBoard.h"

////////////////////////////////////

const std::string iCubBLLBoard::mBoardType="BLL";

const char* iCubBLLBoard::mRowNames[]=
{
    "Board ID",       // The id with which the board is identified on the canbus
    "Device Type",   // ="BLL"
    NULL
};

const std::string iCubAnalogBoard::mBoardType="analog";

const char* iCubAnalogBoard::mRowNames[]=
{
    "Board ID",       // The id with which the board is identified on the canbus
    "Device Type",    // ="analog"
    "Channels",       // Number of channels
    "!Saturations",
    "!Errors",
    "!Timeouts",
    NULL
};

bool iCubBLLBoard::findAndWrite(std::string address,const yarp::os::Value& data)
{
    int separator=address.find(",");
    if (separator<0) return false; // should never happen
    std::string boardType=address.substr(0,separator);
    if (boardType.length()==0) return false; // should never happen
    if (boardType!=mBoardType) return false;
    ++separator;
    address=address.substr(separator,address.length()-separator);

    // address==[joint,]index

    separator=address.find(",");
    
    // is the message for the board or for a joint?
    if (separator<0) // for the board
    {
        // address==index
        mData.write(atoi(address.c_str()),data);

        return true;
    }
    
    // address==joint,index
   
    for (int i=0; i<2; ++i)
    {
        if (mChannel[i]->findAndWrite(address,data))
        {
            return true;
        }
    }

    return false;
}

bool iCubAnalogBoard::findAndWrite(std::string address,const yarp::os::Value& data)
{
    int separator=address.find(",");
    if (separator<0) return false; // should never happen
    std::string boardType=address.substr(0,separator);
    if (boardType.length()==0) return false; // should never happen
    if (boardType!=mBoardType) return false;
    ++separator;
    address=address.substr(separator,address.length()-separator);

    separator=address.find(",");
    if (separator<0) return false; // should never happen
    std::string sID=address.substr(0,separator);
    if (sID.length()==0) return false; //should never happen
    if (mID!=atoi(sID.c_str())) return false;
    ++separator;
    address=address.substr(separator,address.length()-separator);

    // address==[channel,]index

    separator=address.find(",");
    
    // is the message for the board or for a channel?
    if (separator<0) // for the board
    {
        // address==index
        mData.write(atoi(address.c_str()),data);

        return true;
    }
    
    // address==channel,index
    
    if (mChannel)
    {
        for (int i=0; i<nChannels; ++i)
        {
            if (mChannel[i]->findAndWrite(address,data))
            {
                return true;
            }
        }
    }

    return false;
}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////


/*
bool iCubBLLBoard::findAndRead(std::string addr,yarp::os::Value& data)
{
    int index=addr.find(",");
    std::string sID=index<0?addr:addr.substr(0,index);

    if (sID.length()==0) return false; //should never happen

    if (mID!=atoi(sID.c_str())) return false;

    ++index;

    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<2; ++i)
    {
        if (mChannel[i]->findAndRead(addr,data))
        {
            return true;
        }
    }

    return false;
}

yarp::dev::LoggerDataRef* iCubAnalogBoard::getDataReference(std::string addr)
{
    int index=addr.find(",");
    if (index<0) return NULL; // should never happen

    std::string sID=addr.substr(0,index);

    if (sID.length()==0) return NULL; //should never happen
    if (mID!=atoi(sID.c_str())) return NULL;

    ++index;
    addr=addr.substr(index,addr.length()-index);

    yarp::dev::LoggerDataRef* pRef=NULL;

    for (int i=0; i<nChannels; ++i)
    {
        if ((pRef=mChannel[i]->getDataReference(addr)))
        {
            return pRef;
        }
    }

    return NULL;
}

bool iCubAnalogBoard::findAndRead(std::string addr,yarp::os::Value& data)
{
    int index=addr.find(",");
    std::string sID=index<0?addr:addr.substr(0,index);

    if (sID.length()==0) return false; //should never happen

    if (mID!=atoi(sID.c_str())) return false;

    ++index;

    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<nChannels; ++i)
    {
        if (mChannel[i]->findAndRead(addr,data))
        {
            return true;
        }
    }

    return false;
}

yarp::dev::LoggerDataRef* iCubBLLBoard::getDataReference(std::string addr)
{
    int index=addr.find(",");
    if (index<0) return NULL; // should never happen

    std::string sID=addr.substr(0,index);

    if (sID.length()==0) return NULL; //should never happen
    if (mID!=atoi(sID.c_str())) return NULL;

    ++index;
    addr=addr.substr(index,addr.length()-index);

    yarp::dev::LoggerDataRef* pRef=NULL;    

    for (int i=0; i<2; ++i)
    {
        if ((pRef=mChannel[i]->getDataReference(addr)))
        {
            return pRef;
        }
    }

    return NULL;
}
*/