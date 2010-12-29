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

const char* iCubBLLBoard::mRowNames[]=
{
    "Device Type",   // ="BLL"
    "Board ID",       // The id with which the board is identified on the canbus
    NULL
};

const char* iCubAnalogBoard::mRowNames[]=
{
    "Device Type",    // ="analog"
    "Board ID",       // The id with which the board is identified on the canbus
    "#Channels",      // Number of channels
    NULL
};

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

bool iCubBLLBoard::findAndWrite(std::string addr,const yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sID=addr.substr(0,index);

    if (sID.length()==0) return false; //should never happen
    if (mID!=atoi(sID.c_str())) return false;

    ++index;
    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<2; ++i)
    {
        if (mChannel[i]->findAndWrite(addr,data))
        {
            return true;
        }
    }

    return false;
}

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

bool iCubAnalogBoard::findAndWrite(std::string addr,const yarp::os::Value& data)
{
    int index=addr.find(",");
    if (index<0) return false; // should never happen

    std::string sID=addr.substr(0,index);

    if (sID.length()==0) return false; //should never happen
    if (mID!=atoi(sID.c_str())) return false;

    ++index;
    addr=addr.substr(index,addr.length()-index);

    for (int i=0; i<nChannels; ++i)
    {
        if (mChannel[i]->findAndWrite(addr,data))
        {
            return true;
        }
    }

    return false;
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