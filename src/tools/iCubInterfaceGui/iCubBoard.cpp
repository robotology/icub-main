// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

////////////////////////////////////

#include "iCubBoard.h"

////////////////////////////////////

char *mRowNames[]=
{
    "Board Type",    // ="BLL"
    "Board ID",      //The id with which the board is identified on the canbus
    NULL
};

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

bool iCubBLLBoard::findAndRead(std::string addr,yarp::os::Value* data)
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