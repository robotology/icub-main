// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_H__
#define __GTKMM_ICUB_BOARD_H__

#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

#include "iCubBoardChannel.h"

////////////////////////////////////

class iCubBoard
{
public:
    iCubBoard(int ID) : mID(ID)
    {
    }

    virtual ~iCubBoard()
    {
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)=0;

protected:
    const int mID;
};

class iCubBLLBoard : iCubBoard
{
public:
    iCubBLLBoard(int ID,int j0,int j1) : iCubBoard(ID)
    {
        mChannel[0]=new iCubBLLChannel(0,j0);
        mChannel[1]=new iCubBLLChannel(1,j1);
    }

    virtual ~iCubBLLBoard()
    {
        delete mChannel[0];
        delete mChannel[1];
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt);

protected:
    iCubBLLChannel *mChannel[2];
};

#endif
