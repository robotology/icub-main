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
    virtual yarp::os::Bottle toBottle(bool bConfig=false)=0;
    virtual void fromBottle(yarp::os::Bottle& bot)=0;

    int getID()
    {
        return mID;
    }

protected:
    int mID;
};

class iCubBLLBoard : public iCubBoard
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

    yarp::os::Bottle toBottle(bool bConfig)
    {
        yarp::os::Bottle bot;

        if (bConfig)
        {
            bot.addInt(CONFIG_FLAG);
            bot.addString("BLL");
            bot.addInt(mID);
        }
        else
        {
            bot.addInt(ONLY_DATA_FLAG);
        }

        yarp::os::Bottle& chan0=bot.addList();
        chan0=mChannel[0]->toBottle();
        yarp::os::Bottle& chan1=bot.addList();
        chan1=mChannel[1]->toBottle();

        return bot;
    }

    void fromBottle(yarp::os::Bottle& bot)
    {
        int i=1;
        if (bot.get(0).asInt()==CONFIG_FLAG)
        {
            i=3;
            mID=bot.get(2).asInt();
        }

        mChannel[0]->fromBottle(*(bot.get(i  ).asList()));
        mChannel[1]->fromBottle(*(bot.get(i+1).asList()));
    }

    virtual bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt);

protected:
    iCubBLLChannel *mChannel[2];
};

#endif
