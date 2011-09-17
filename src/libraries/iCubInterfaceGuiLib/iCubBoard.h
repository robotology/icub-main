// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_BOARD_H__
#define __GTKMM_ICUB_BOARD_H__

#include <string>
#include <stdlib.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>

////////////////////////////////////

#include "iCubBoardChannel.h"

////////////////////////////////////

class iCubBoard
{
public:
    enum { ALARM_NONE,ALARM_HIGH,ALARM_LOW };

    iCubBoard()
    {
    }

    virtual ~iCubBoard()
    {
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data)=0;
    
    virtual yarp::os::Bottle getConfig()=0;
    //virtual void setConfig(yarp::os::Bottle& bot)=0;

    virtual yarp::os::Bottle toBottle()=0;
    //virtual void fromBottle(yarp::os::Bottle& bot)=0;

    virtual bool hasAlarm(){ return false; }
};

class iCubBLLBoard : public iCubBoard
{
public:
    enum Index
    {
        INT_Board_ID,       //The id with which the board is identified on the canbus
        STRING_Board_Type,  // ="BLL"
        NUM_ROWS
    };

    /*
    iCubBLLBoard() : iCubBoard()
    {
        mLocalData=new RawDataArray(mRowNames);
    }
    */

    iCubBLLBoard(int ID,int j0,int j1) : iCubBoard(),mID(ID)
    {
        mLocalData=new RawDataArray(mRowNames);

        mLocalData->findAndWrite(INT_Board_ID,yarp::os::Value(ID));
        mLocalData->findAndWrite(STRING_Board_Type,yarp::os::Value(mBoardType.c_str()));

        mChannel[0]=new iCubBLLChannel(0,j0);
        mChannel[1]=new iCubBLLChannel(1,j1);
    }

    virtual ~iCubBLLBoard()
    {
        if (mChannel[0]!=NULL) delete mChannel[0];
        if (mChannel[1]!=NULL) delete mChannel[1];

        if (mLocalData) delete mLocalData;
    }

    virtual yarp::os::Bottle getConfig()
    {
        yarp::os::Bottle config;

        config.addList()=mLocalData->getConfig();

        for (int i=0; i<2; ++i)
        {
            config.addList()=mChannel[i]->getConfig();
        }

        return config;
    }

    /*
    virtual void setConfig(yarp::os::Bottle &config)
    {
        mLocalData->setConfig(*(config.get(0).asList()));

        for (int i=1; i<(int)config.size(); ++i)
        {
            mChannel[i-1]->setConfig(*(config.get(i).asList()));
        }
    }
    */

    virtual yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle bot;

        yarp::os::Bottle data=mLocalData->toBottle();
        if (data.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(0);
            addList.append(data);
        }

        for (int c=0; c<2; ++c)
        {
            yarp::os::Bottle chan=mChannel[c]->toBottle();
            if (chan.size())
            {
                yarp::os::Bottle &addList=bot.addList();
                addList.addInt(c+1);
                addList.append(chan);
            }
        }

        return bot;
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);

protected:
    static const std::string mBoardType;
    int mID;
    
    iCubBLLChannel *mChannel[2];
    
    RawDataArray *mLocalData;
    static const char *mRowNames[];
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class iCubAnalogBoard : public iCubBoard
{
public:
    enum Index
    {
        INT_Board_ID,       //The id with which the board is identified on the canbus
        STRING_Board_Type,  // ="analog"
        INT_Num_Channels,
        INT_Saturations,
        INT_Errors,
        INT_Timeouts,
        NUM_ROWS
    };

    /*
    iCubAnalogBoard() : iCubBoard()
    {
        nChannels=0;

        mLocalData=new RawDataArray(mRowNames);
    }
    */

    iCubAnalogBoard(int ID,int channels) : iCubBoard(),mID(ID)
    {
        nChannels=channels;

        mLocalData=new RawDataArray(mRowNames);

        mLocalData->findAndWrite(INT_Board_ID,yarp::os::Value(ID));
        mLocalData->findAndWrite(STRING_Board_Type,yarp::os::Value(mBoardType.c_str()));
        mLocalData->findAndWrite(INT_Num_Channels,yarp::os::Value(nChannels));

        /////////////////////////////////////////////////////////

        mChannel=NULL;

        if (nChannels)
        {
            mChannel=new iCubAnalogChannel*[nChannels];

            for (int c=0; c<nChannels; ++c)
            {
                mChannel[c]=new iCubAnalogChannel(c);
            }
        }
    }

    virtual ~iCubAnalogBoard()
    {
        if (mChannel)
        {
            for (int c=0; c<nChannels; ++c)
            {
                if (mChannel[c]!=NULL)
                {
                    delete mChannel[c];
                    mChannel[c]=NULL;
                }
            }

            delete [] mChannel;
            mChannel=NULL;
        }

        if (mLocalData) delete mLocalData;
    }

    virtual yarp::os::Bottle getConfig()
    {
        yarp::os::Bottle config;

        config.addList()=mLocalData->getConfig();

        for (int i=0; i<nChannels; ++i)
        {
            config.addList()=mChannel[i]->getConfig();
        }

        return config;
    }

    /*
    virtual void setConfig(yarp::os::Bottle &config)
    {
        mLocalData.setConfig(*config.get(0).asList());

        for (int i=1; i<(int)config.size(); ++i)
        {
            mChannel[i-1]->setConfig(*config.get(i).asList());
        }
    }
    */

    virtual yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle bot;

        yarp::os::Bottle data=mLocalData->toBottle();
        if (data.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(0);
            addList.append(data);
        }

        if (mChannel)
        {
            for (int c=0; c<nChannels; ++c)
            {
                yarp::os::Bottle chan=mChannel[c]->toBottle();
                if (chan.size())
                {
                    yarp::os::Bottle &addList=bot.addList();
                    addList.addInt(c+1);
                    addList.append(chan);
                }
            }
        }

        return bot;
    }

    virtual bool findAndWrite(std::string addr,const yarp::os::Value& data);

protected:
    static const std::string mBoardType;

    iCubAnalogChannel **mChannel;
    
    int mID;
    int nChannels;

    RawDataArray *mLocalData;
    static const char *mRowNames[];
};

#endif

