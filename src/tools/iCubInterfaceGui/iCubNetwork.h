// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_NETWORK_H__
#define __GTKMM_ICUB_NETWORK_H__

#include <string>
#include <vector>
#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>

////////////////////////////////////

#include "iCubBoard.h"

////////////////////////////////////

class iCubNetwork
{
public:
    iCubNetwork(std::string &name,
              std::string &file,
              std::string &device,
              std::string &canBusDevice,
              int threadRate)
        : 
            mName(name),
            mFile(file),
            mDevice(device),
            mCanBusDevice(canBusDevice),
            mThreadRate(threadRate) 
    {
    }

    ~iCubNetwork()
    {
        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            delete mBoards[i];
        }
    }
    
    enum
    {
        INT_Network_id,	    // Usually a number for each device, from 0 to … n
        INT_Driver_Rx_ovf,	// Rx buffer overflow in device driver (# messages)
        INT_Driver_Tx_ovf,	// Tx buffer overflow in device driver (# messages)
        INT_Rx_Can_errors,	// Rx errors (can device)
        INT_Tx_Can_errors,	// Tx errors  (can device)
        INT_Rx_buffer_ovf, 	// Overflow Rx buffer (can device)
        INT_NUM
    }; 

    enum
    {
        BOOL_Bus_off, //Bus off flag
        BOOL_NUM
    };
    
    enum
    {
        DOUBLE_Requested_rate,         //Requested rate for the associated thread [ms]
        DOUBLE_Estimated_average_rate, // Estimated rate for the associated thread [ms]
        DOUBLE_Estimated_std_rate,	   // Same as before, standard deviation, ms
        DOUBLE_NUM
    };

    inline bool operator==(iCubNetwork& n)
    {
        return mName==n.mName;
    }

    void addBoard(iCubBoard *board)
    {
        mBoards.push_back(board);
    }

    bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt);

    yarp::os::Bottle toBottle(bool bConfig=false)
    {
        yarp::os::Bottle bot;

        if (bConfig)
        {
            bot.addInt(CONFIG_FLAG);
            bot.addString(mName.c_str());
            bot.addString(mFile.c_str());
            bot.addString(mDevice.c_str());
            bot.addString(mCanBusDevice.c_str());
            bot.addInt(mThreadRate);
        }
        else
        {
            bot.addInt(ONLY_DATA_FLAG);
        }

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            yarp::os::Bottle& board=bot.addList();
            board=mBoards[i]->toBottle();
        }

        return bot;
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        int i0=1;
        if (bot.get(0).asInt()==CONFIG_FLAG)
        {
            i0=6;
            mName=bot.get(1).asString().c_str();
            mFile=bot.get(2).asString().c_str();;
            mDevice=bot.get(3).asString().c_str();;
            mCanBusDevice=bot.get(4).asString().c_str();
            mThreadRate=bot.get(5).asInt();
        }

        for (int i=i0; i<(int)bot.size(); ++i)
        {
            mBoards[i-i0]->fromBottle(*(bot.get(i).asList()));
        }
    }

    std::string mName;
    std::string mFile;
    std::string mDevice;
    std::string mCanBusDevice;
    int mThreadRate;

protected:
    std::vector<iCubBoard*> mBoards;

    RawData<double,DOUBLE_NUM> mDoubleData;
    RawData<bool,BOOL_NUM> mBoolData;
    RawData<int,INT_NUM> mIntData;
};

#endif
