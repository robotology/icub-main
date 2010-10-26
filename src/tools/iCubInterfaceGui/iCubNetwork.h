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
    iCubNetwork()
    {
        mName="";
        mFile="";
        mDevice="";
    }

    iCubNetwork(std::string &name,std::string &file,std::string &device) 
        : mName(name),mFile(file),mDevice(device)
    {
        yarp::os::Value valName(name.c_str());
        yarp::os::Value identif(device.c_str());
        mData.write(STRING_Name,valName);
        mData.write(STRING_Device_identifier,identif);
    }

    ~iCubNetwork()
    {
        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i]!=NULL) delete mBoards[i];
        }
    }

    void setID(int ID)
    {
        mData.write(INT_Network_id,yarp::os::Value(ID));
    }

    enum Index
    {
        STRING_Name,
        STRING_Device_identifier, // Name of the yarp can device: pcan/cfw2
        INT_Network_id,	    // Usually a number for each device, from 0 to … n

        INT_Driver_Rx_ovf,	// Rx buffer overflow in device driver (# messages)
        INT_Driver_Tx_ovf,	// Tx buffer overflow in device driver (# messages)
        INT_Rx_Can_errors,	// Rx errors (can device)
        INT_Tx_Can_errors,	// Tx errors  (can device)
        INT_Rx_buffer_ovf, 	// Overflow Rx buffer (can device)
        
        BOOL_Bus_off, //Bus off flag
 
        DOUBLE_Requested_rate,         //Requested rate for the associated thread [ms]
        DOUBLE_Estimated_average_rate, // Estimated rate for the associated thread [ms]
        DOUBLE_Estimated_std_rate	   // Same as before, standard deviation, ms
    };

    /*
    inline bool operator==(iCubNetwork& n)
    {
        return mName==n.mName;
    }
    */

    void addBoard(iCubBoard* board)
    {
        mBoards.push_back(board);
    }

    bool findAndWrite(std::string addr,yarp::os::Value* data);

    yarp::os::Bottle toBottle(bool bConfig=false)
    {
        yarp::os::Bottle bot;

        bot.addList()=mData.toBottle(bConfig);

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            bot.addList()=mBoards[i]->toBottle(bConfig);
        }

        return bot;
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        mData.fromBottle(*(bot.get(0).asList()));
        
        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            mBoards[i]->fromBottle(*(bot.get(i+1).asList()));
        }
    }

    std::string mName,mFile,mDevice;

protected:
    std::vector<iCubBoard*> mBoards;

    RawData mData;
    static char* mRowNames[];
};

#endif
