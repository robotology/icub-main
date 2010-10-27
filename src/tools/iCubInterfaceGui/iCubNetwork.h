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
        mData.write(STRING_Name,yarp::os::Value(name.c_str()));
        mData.write(STRING_Device_identifier,yarp::os::Value(device.c_str()));

        for (int i=INT_Network_id; i<=(int)DOUBLE_Estimated_std_rate; ++i)
        {
            mData.write(i,yarp::os::Value(0));
        }
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

        yarp::os::Bottle data=mData.toBottle(bConfig);
        if (data.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(-1);
            addList.append(data);
        }

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            yarp::os::Bottle board=mBoards[i]->toBottle(bConfig);
            if (board.size())
            {
                yarp::os::Bottle &addList=bot.addList();
                addList.addInt(i);
                addList.append(board);
            }
        }

        return bot;
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        for (int i=1; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *list=bot.get(i).asList();

            if (list->get(0).asInt()==-1)
            {
                mData.fromBottle(*list);
            }
            else
            {
                mBoards[list->get(0).asInt()]->fromBottle(*list);
            }
        }
    }

    std::string mName,mFile,mDevice;

protected:
    std::vector<iCubBoard*> mBoards;

    RawData mData;
    static char* mRowNames[];
};

#endif
