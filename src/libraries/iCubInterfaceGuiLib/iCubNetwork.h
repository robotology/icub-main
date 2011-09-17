// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_NETWORK_H__
#define __GTKMM_ICUB_NETWORK_H__

#include <string>
#include <vector>
#include <stdlib.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>

////////////////////////////////////

#include "iCubBoard.h"

////////////////////////////////////

class iCubNetwork
{
public:
    
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
        INT_Tx_buffer_ovf, 	// Overflow Tx buffer (can device)

        BOOL_Bus_off, //Bus off flag
 
        DOUBLE_Requested_rate,         //Requested rate for the associated thread [ms]
        DOUBLE_Estimated_average_rate, // Estimated rate for the associated thread [ms]
        DOUBLE_Estimated_std_rate	   // Same as before, standard deviation, ms
    };

    iCubNetwork(std::string &name,std::string &file,std::string &device) 
    {
        mName=name;
        mFile=file;
        mDevice=device;
        mLocalData=new RawDataArray(mRowNames);
        mLocalData->findAndWrite(STRING_Name,yarp::os::Value(mName.c_str()));
        mLocalData->findAndWrite(STRING_Device_identifier,yarp::os::Value(mDevice.c_str()));
    }

    ~iCubNetwork()
    {
        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i]!=NULL) delete mBoards[i];
        }

        if (mLocalData) delete mLocalData;
    }

    void setID(int ID)
    {
        mLocalData->findAndWrite(INT_Network_id,yarp::os::Value(ID));

        char buff[32];
        sprintf(buff,"%d",ID);
        mID=buff;

        mDevice_ID=mDevice+" "+mID;
    }

    inline bool operator==(std::string& name)
    {
        return name==mName;
    }

    void addBoard(iCubBoard* board)
    {
        mBoards.push_back(board);
    }

    //yarp::dev::LoggerDataRef* getDataReference(std::string addr);
    //bool findAndWrite(std::string addr,const yarp::os::Value& data);

    virtual yarp::os::Bottle getConfig()
    {
        yarp::os::Bottle config;

        config.addList()=mLocalData->getConfig();

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            config.addList()=mBoards[i]->getConfig();
        }

        return config;
    }

    virtual yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle bot;

        yarp::os::Bottle data=mLocalData->toBottle();

        if (data.size())
        {
            yarp::os::Bottle &tail=bot.addList();
            tail.addInt(0);
            tail.append(data);
        }

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            yarp::os::Bottle board=mBoards[i]->toBottle();
            
            if (board.size())
            {
                yarp::os::Bottle &tail=bot.addList();
                tail.addInt(i+1);
                tail.append(board);
            }
        }

        return bot;
    }

    bool findAndWrite(std::string address,const yarp::os::Value& data)
    {
        int separator=address.find(",");
        if (separator<0) return false; // should never happen
        std::string device_ID=address.substr(0,separator);
        if (device_ID.length()==0) return false; // should never happen
        if (device_ID!=mDevice_ID) return false;
        ++separator;
        address=address.substr(separator,address.length()-separator);

        separator=address.find(",");
        if (separator<0) return false; // should never happen
        std::string msgType=address.substr(0,separator);
        if (msgType.length()==0) return false; // should never happen

        if (msgType=="network")
        {
            ++separator;
            address=address.substr(separator,address.length()-separator);
            return mLocalData->findAndWrite(atoi(address.c_str()),data);
        }

        // for a or channel board channel
        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i]->findAndWrite(address,data))
            {
                return true;
            }
        }

        return false;
    }

    virtual bool hasAlarm(){ return false; }

    std::string mName,mFile,mDevice,mID,mDevice_ID;

protected:
    std::vector<iCubBoard*> mBoards;
    RawDataArray *mLocalData;

    static const char* mRowNames[];
};

#endif
