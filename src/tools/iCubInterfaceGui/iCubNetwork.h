// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_NETWORK_H__
#define __GTKMM_ICUB_NETWORK_H__

#include <string>
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
        for (int i=0; i<mBoards.size(); ++i)
        {
            delete mBoards[i];
        }
    }

    enum
    {
    };

    enum
    {
    };

    enum
    {
    };

    inline bool operator==(iCubNetwork& n)
    {
        return name==n.mName;
    }

    void addBoard(iCubBoard *board)
    {
        mBoards.push_back(board);
    }

    bool findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt);

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
