// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "iCubBoardChannel.h"

yarp::os::Bottle iCubBLLChannel::toBottle(bool bConfig)
{
    yarp::os::Bottle bot;

    if (bConfig)
    {
        bot.addInt(CONFIG_FLAG);
        bot.addInt(mChannel);
        bot.addInt(mJoint);
    }
    else
    {
        bot.addInt(ONLY_DATA_FLAG);
    }

    yarp::os::Value data;
    for (int i=0; i<(int)mData.size(); ++i)
    {
        if (mData.read(i,data))
        {
            bot.addInt(i);
            bot.add(data);
        }
    }

    return bot;
}

void iCubBLLChannel::fromBottle(yarp::os::Bottle& bot)
{
    int i=1;

    if (bot.get(0).asInt()==CONFIG_FLAG)
    {
        i=3;
        mChannel=bot.get(1).asInt();
        mJoint=bot.get(2).asInt();
    }

    for (; i<bot.size(); i+=2)
    {
        int index=bot.get(i).asInt();
        yarp::os::Value data=bot.get(i+1);

        mData.write(index,data);
    }
}

bool iCubBLLChannel::findAndWrite(std::string addr,yarp::os::Value* data)
{
    int index=addr.find(",");

    std::string sCh=index<0?addr:addr.substr(0,index);

    if (sCh.length()==0) return false; // should never happen

    if (mChannel!=atoi(sCh.c_str())) return false;

    for (int i=0; i<(int)mData.size(); ++i)
    {
        mData.write(i,data[i]);
    }

    return true;
}
