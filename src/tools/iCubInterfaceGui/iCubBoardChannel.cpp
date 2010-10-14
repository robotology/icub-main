// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "iCubBoardChannel.h"

yarp::os::Bottle iCubBLLChannel::toBottle()
{
    yarp::os::Bottle bot;

    bot.addInt(mChannel);
    bot.addInt(mJoint);

    double d;
    for (int i=0; i<(int)DOUBLE_NUM; ++i)
    {
        if (mDoubleData.read(i,d))
        {
            bot.addInt(i);
            bot.addDouble(d);
        }
    }

    bool d;
    for (int i=0; i<(int)BOOL_NUM; ++i)
    {
        if (mBoolData.read(i,d))
        {
            bot.addInt(i);
            bot.addVocab(d?'T':'F');
        }
    }

    int d;
    for (int i=0; i<(int)INT_NUM; ++i)
    {
        if (mBoolData.read(i,d))
        {
            bot.addInt(i);
            bot.addInt(d);
        }
    }
}

void iCubBLLChannel::fromBottle(yarp::os:Bottle& bot)
{
    double d;
    for (int i=2; i<bot.size(); i+=2)
    {
        int index=bot.get(i).asInt();
        yarp::os:Value data=bot.get(i+1);

        if (data.isDouble())
        {
            mDoubleData.write(index,data.asDouble());
        }
        else if (data.isVocab())
        {
            mBoolData.write(index,data.asVocab()=='T');
        }
        else if (data.isInt())
        {
            mIntData.write(index,data.asInt());
        }
    }
}

bool iCubBLLChannel::findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
{
    int index=addr.find(",");

    std::string sCh=index<0?addr:addr.substr(0,index);

    if (sCh.length()==0) return false; // should never happen

    if (mChannel!=atoi(sCh.c_str())) return false;

    for (int i=0; i<(int)DOUBLE_NUM; ++i)
    {
        mDoubleData.write(i,dataDouble[i]);
    }

    for (int i=0; i<(int)BOOL_NUM; ++i)
    {
        mBoolData.write(i,dataBool[i]);
    }

    for (int i=0; i<(int)INT_NUM; ++i)
    {
        mIntData.write(i,dataInt[i]);
    }
}
