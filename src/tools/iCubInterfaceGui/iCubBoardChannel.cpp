// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "iCubBoardChannel.h"

iCubBLLChannel::iCubBLLChannel(int ch,int j) : iCubBoardChannel(ch),mJoint(j)
{
    for (int i=0; i<(int)DOUBLE_NUM; ++i)
    {
        mDoubleFlag[i]=true;
    }

    for (int i=0; i<(int)BOOL_NUM; ++i)
    {
        mBoolFlag[i]=true;
    }

    for (int i=0; i<(int)INT_NUM; ++i)
    {
        mIntFlag[i]=true;
    }
}

bool iCubBLLChannel::write(int index,double d)
{
    if (index<0 || index>=DOUBLE_NUM) return false;

    if (mDoubleData[index]!=d)
    {
        mDoubleData[index]=d;
        mDoubleFlag[index]=true;
    }

    return true;
}

bool iCubBLLChannel::write(int index,bool b)
{
    if (index<0 || index>=BOOL_NUM) return false;

    if (mBoolData[index]!=d)
    {
        mBoolData[index]=d;
        mBoolFlag[index]=true;
    }

    return true;
}

bool iCubBLLChannel::write(int index,int d)
{
    if (index<0 || index>=INT_NUM) return false;

    if (mIntData[index]!=d)
    {
        mIntData[index]=d;
        mIntFlag[index]=true;
    }

    return true;
}

bool iCubBLLChannel::read(int index,double& d,bool rst)
{
    if (index<0 || index>=DOUBLE_NUM) return false;

    d=mDoubleData[index];

    bool tmp=mDoubleFlag[index];

    if (rst) mDoubleFlag[index]=false;

    return tmp;
}

bool iCubBLLChannel::read(int index,bool& d,bool rst)
{
    if (index<0 || index>=BOOL_NUM) return false;

    d=mBoolData[index];

    bool tmp=mBoolFlag[index];

    if (rst) mBoolFlag[index]=false;

    return tmp;
}

bool iCubBLLChannel::read(int index,int& d,bool rst)
{
    if (index<0 || index>=INT_NUM) return false;

    d=mIntData[index];

    bool tmp=mIntFlag[index];

    if (rst) mIntFlag[index]=false;

    return tmp;
}

yarp::os::Bottle iCubBLLChannel::toBottle()
{
    yarp::os::Bottle bot;

    bot.addInt(mChannel);
    bot.addInt(mJoint);

    double d;
    for (int i=0; i<(int)DOUBLE_NUM; ++i)
    {
        if (read(i,d))
        {
            bot.addInt(i);
            bot.addDouble(d);
        }
    }

    bool d;
    for (int i=0; i<(int)BOOL_NUM; ++i)
    {
        if (read(i,d))
        {
            bot.addInt(i);
            bot.addVocab(d?'T':'F');
        }
    }

    int d;
    for (int i=0; i<(int)INT_NUM; ++i)
    {
        if (read(i,d))
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
            mDoubleData[index]=data.asDouble();
            mDoubleFlag[index]=true;
        }
        else if (data.isVocab())
        {
            mBoolData[index]=data.asVocab()=='T';
            mBoolFlag[index]=true;
        }
        else if (data.isInt())
        {
            mIntData[index]=data.asInt();
            mIntFlag[index]=true;
        }
    }
}

bool iCubBLLChannel::findAndWrite(std::string addr,double* dataDouble,bool* dataBool,int* dataInt)
{
    int index=addr.find(",");

    if (index<0) return false;

    std::string sCh=addr.substr(0,index);

    int ch=atoi(sCh.c_str());

    if (ch!=mChannel) return false;

    for (int i=0; i<(int)DOUBLE_NUM; ++i)
    {
        write(i,dataDouble[i]);
    }

    for (int i=0; i<(int)BOOL_NUM; ++i)
    {
        write(i,dataBool[i]);
    }

    for (int i=0; i<(int)INT_NUM; ++i)
    {
        write(i,dataInt[i]);
    }
}
