// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ICUBINTERFACEGUI_RAW_DATA_H__
#define __ICUBINTERFACEGUI_RAW_DATA_H__

#include <string>
#include <vector>
#include <time.h>
#include <yarp/os/Bottle.h>

class RawData
{
public:
    RawData(const char* name)
    { 
        mStatusType=T_VOID;
        mNewData=false;
        mName=name;
    }
    virtual ~RawData(){}

    virtual bool write(const yarp::os::Value& data)
    {
        if (data.isInt())
        {
            if (mStatusType!=T_INT || mStatusInt!=data.asInt())
            {
                mStatusType=T_INT;
                mNewData=true;
                mStatusInt=data.asInt();
                return true;
            }
        }
        else if (data.isDouble())
        {
            if (mStatusType!=T_DOUBLE || mStatusDouble!=data.asDouble())
            {
                mStatusType=T_DOUBLE;
                mNewData=true;
                mStatusDouble=data.asDouble();
                return true;
            }
        }
        else if (data.isString())
        {
            if (mStatusType!=T_STRING || mStatusString!=data.asString())
            {
                mStatusType=T_STRING;
                mNewData=true;
                mStatusString=data.asString();
                return true;
            }
        }

        return false;
    }

    virtual yarp::os::Bottle toBottle()
    {
        mNewData=false;
        yarp::os::Bottle bot;
        switch(mStatusType)
        {
        case T_INT: 
            bot.addInt(mStatusInt); 
            break;
        case T_DOUBLE:
            bot.addDouble(mStatusDouble);
            break;
        case T_STRING:
            bot.addString(mStatusString.c_str());
            break;
        }
        
        return bot;
    }

    bool newData()
    {
        return mNewData;
    }

    std::string getConfig()
    { 
        return mName; 
    }

protected:
    bool mNewData;
    std::string mName;

    enum { T_VOID,T_INT,T_DOUBLE,T_STRING } mStatusType;
    int mStatusInt;
    double mStatusDouble;
    yarp::os::ConstString mStatusString;
};

class RawDataLogged : public RawData
{
public:
    RawDataLogged(const char* name) : RawData(name)
    {
    }

    ~RawDataLogged()
    {
    }

    bool write(const yarp::os::Value& data)
    {
        if (RawData::write(data))
        {
            if (mLog.size()>=64) mLog=mLog.tail();
            mLog.add(data);
            time_t seconds;
            time(&seconds);
            mLog.addInt((int)seconds);
            return true;
        }

        return false;
    }
    
    yarp::os::Bottle toBottle()
    {
        mNewData=false;
        yarp::os::Bottle bot=mLog;
        mLog.clear();
          
        return bot;
    }

protected:
    yarp::os::Bottle mLog;
};

class RawDataArray
{
public:
    RawDataArray(const char **names)
    {
        for (int i=0; names[i]; ++i)
        {
            mArray.push_back(names[i][0]=='!' ? new RawDataLogged(names[i]) : new RawData(names[i]));
        }
    }

    ~RawDataArray()
    {
        for (int i=0; i<(int)mArray.size(); ++i)
        {
            if (mArray[i]) delete mArray[i];
        }
    }

    virtual yarp::os::Bottle getConfig()
    {
        yarp::os::Bottle config;

        for (int i=0; i<(int)mArray.size(); ++i)
        {
            config.addString(mArray[i]->getConfig().c_str());
        }

        return config;
    }

    virtual yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle bot;

        for (int i=0; i<(int)mArray.size(); ++i)
        {
            if (mArray[i] && mArray[i]->newData())
            {
                bot.addInt(i);
                bot.addList()=mArray[i]->toBottle();
            }
        }

        return bot;
    }

    bool findAndWrite(int index,const yarp::os::Value& data)
    {
        if (index<0 || index>=(int)mArray.size()) return false;

        mArray[index]->write(data);

        return true;
    }

    int size(){ return mArray.size(); }

protected:
    std::vector<RawData*> mArray;
};

#endif


