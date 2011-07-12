// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_NETWORK_GUI_H__
#define __GTKMM_ICUB_NETWORK_GUI_H__

#include <gtkmm.h>
#include <yarp/os/RateThread.h>
#include "iCubNetwork.h"
#include "iCubBoardGui.h"

class iCubNetworkGui : public iCubNetwork, public iCubInterfaceGuiRows
{
public:
    enum { ALARM_NONE,ALARM_HIGH,ALARM_LOW };

    iCubNetworkGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot)
        : iCubNetwork(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        for (int i=2; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *netBot=bot.get(i).asList();
            yarp::os::ConstString boardType=netBot->get(1).asList()->get(4).asString();

            if (boardType=="BLL")
            {
                mBoards.push_back(new iCubBLLBoardGui(refTreeModel,*baseRow,*netBot));
            }
            else if (boardType=="analog")
            {
                mBoards.push_back(new iCubAnalogBoardGui(refTreeModel,*baseRow,*netBot));
            }
        }

        mData.fromBottle(*(bot.get(1).asList()));

        mAlarmMask=new char[mNumRows];
        mAlarmState=new bool[mNumRows];

        for (int i=0; i<mNumRows; ++i)
        {
            mAlarmState[i]=false;
            mAlarmMask[i]=ALARM_NONE;
            if (mRowNames[i][0]=='!')
            {
                if (mRowNames[i][1]=='~')
                {
                    mAlarmMask[i]=ALARM_LOW;
                }
                else
                {
                    mAlarmMask[i]=ALARM_HIGH;
                }
            }
            
            if (mAlarmMask[i]==ALARM_HIGH)
            {
                mAlarmState[i]=mData.isHigh(i);                    
            }
            else if (mAlarmMask[i]==ALARM_LOW)
            {
                mAlarmState[i]=mData.isLow(i);
            }

            mRows[i][mColumns.mColValue]=mData.toString(i);
        }
    }

    virtual ~iCubNetworkGui()
    {
        delete [] mAlarmMask;
        delete [] mAlarmState;
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubNetwork::fromBottle(bot);

        for (int i=0; i<mNumRows; ++i)
        {
            if (mData.test(i))
            {
                if (mAlarmMask[i]==ALARM_HIGH)
                {
                    mAlarmState[i]=mData.isHigh(i);                    
                }
                else if (mAlarmMask[i]==ALARM_LOW)
                {
                    mAlarmState[i]=mData.isLow(i);
                }

                mRows[i][mColumns.mColValue]=mData.toString(i);
            }
        }
    }

    virtual bool hasAlarm()
    {
        bool alarm=false;

        for (int i=0; i<mNumRows; ++i)
        {
            if (mAlarmState[i])
            {
                alarm=true;
                mRows[i][mColumns.mColIcon]="(!)";
            }
            else
            {
                mRows[i][mColumns.mColIcon]="";
            }
        }

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i]->hasAlarm()) 
            {
                alarm=true;
            }
        }

        if (alarm)
        {
            mRows[0][mColumns.mColIcon]="(!)";
        }
        else
        {
            mRows[0][mColumns.mColIcon]="";
        }

        return alarm;
    }

protected:
    bool* mAlarmState;
    char* mAlarmMask;
};

#endif

