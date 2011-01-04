// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_GUI_H__
#define __GTKMM_ICUB_BOARD_GUI_H__

#include <gtkmm.h>
#include "iCubBoard.h"
#include "iCubBoardChannelGui.h"

class iCubBLLBoardGui : public iCubBLLBoard, public iCubInterfaceGuiRows
{
public:
    iCubBLLBoardGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot) 
        : iCubBLLBoard(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        mChannel[0]=new iCubBLLChannelGui(refTreeModel,*baseRow,*(bot.get(2).asList()));
        mChannel[1]=new iCubBLLChannelGui(refTreeModel,*baseRow,*(bot.get(3).asList()));

        mData.fromBottle(*(bot.get(1).asList()));

        mAlarmMask=new char[mNumRows];
        mAlarmState=new bool[mNumRows];

        for (int i=0; i<(int)mData.size(); ++i)
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

    virtual ~iCubBLLBoardGui()
    {
        delete [] mAlarmMask;
        delete [] mAlarmState;
    }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        iCubBLLBoard::fromBottle(bot);

        for (int i=0; i<(int)mData.size(); ++i)
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

        for (int i=0; i<2; ++i)
        {
            if (mChannel[i]->hasAlarm()) 
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

class iCubAnalogBoardGui : public iCubAnalogBoard, public iCubInterfaceGuiRows
{
public:
    iCubAnalogBoardGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot) 
        : iCubAnalogBoard(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        nChannels=bot.get(1).asList()->get(6).asInt();

        mChannel=NULL; //new iCubAnalogChannel*[nChannels];

        if (mChannel)
        {
            for (int c=0; c<nChannels; ++c)
            {
                mChannel[c]=new iCubAnalogChannelGui(refTreeModel,*baseRow,*(bot.get(c+2).asList()));
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

    virtual ~iCubAnalogBoardGui()
    {
        delete [] mAlarmMask;
        delete [] mAlarmState;
    }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        iCubAnalogBoard::fromBottle(bot);

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

        if (mChannel)
        {
            for (int i=0; i<nChannels; ++i)
            {
                if (mChannel[i]->hasAlarm()) 
                {
                    alarm=true;
                }
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
