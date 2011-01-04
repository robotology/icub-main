// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_CHANNEL_GUI_H__
#define __GTKMM_ICUB_BOARD_CHANNEL_GUI_H__

#include <gtkmm.h>
#include "iCubBoardChannel.h"

//Tree model columns
class ModelColumns : public Gtk::TreeModel::ColumnRecord
{
public:
    ModelColumns()
    {
        add(mColIcon);
        add(mColName);
        add(mColValue);
    }

    Gtk::TreeModelColumn<Glib::ustring> mColIcon;
    //Gtk::TreeModelColumn<Glib::RefPtr<Gdk::Pixbuf> > mColIcon;
    int mColIconID;
    Gtk::TreeModelColumn<Glib::ustring> mColName;
    int mColNameID;
    Gtk::TreeModelColumn<Glib::ustring> mColValue;
    int mColValueID;
};

///////////////////////////////////////////////////

class iCubInterfaceGuiRows
{
public:
    iCubInterfaceGuiRows()
    {
        mRows=NULL;
    }

    Gtk::TreeModel::Row* createRows(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,const char *rowNames[])
    {
        mParent=&parent;

        mNumRows=0;

        for (; rowNames[mNumRows]; ++mNumRows){}

        mRows=new Gtk::TreeModel::Row[mNumRows];

        mRows[0]=*(refTreeModel->append(parent.children()));
        mRows[0][mColumns.mColIcon]="";
        mRows[0][mColumns.mColName]=rowNames[0];
        mRows[0][mColumns.mColValue]="";

        for (int i=1; i<mNumRows; ++i)
        {
            mRows[i]=*(refTreeModel->append(mRows[0].children()));
            char *name=(char*)rowNames[i];
            if (*name=='!') ++name;
            if (*name=='~') ++name;
            mRows[i][mColumns.mColIcon]="";
            mRows[i][mColumns.mColName]=name;
            mRows[i][mColumns.mColValue]="";
        }

        return &mRows[0];
    }

    virtual ~iCubInterfaceGuiRows()
    {
        if (mRows!=NULL) delete [] mRows;
    }

protected:
    int mNumRows;
    Gtk::TreeModel::Row *mParent;
    Gtk::TreeModel::Row *mRows;
    ModelColumns mColumns;
};

/*
class iCubChannelGui : public iCubInterfaceGuiRows
{
}
*/

class iCubBLLChannelGui : public iCubBLLChannel, public iCubInterfaceGuiRows
{
public:
    iCubBLLChannelGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot)
        : iCubBLLChannel(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        mData.fromBottle(bot);

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

    virtual ~iCubBLLChannelGui()
    {
        delete [] mAlarmMask;
        delete [] mAlarmState;
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubBLLChannel::fromBottle(bot);

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

class iCubAnalogChannelGui : public iCubAnalogChannel, public iCubInterfaceGuiRows
{
public:
    iCubAnalogChannelGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot)
        : iCubAnalogChannel(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        mData.fromBottle(bot);
        
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

    virtual ~iCubAnalogChannelGui()
    {
        delete [] mAlarmMask;
        delete [] mAlarmState;
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubAnalogChannel::fromBottle(bot);

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
