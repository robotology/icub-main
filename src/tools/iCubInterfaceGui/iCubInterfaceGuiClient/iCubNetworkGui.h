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
#include "iCubBoardGui.h"

class iCubNetworkGui
{
public:
    enum { ALARM_NONE,ALARM_HIGH,ALARM_LOW };

    iCubNetworkGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &netConf)
    {
        mLocalData=new GuiRawDataArray(refTreeModel,parent,netConf.get(0).asList());

        mRoot=mLocalData->getRoot();

        for (int i=1; i<(int)netConf.size(); ++i)
        {
            mBoards.push_back(new iCubBoardGui(refTreeModel,*mRoot,*(netConf.get(i).asList())));
        }
    }

    virtual ~iCubNetworkGui()
    {
        delete mLocalData;

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i]!=NULL) delete mBoards[i];
        }
    }

    Gtk::TreeModel::Row *getRoot(){ return mRoot; }

    virtual void fromBottle(const yarp::os::Bottle &bot)
    {
        static const int LOCAL_DATA=0;

        for (int i=0; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *list=bot.get(i).asList();

            int index=list->get(0).asInt();

            if (index==LOCAL_DATA)
            {
                mLocalData->fromBottle(list->tail());
            }
            else
            {
                mBoards[index-1]->fromBottle(list->tail());
            }
        }

        mLocalData->getRoot();
    }

    int alarmLevel()
    {
        int alarm=mLocalData ? mLocalData->alarmLevel() : 0;

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i])
            {
                int childAlarmLevel=mBoards[i]->alarmLevel();
                
                if (childAlarmLevel>alarm)
                {
                    alarm=childAlarmLevel;
                }
            }
        }

        switch(alarm)
        {
        case 0:
            (*mRoot)[mColumns.mColIcon]=GuiRawData::mIconEmpty;
            (*mRoot)[mColumns.mColColor]=GuiRawData::mColorEmpty;
            (*mRoot)[mColumns.mColColorFg]=GuiRawData::mColorBlack;
            break;
        case 1:
            (*mRoot)[mColumns.mColIcon]=GuiRawData::mIconWarning;
            (*mRoot)[mColumns.mColColor]=GuiRawData::mColorWarning;
            (*mRoot)[mColumns.mColColorFg]=GuiRawData::mColorBlack;
            break;
        case 2:
            (*mRoot)[mColumns.mColIcon]=GuiRawData::mIconError;
            (*mRoot)[mColumns.mColColor]=GuiRawData::mColorError;
            (*mRoot)[mColumns.mColColorFg]=GuiRawData::mColorEmpty;
            break;
        }

        return alarm;
    }

    bool findAndReset(const Gtk::TreeModel::Row row)
    {
        if (mLocalData)
        {
            if (*(mLocalData->getRoot())==row)
            {
                mLocalData->reset();
                return true;
            }
            else
            {
                if (mLocalData->findAndReset(row)) return true;
            }
        }

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i])
            {
                if (*(mBoards[i]->getRoot())==row)
                {
                    mBoards[i]->reset();
                    return true;
                }
                else
                {
                    if (mBoards[i]->findAndReset(row)) return true;
                }
            }
        }

        return false;
    }

    void reset()
    {
        if (mLocalData) mLocalData->reset();

        for (int i=0; i<(int)mBoards.size(); ++i)
        {
            if (mBoards[i])
            {
                mBoards[i]->reset();
            }
        }
    }

protected:
    Gtk::TreeModel::Row *mRoot;
    GuiRawDataArray *mLocalData;
    std::vector<iCubBoardGui*> mBoards;

    ModelColumns mColumns;
};

#endif

