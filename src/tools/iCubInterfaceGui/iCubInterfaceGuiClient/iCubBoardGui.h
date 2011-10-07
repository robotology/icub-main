// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_BOARD_GUI_H__
#define __GTKMM_ICUB_BOARD_GUI_H__

#include <gtkmm.h>
#include "GuiRawData.h"

class iCubBoardGui
{
public:
    iCubBoardGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &boardConf) 
    {
        nModules=boardConf.size();
        mChannel=new GuiRawDataArray*[nModules];

        mChannel[0]=new GuiRawDataArray(refTreeModel,parent,boardConf.get(0).asList());

        mRoot=mChannel[0]->getRoot();

        for (int i=1; i<nModules; ++i)
        {
            yarp::os::Bottle *rowNames=boardConf.get(i).asList();

            mChannel[i]=new GuiRawDataArray(refTreeModel,*(mChannel[0]->getRoot()),boardConf.get(i).asList());
        }
    }

    virtual ~iCubBoardGui()
    {
        if (mChannel)
        {
            for (int i=0; i<nModules; ++i)
            {
                if (mChannel[i]) delete mChannel[i];
            }
    
            delete [] mChannel;
        }
    }

    Gtk::TreeModel::Row *getRoot(){ return mRoot; }

    virtual void fromBottle(const yarp::os::Bottle& bot)
    {
        for (int i=0; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *list=bot.get(i).asList();
            mChannel[list->get(0).asInt()]->fromBottle(list->tail());
        }
    }

    int alarmLevel()
    {
        int alarm=0;

        for (int i=0; i<nModules; ++i)
        {
            if (mChannel[i])
            {
                int childAlarmLevel=mChannel[i]->alarmLevel();
                
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
        for (int i=0; i<nModules; ++i)
        {
            if (mChannel[i])
            {
                if (*(mChannel[i]->getRoot())==row)
                {
                    mChannel[i]->reset();
                    return true;
                }
                else
                {
                    if (mChannel[i]->findAndReset(row)) return true;
                }
            }
        }

        return false;
    }

    void reset()
    {
        for (int i=0; i<nModules; ++i)
        {
            if (mChannel[i]) mChannel[i]->reset();
        }
    }

protected:
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    Gtk::TreeModel::Row *mRoot;
    ModelColumns mColumns;

    int nModules;
    GuiRawDataArray **mChannel;
};

#endif
