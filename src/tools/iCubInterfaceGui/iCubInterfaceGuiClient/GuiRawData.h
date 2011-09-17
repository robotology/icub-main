// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_BOARD_CHANNEL_GUI_H__
#define __GTKMM_ICUB_BOARD_CHANNEL_GUI_H__

#include <gtkmm.h>
#include "RawData.h"

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

class GuiRawData
{
public:
    GuiRawData(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,const char* name)
    { 
        mStatusRow=*(refTreeModel->append(parent.children()));

        mStatusRow[mColumns.mColName]=name;
        mStatusRow[mColumns.mColValue]="";
        mStatusRow[mColumns.mColIcon]="";
    }

    virtual ~GuiRawData(){}

    virtual void reset(){}

    virtual int alarmLevel(){ return 0; }

    Gtk::TreeModel::Row *getRoot(){ return &mStatusRow; }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        mStatusRow[mColumns.mColValue]=bot.get(0).toString().c_str();
    }

protected:
    Gtk::TreeModel::Row mStatusRow;
    ModelColumns mColumns;
};

///////////////////////////////////////

class GuiRawDataLogged : public GuiRawData
{
public:
    GuiRawDataLogged(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,const char* name)
        : GuiRawData(refTreeModel,parent,name)
    {
        mRefTreeModel=refTreeModel;
        mAlarm=0;
    }

    ~GuiRawDataLogged()
    {
        reset();
    }

    void reset()
    {  
        while(!mStatusRow->children().empty())
        {
            mRefTreeModel->erase(mStatusRow->children().begin());
        }

        /*
        for (Gtk::TreeIter iter=mStatusRow.children().begin(); iter!=mStatusRow.children().end(); ++iter)
        {
            mRefTreeModel->erase(iter);
        }
        */
    }

    void fromBottle(yarp::os::Bottle& bot)
    {
        int top=(int)bot.size()-2;

        for (int i=0; i<=top; i+=2)
        {
            Gtk::TreeModel::Row newRow=*(mRefTreeModel->prepend(mStatusRow.children()));
            //time_t seconds;
            //time(&seconds);
            time_t seconds=(time_t)(bot.get(i+1).asInt());
            std::string timeStamp=ctime(&seconds);
            newRow[mColumns.mColName]=timeStamp.substr(0,timeStamp.length()-1).c_str();
            newRow[mColumns.mColValue]=bot.get(i).toString().c_str();
            newRow[mColumns.mColIcon]=bot.get(i).asInt() ? "(!)" : "";
        }

        mStatusRow[mColumns.mColValue]=bot.get(top).toString().c_str();

        if (bot.get(top).asInt())
        {
            mAlarm=2;
        }
        else
        {
            if (mAlarm) mAlarm=1;
        }
        
        switch(mAlarm)
        {
        case 0:
            mStatusRow[mColumns.mColIcon]="";
            break;
        case 1:
            mStatusRow[mColumns.mColIcon]="*";
            break;
        case 2:
            mStatusRow[mColumns.mColIcon]="(!)";
            break;
        }
    }

    int alarmLevel(){ return mAlarm; }

protected:
    int mAlarm;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
};

///////////////////////////////////////////////////

class GuiRawDataArray
{
public:
    GuiRawDataArray(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle *rowNames)
    {
        mArray.clear();

        yarp::os::ConstString name=rowNames->get(0).toString();

        if (name[0]=='!')
        {
            mArray.push_back(new GuiRawDataLogged(refTreeModel,parent,name.c_str()+1));
        }
        else
        {
            mArray.push_back(new GuiRawData(refTreeModel,parent,name.c_str()));
        }
        
        mRoot=mArray[0]->getRoot();

        for (int i=1; i<rowNames->size(); ++i)
        {
            name=rowNames->get(i).toString();
            
            if (name[0]=='!')
            {
                mArray.push_back(new GuiRawDataLogged(refTreeModel,*mRoot,name.c_str()+1));
            }
            else
            {
                mArray.push_back(new GuiRawData(refTreeModel,*mRoot,name.c_str()));
            }
        }
    }

    virtual ~GuiRawDataArray()
    {
        for (int i=0; i<(int)mArray.size(); ++i)
        {
            if (mArray[i]) delete mArray[i];
        }
    }

    Gtk::TreeModel::Row *getRoot(){ return mRoot; }

    void reset()
    {
        for (int i=0; i<(int)mArray.size(); ++i)
        {
            mArray[i]->reset();
        }
    }

    bool findAndReset(const Gtk::TreeModel::Row row)
    {
        for (int i=0; i<(int)mArray.size(); ++i)
        {
            if (mArray[i])
            {
                if (*(mArray[i]->getRoot())==row)
                {
                    mArray[i]->reset();
                    return true;
                }
            }
        }

        return false;
    }

    void reset(int i)
    {
        if (i>=0 && i<(int)mArray.size())
        {
            mArray[i]->reset();
        }
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        for (int i=0; i<bot.size(); i+=2)
        {
            mArray[bot.get(i).asInt()]->fromBottle(*(bot.get(i+1).asList()));
        }
    }

    int alarmLevel()
    {
        int alarm=0;

        for (int i=0; i<(int)mArray.size(); ++i)
        {
            if (mArray[i])
            {
                int childAlarmLevel=mArray[i]->alarmLevel();
                
                if (childAlarmLevel>alarm)
                {
                    alarm=childAlarmLevel;
                }
            }
        }

        switch(alarm)
        {
        case 0:
            (*mRoot)[mColumns.mColIcon]="";
            break;
        case 1:
            (*mRoot)[mColumns.mColIcon]="*";
            break;
        case 2:
            (*mRoot)[mColumns.mColIcon]="(!)";
            break;
        }

        return alarm;
    }

protected:
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    std::vector<GuiRawData*> mArray;
    Gtk::TreeModel::Row *mRoot;

    ModelColumns mColumns;
}; 

#endif
