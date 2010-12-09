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
        add(mColName);
        add(mColValue);
        add(mColIcon);
    }

    Gtk::TreeModelColumn<Glib::ustring> mColName;
    int mColNameID;
    Gtk::TreeModelColumn<Glib::ustring> mColValue;
    int mColValueID;
    Gtk::TreeModelColumn<Glib::RefPtr<Gdk::Pixbuf> > mColIcon;
    int mColIconID;
};

///////////////////////////////////////////////////

class iCubInterfaceGuiRows
{
public:
    iCubInterfaceGuiRows()
    {
        mRows=NULL;
    }

    Gtk::TreeModel::Row* createRows(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,char *rowNames[])
    {
        int numRows=0;

        for (; rowNames[numRows]; ++numRows)
        {
        }

        mRows=new Gtk::TreeModel::Row[numRows];

        mRows[0]=*(refTreeModel->append(parent.children()));
        mRows[0][mColumns.mColName]=rowNames[0];
        mRows[0][mColumns.mColValue]="";

        for (int i=1; i<numRows; ++i)
        {
            mRows[i]=*(refTreeModel->append(mRows[0].children()));
            mRows[i][mColumns.mColName]=rowNames[i];
            mRows[i][mColumns.mColValue]="";
        }

        return &mRows[0];
    }

    virtual ~iCubInterfaceGuiRows()
    {
        if (mRows!=NULL) delete [] mRows;
    }

    //Gtk::TreeModel::Row* getBase(){ return &mRows[0]; }

protected:
    int mNumRows;
    Gtk::TreeModel::Row *mRows;
    ModelColumns mColumns;
};

class iCubBLLChannelGui : public iCubBLLChannel, public iCubInterfaceGuiRows
{
public:
    iCubBLLChannelGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot)
        : iCubBLLChannel(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        //mColumns=&modelColumns;

        mData.fromBottle(bot);
        
        for (int i=0; i<(int)mData.size(); ++i)
        {
            mRows[i][mColumns.mColValue]=mData.toString(i);
        }
    }

    virtual ~iCubBLLChannelGui()
    {
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubBLLChannel::fromBottle(bot);

        for (int i=0; i<mData.size(); ++i)
        {
            if (mData.test(i))
            {
                mRows[i][mColumns.mColValue]=mData.toString(i);
            }
        }
    }

protected:
};

#endif
