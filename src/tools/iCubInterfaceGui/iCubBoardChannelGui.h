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

inline std::string toString(int i)
{
    char buff[16];
    sprintf(buff,"%d",i);
    return std::string(buff);
}

inline std::string toString(bool b)
{
    return b ? std::string("true") : std::string("false");
}

inline std::string toString(double d)
{
    char buff[64];
    sprintf(buff,"%f",d);
    return std::string(buff);
}

inline std::string toString(char *s)
{
    return std::string(s);
}

//Tree model columns
class ModelColumns : public Gtk::TreeModel::ColumnRecord
{
public:
    ModelColumns()
    {
        add(mColName);
        add(mColValue);
        add(mColStatus);
    }

    Gtk::TreeModelColumn<Glib::ustring> mColName;
    Gtk::TreeModelColumn<Glib::ustring> mColValue;
    Gtk::TreeModelColumn<Glib::RefPtr<Gdk::Pixbuf> > mColStatus;
};

///////////////////////////////////////////////////

class iCubInterfaceGuiRows
{
public:
    iCubInterfaceGuiRows()
    {
        mRows=NULL;
    }

    void createRows(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,char *rowNames[],int numRows)
    {
        mRows=new Gtk::TreeModel::Row[numRows=mNumRows];

        mRows[0]=*(refTreeModel->append(parent.children()));
        mRows[0][mColumns.mColName]=rowNames[0];
        mRows[0][mColumns.mColValue]="";

        for (int i=1; i<numRows; ++i)
        {
            mRows[i]=*(refTreeModel->append(mRows[0].children()));
            mRows[i][mColumns.mColName]=rowNames[i];
            mRows[i][mColumns.mColValue]="";
        }
    }

    virtual ~iCubInterfaceGuiRows()
    {
        if (mRows!=NULL) delete [] mRows;
    }

protected:
    int mNumRows;
    Gtk::TreeModel::Row *mRows;
    ModelColumns mColumns;
};

class iCubBLLChannelGui : public iCubBLLChannel, public iCubInterfaceGuiRows
{
public:
    iCubBLLChannelGui() : iCubBLLChannel(-1,-1)
    {
        int numRows=(int)iCubBLLChannel::DOUBLE_NUM+(int)iCubBLLChannel::BOOL_NUM+(int)iCubBLLChannel::INT_NUM;
        //createRows(mRefTreeModel,NULL,iCubBLLChannel::mRowNames,numRows);
    }

    virtual ~iCubBLLChannelGui()
    {
        delete [] mRows;
    }

    void fromBottle(yarp::os::Bottle &bot)
    {
        iCubBLLChannel::fromBottle(bot);

        double d;
        for (int i=0; i<(int)DOUBLE_NUM; ++i)
        {
            if (iCubBLLChannel::mDoubleData.read(i,d))
            {
                mRows[i][mColumns.mColValue]=toString(d);
            }
        }

        bool b;
        for (int i=0; i<(int)BOOL_NUM; ++i)
        {
            if (iCubBLLChannel::mBoolData.read(i,b))
            {
                mRows[i+(int)DOUBLE_NUM][mColumns.mColValue]=toString(b);
            }
        }

        int k;
        for (int i=0; i<(int)INT_NUM; ++i)
        {
            if (iCubBLLChannel::mIntData.read(i,k))
            {
                mRows[i+(int)DOUBLE_NUM+(int)BOOL_NUM][mColumns.mColValue]=toString(k);
            }
        }
    }

protected:
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
};

#endif
