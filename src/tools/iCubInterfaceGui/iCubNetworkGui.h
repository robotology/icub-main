// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
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
    iCubNetworkGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent)
        : iCubNetwork(),iCubInterfaceGuiRows()
    {
        int numRows=(int)DOUBLE_NUM+(int)BOOL_NUM+(int)INT_NUM;
        createRows(mRefTreeModel,parent,mRowNames,numRows);
    }

    virtual ~iCubNetworkGui()
    {
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubNetwork::fromBottle(bot);

        double d;
        for (int i=0; i<(int)DOUBLE_NUM; ++i)
        {
            if (iCubNetwork::mDoubleData.read(i,d))
            {
                mRows[i][mColumns.mColValue]=toString(d);
            }
        }

        bool b;
        for (int i=0; i<(int)BOOL_NUM; ++i)
        {
            if (iCubNetwork::mBoolData.read(i,b))
            {
                mRows[i+(int)DOUBLE_NUM][mColumns.mColValue]=toString(b);
            }
        }

        int k;
        for (int i=0; i<(int)INT_NUM; ++i)
        {
            if (iCubNetwork::mIntData.read(i,k))
            {
                mRows[i+(int)DOUBLE_NUM+(int)BOOL_NUM][mColumns.mColValue]=toString(k);
            }
        }
    }

protected:
    Gtk::ScrolledWindow mScrolledWindow;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;
};

#endif
