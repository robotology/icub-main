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
    iCubNetworkGui(Glib::RefPtr<Gtk::TreeStore> refTreeModel,Gtk::TreeModel::Row& parent,yarp::os::Bottle &bot)
        : iCubNetwork(),iCubInterfaceGuiRows()
    {
        createRows(mRefTreeModel,parent,mRowNames);

        yarp::os::Bottle *boardBot=bot.get(6).asList();

        this->addBoard(new iCubBLLBoardGui(mRefTreeModel,parent,*boardBot));

        ////////////////////////////////////////////////////////////////

        fromBottle(bot);
    }

    virtual ~iCubNetworkGui()
    {
    }

    virtual void fromBottle(yarp::os::Bottle &bot)
    {
        iCubNetwork::fromBottle(bot);

        for (int i=0; i<(int)mData.size(); ++i)
        {
            if (mData.test(i))
            {
                mRows[i][mColumns.mColValue]=mData.toString(i);
            }
        }
    }

protected:
    Gtk::ScrolledWindow mScrolledWindow;
    Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;
};

#endif
