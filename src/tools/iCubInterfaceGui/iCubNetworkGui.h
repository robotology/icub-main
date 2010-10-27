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
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        for (int i=1; i<(int)bot.size(); ++i)
        {
            yarp::os::Bottle *netBot=bot.get(i).asList();

            //if (netBot->get(1).asString()=="BLL")
            {
                mBoards.push_back(new iCubBLLBoardGui(refTreeModel,*baseRow,*netBot));
            }
            /*
            else
            {
                will deal with other board models
            }
            */
        }

        mData.fromBottle(*(bot.get(0).asList()));
        for (int i=0; i<(int)mData.size(); ++i)
        {
            mRows[i][mColumns.mColValue]=mData.toString(i);
        }
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
    //Gtk::ScrolledWindow mScrolledWindow;
    //Glib::RefPtr<Gtk::TreeStore> mRefTreeModel;
    ModelColumns mColumns;
};

#endif
