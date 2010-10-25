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
        : iCubBLLBoard(-1,-1,-1),iCubInterfaceGuiRows()
    {
        createRows(refTreeModel,parent,mRowNames);

        yarp::os::Bottle *chanBot0=bot.get(3).asList();
        mChannel[0]->fromBottle(*chanBot0);
        yarp::os::Bottle *chanBot1=bot.get(4).asList();
        mChannel[1]->fromBottle(*chanBot1);

        fromBottle(bot);
    }

    void fromBottle(yarp::os::Bottle& bot)
    {
        iCubBLLBoard::fromBottle(bot);
        
        if (bot.get(0).asInt()==CONFIG_FLAG)
        {
            mID=bot.get(2).asInt();
            mRows[0][mColumns.mColValue]=bot.get(1).asString().c_str();
            mRows[1][mColumns.mColValue]=bot.get(2).toString().c_str();
        }
    }

    virtual ~iCubBLLBoardGui()
    {
    }

protected:
    static char* mRowNames[0];
};

#endif
