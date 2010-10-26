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
        : iCubBLLBoard(),iCubInterfaceGuiRows()
    {
        Gtk::TreeModel::Row* baseRow=createRows(refTreeModel,parent,mRowNames);

        fromBottle(*(bot.get(0).asList()));

        mChannel[0]=new iCubBLLChannelGui(refTreeModel,*baseRow,*(bot.get(1).asList()));
        mChannel[1]=new iCubBLLChannelGui(refTreeModel,*baseRow,*(bot.get(2).asList()));
    }

    virtual void fromBottle(yarp::os::Bottle& bot)
    {
        iCubBLLBoard::fromBottle(bot);

        for (int i=0; i<(int)mData.size(); ++i)
        {
            if (mData.test(i))
            {
                mRows[i][mColumns.mColValue]=mData.toString(i);
            }
        }
    }

    virtual ~iCubBLLBoardGui()
    {
    }

protected:
    static char* mRowNames[0];
};

#endif
