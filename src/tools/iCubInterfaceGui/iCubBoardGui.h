// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_BOARD_GUI_H__
#define __GTKMM_ICUB_BOARD_GUI_H__

#include <gtkmm.h>
#include <yarp/os/RateThread.h>
#include "iCubBoard.h"
#include "iCubBoardChannelGui.h"

class iCubBLLBoardGui : public iCubBLLBoard, public iCubInterfaceGuiRows
{
public:
    iCubBLLBoardGui() : iCubBLLBoard(-1,-1,-1)
    {
    }

    virtual ~iCubBLLBoardGui()
    {
    }
};

#endif
