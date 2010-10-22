// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__
#define __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__

#include "iCubNetwork.h"
#include <yarp/os/Port.h>
#include <yarp/os/Semaphore.h>

class iCubInterfaceGuiServer : public yarp::os::Thread
{
public:
    iCubInterfaceGuiServer()
    {
    }
    
    virtual ~iCubInterfaceGuiServer()
    {
        mPort.close();

        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i]!=NULL) delete mNetworks[i];
        }
    }

    void config(yarp::os::Property &robot);
    void run();
    bool findAndWrite(std::string address,yarp::os::Value* data);
    yarp::os::Bottle toBottle(bool bConfig=false);

protected:
    yarp::os::Port mPort;
    yarp::os::Semaphore mMutex;
    std::vector<iCubNetwork*> mNetworks;
};

#endif
