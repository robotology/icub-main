// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2010 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__
#define __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__

#include "iCubNetwork.h"
#include <yarp/os/all.h>
#include <iCub/LoggerInterfaces.h>

class iCubInterfaceGuiServer : public yarp::os::Thread, public yarp::dev::IServerLogger
{
public:
    iCubInterfaceGuiServer() : mMutex(1)
    {
    }
    
    virtual ~iCubInterfaceGuiServer()
    {
        stop();
    }

    void config(std::string& PATH,yarp::os::Property &robot);

    void run();

    bool stop()
    {
        mPort.interrupt();
        mPort.close();

		askStop=true;
        bool ret=yarp::os::Thread::stop();

        for (int i=0; i<(int)mNetworks.size(); ++i)
        {
            if (mNetworks[i]!=NULL)
            {
                delete mNetworks[i];
                mNetworks[i]=NULL;
            }
        }

        return ret;
    }
    
    bool log(const std::string &key,const yarp::os::Value &data);
    bool log(const yarp::os::Bottle &data){ return false; }

    yarp::os::Bottle getConfig()
    {
        yarp::os::Bottle config;

        for (int n=0; n<(int)mNetworks.size(); ++n)
        {   
            config.addList()=mNetworks[n]->getConfig();
        }

        return config;
    }

    yarp::os::Bottle toBottle()
    {
        yarp::os::Bottle data;

        for (int n=0; n<(int)mNetworks.size(); ++n)
        {
            yarp::os::Bottle netData=mNetworks[n]->toBottle();

            if (netData.size())
            {
                yarp::os::Bottle &addList=data.addList();
                addList.addInt(n);
                addList.append(netData);
            }
        }

        return data;
    }

protected:
    yarp::os::RpcServer mPort;
    yarp::os::Semaphore mMutex;
    std::vector<iCubNetwork*> mNetworks;
	bool askStop;
};

#endif

