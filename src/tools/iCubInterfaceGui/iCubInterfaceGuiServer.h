// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#ifndef __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__
#define __GTKMM_ICUB_INTERFACE_GUI_SERVER_H__

#include <yarp/os/Property.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Port.h>
#include <yarp/os/Semaphore.h>

class iCubInterfaceGuiServer : public yarp::os::Thread, public iCubInterfaceGui
{
public:
    iCubInterfaceGuiServer(yarp::os::Property &robot)
    {
        yarp::os::ConstString robotName=robot.find("name").asString();

        yarp::os::Bottle general=robot.findGroup("GENERAL");
        yarp::os::Bottle *parts=general.find("parts").asList();

        for (int t=0; t<parts->size(); ++t)
        {
            yarp::os::ConstString partName=parts->get(t).asString();
            yarp::os::Bottle part=robot.findGroup(partName.c_str());
            
            yarp::os::Bottle *networks=part.find("networks").asList();

            for (int n=0; n<networks->size(); ++n)
            {
                yarp::os::ConstString netName=networks->get(n).asString();
                yarp::os::Bottle net=robot.findGroup(netName.c_str());

                addNetwork(new iCubNetwork(std::string(netName.c_str()),
                                   std::string(net.find("file").asString().c_str()),
                                   std::string(net.find("device").asString().c_str()),
                                   std::string(net.find("canbusdevice").asString().c_str()),
                                   part.find("threadrate").asInt()));
            }
        }

        // we have now the networks list

        for (unsigned int n=0; n<m_networks.size(); ++n)
        {
            yarp::os::Property netConf;
            netConf.fromConfigFile(m_networks[n]->file.c_str());
            yarp::os::Bottle canConf=netConf.findGroup("CAN");

            m_networks[n]->config(canConf.find("CanDeviceNum").asInt());
        
            yarp::os::Bottle devices=canConf.findGroup("CanAddresses");
            
            for (int d=1; d<devices.size(); ++d)
            {
                m_networks[n]->addBoard(new iCubBLL(devices.get(d).asInt()));
            }
        }

        m_port.open((robotName+"/gui").c_str());
    }

    virtual ~iCubInterfaceGui()
    {
    }

    void run()
    {
        yarp::os::Bottle msg;
        yarp::os::Bottle rpl;

        while (!isStopping())
        {
            m_port.read(msg,true);
            yarp::os::ConstString cmd=msg.get(0).asString();

            if (cmd=="get_conf")
            {
                rpl.clear();
                m_mutex.wait();
                for (unsigned int n=0; n<m_networks.size(); ++n)
                {
                    // ...
                    // tutto cambiato
                    // ...
                }    
                m_mutex.post();

                m_port.reply(rpl);
            }
            else if (cmd=="get_data")
            {
                m_mutex.wait();
                
                // ...
                // tutto cambiato
                // ...

                m_mutex.post();
            }
        } 
    }

    yarp::os::Bottle toBottle();

    bool write(std::string& address,std::string& data)
    {
        m_mutex.wait();
        for (unsigned int n=0; n<m_networks.size(); ++n)
        {
            if (m_networks[n]->write(address,data)
            {
                m_mutex.post();
                return true;
            }
        }
        m_mutex.post();
        return false;
    }

protected:
    yarp::os::Port m_port;
    yarp::os::Semaphore m_mutex;
};

#endif //__GTKMM_ICUB_INTERFACE_GUI_H__
