// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Robotcub Consortium
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <yarp/os/Time.h>
#include "iCubInterfaceGuiServer.h"

class JointRemapper
{
public:
    JointRemapper(){}
    ~JointRemapper(){}

    void push(int d0,int d1,int j0)
    {
        mapD0.push_back(d0);
        mapD1.push_back(d1);
        mapJ0.push_back(j0);
    }

    int getJoint(int pos)
    {
        pos*=2;

        for (int i=0; i<(int)mapD0.size(); ++i)
        {
            if (pos>=mapD0[i] && pos<=mapD1[i])
            {
                return mapJ0[i]+pos-mapD0[i];
            }
        }

        return -1;
    }

protected:
    std::vector<int> mapD0,mapD1,mapJ0;
};

void iCubInterfaceGuiServer::config(yarp::os::Property &robot)
{
    std::vector<JointRemapper> jointRmp;

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
            std::string netName(networks->get(n).asString().c_str());

            yarp::os::Bottle jMap=part.findGroup(netName.c_str());

            int j0=jMap.get(1).asInt();
            int d0=jMap.get(3).asInt();
            int d1=jMap.get(4).asInt();

            yarp::os::Bottle net=robot.findGroup(netName.c_str());

            bool bExists=false;

            for (unsigned int i=0; i<mNetworks.size(); ++i)
            {
                if (mNetworks[i]->mName==netName)
                {
                    jointRmp[i].push(d0,d1,j0);
                    bExists=true;
                    break;
                }
            }

            if (!bExists)
            {
                std::string file(net.find("file").asString().c_str());
                std::string device(net.find("canbusdevice").asString().c_str());
                mNetworks.push_back(new iCubNetwork(netName,file,device));

                jointRmp.push_back(JointRemapper());
                jointRmp.back().push(d0,d1,j0);
            }
        }
    }

    // we have now the networks list

    for (unsigned int n=0; n<mNetworks.size(); ++n)
    {
        yarp::os::Property netConf;
        netConf.fromConfigFile(mNetworks[n]->mFile.c_str());
        yarp::os::Bottle canConf=netConf.findGroup("CAN");
    
        mNetworks[n]->setID(canConf.find("CanDeviceNum").asInt());
    
        yarp::os::Bottle devices=canConf.findGroup("CanAddresses");
        devices=devices.tail();
    
        for (int d=0; d<devices.size(); ++d)
        {
            int joint=jointRmp[n].getJoint(d);
            mNetworks[n]->addBoard(new iCubBLLBoard(devices.get(d).asInt(),joint,joint+1));
        }
    }

    mPort.open("/icubinterfacegui/server");
}

void iCubInterfaceGuiServer::run()
{
    yarp::os::Bottle msg;
    yarp::os::Bottle rpl;

    while (!isStopping())
    {
        if (mPort.read(msg,true))
        {
            yarp::os::ConstString cmd=msg.get(0).asString();

            if (cmd=="GET_CONF")
            {
                mMutex.wait();
                rpl=toBottle(true);
                mMutex.post();

                //printf("%s\n",rpl.toString().c_str());
                //fflush(stdout);

                mPort.reply(rpl);
            }
            else if (cmd=="GET_DATA")
            {
                mMutex.wait();

                rpl=toBottle();

                //printf("%s\n",rpl.toString().c_str());

                mPort.reply(rpl);

                mMutex.post();
            }
        }
        else
        {
            yarp::os::Time::delay(1.0);
        }
    } 
}

bool iCubInterfaceGuiServer::log(std::string &key,yarp::os::Value &data)
{
    mMutex.wait();
    for (unsigned int n=0; n<(int)mNetworks.size(); ++n)
    {
        if (mNetworks[n]->findAndWrite(key,data))
        {
            mMutex.post();
            return true;
        }
    }
    mMutex.post();
    return false;
}

bool iCubInterfaceGuiServer::findAndRead(std::string address,yarp::os::Value* data)
{
    mMutex.wait();
    for (unsigned int n=0; n<(int)mNetworks.size(); ++n)
    {
        if (mNetworks[n]->findAndRead(address,data))
        {
            mMutex.post();
            return true;
        }
    }
    mMutex.post();
    return false;
}

yarp::os::Bottle iCubInterfaceGuiServer::toBottle(bool bConfig)
{
    yarp::os::Bottle bot;

    bot.addString("DATA");

    for (int n=0; n<(int)mNetworks.size(); ++n)
    {   
        yarp::os::Bottle net=mNetworks[n]->toBottle(bConfig);

        if (net.size())
        {
            yarp::os::Bottle &addList=bot.addList();
            addList.addInt(n);
            addList.append(net);
        }
    }

    return bot;
}

